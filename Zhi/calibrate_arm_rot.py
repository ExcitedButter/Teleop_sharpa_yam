#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Measure the tracker->model WRIST ROTATION map by rotating the ARM - the
rotation twin of calibrate_arm_axes.py, and the fix for hand-calibration's
low quality (free-hand rotations wobble ~25 deg off-axis; the arm's are
mathematically pure, so quality comes out ~0.95+ with no direction guessing).

Strap/tape a Vive tracker RIGIDLY to this arm's wrist. The script rotates the
EE +/-20 deg about each model axis (x roll, y pitch, z yaw) at its CURRENT
pose - position pinned, wrist only - records how the tracker rotates in the
stream frame, builds the exact rotation map, and saves it to
vive_arm_map.json["<side>_rot"] (manus_arm_teleop uses it automatically).

RUN AT THE READY POSE (press "Reset arms" first - the folded home blocks
half the rotation space), followers UP, teleop STOPPED:

  ./calibrate_arm_rot.py --side left        # then move tracker, --side right

The wrist sweeps ~20 deg per axis and returns. Clear the space, e-stop ready.
"""
import argparse
import json
import os
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "i2rt"))
import portal                                             # noqa: E402
from manus_arm_teleop import ArmIK, ssik_ik, SIDE_PORT    # noqa: E402
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml  # noqa: E402
from ssik.prebuilt.i2rt import yam_ik                     # noqa: E402
from scipy.spatial.transform import Rotation as SciRot    # noqa: E402

MAP_FILE = os.path.join(HERE, "params", "vive_arm_map.json")
STREAM_PORT = {"left": 9873, "right": 9874}
ROT_DEG = 20.0
AXES = ["x (roll)", "y (pitch)", "z (yaw)"]


def snap_rots(rx, secs=1.0):
    """side -> mean tracker rotation over `secs` (valid trackers only)."""
    qs = {}
    t_end = time.time() + secs
    while time.time() < t_end:
        try:
            data, _ = rx.recvfrom(65536)
        except socket.timeout:
            continue
        try:
            msg = json.loads(data.decode())
        except ValueError:
            continue
        for g in msg.get("gloves", []):
            if g.get("valid") and "q" in g and g.get("side"):
                qs.setdefault(g["side"], []).append(g["q"])
    out = {}
    for s, v in qs.items():
        q = np.asarray(v)
        out[s] = SciRot.from_quat(
            np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])).mean()
    return out


def goto(client, q_from, q_to, rate=50.0, max_step=0.02):
    q = np.array(q_from, float)
    while np.max(np.abs(q_to - q)) > 1e-4:
        q = q + np.clip(q_to - q, -max_step, max_step)
        client.command_joint_pos(q)
        time.sleep(1.0 / rate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", required=True, choices=["left", "right"])
    ap.add_argument("--tracker", choices=["left", "right"], default=None,
                    help="which tracker is strapped on (default: auto = the "
                         "one that rotates with the arm)")
    ap.add_argument("--server-host", default="192.168.1.9")
    ap.add_argument("--deg", type=float, default=ROT_DEG)
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", STREAM_PORT[args.side]))
    rx.settimeout(0.3)
    if not snap_rots(rx, 1.0):
        print("no valid tracker on the stream - vive_wrist_stream running? "
              "tracker green in SteamVR?")
        return 1

    print("connecting to %s follower ..." % args.side)
    client = portal.Client("%s:%d" % (args.server_host, SIDE_PORT[args.side]))
    q0 = np.asarray(client.get_joint_pos().result(), dtype=np.float64)[:6]
    ik = ArmIK(combine_arm_and_gripper_xml(ArmType.YAM_ULTRA,
                                           GripperType.NO_GRIPPER), "grasp_site")
    T0 = ik.fk(q0)
    R0, p0 = T0[:3, :3].copy(), T0[:3, 3].copy()
    T_g2l = np.linalg.inv(T0) @ np.asarray(yam_ik.fk(q0), float)

    # preflight: all six +/-deg wrist rotations must be reachable from here
    plans = []
    for i in range(3):
        pair = []
        for sign in (1.0, -1.0):
            ax = np.zeros(3); ax[i] = 1.0
            dR = SciRot.from_rotvec(np.radians(args.deg * sign) * ax).as_matrix()
            Tg = np.eye(4); Tg[:3, :3] = dR @ R0; Tg[:3, 3] = p0
            ok, qt = ssik_ik(Tg @ T_g2l, q0, ik.jrange)
            if not ok or np.max(np.abs(qt - q0)) > 1.2:
                print("model %s %+g deg is NOT reachable from the current pose "
                      "- press 'Reset arms' (ready pose) and re-run"
                      % (AXES[i], args.deg * sign))
                return 1
            pair.append(qt)
        plans.append(pair)

    if not args.yes:
        input("\nTracker strapped to the %s arm's wrist? The WRIST will rotate "
              "+/-%.0f deg about 3 axes and return. Clear the space, e-stop "
              "ready, then ENTER... " % (args.side, args.deg))

    cols, used = [], []
    for i, (q_plus, q_minus) in enumerate(plans):
        r_a = snap_rots(rx)
        print("model %s: rotating ..." % AXES[i])
        goto(client, q0, q_plus)
        time.sleep(0.8)
        r_b = snap_rots(rx)
        goto(client, q_plus, q0)
        time.sleep(0.5)
        cands = {}
        for s in r_a:
            if s in r_b and (args.tracker is None or s == args.tracker):
                rv = (r_b[s] * r_a[s].inv()).as_rotvec()
                cands[s] = rv
        if not cands:
            print("tracker lost during the %s rotation - re-run" % AXES[i])
            return 1
        s_best = max(cands, key=lambda s: np.linalg.norm(cands[s]))
        rv = cands[s_best]
        mag = float(np.degrees(np.linalg.norm(rv)))
        print("  '%s' tracker rotated %.1f deg about %s"
              % (s_best, mag, np.round(rv / np.linalg.norm(rv), 3)))
        if mag < args.deg * 0.5:
            print("  rotation too small - tracker strapped to THIS arm? rigid?")
            return 1
        used.append(s_best)
        cols.append(rv / np.linalg.norm(rv))
    if len(set(used)) != 1:
        print("inconsistent tracker detection %s - re-run" % used)
        return 1

    U = np.array(cols).T
    W, S, Vt = np.linalg.svd(U)
    R = W @ Vt
    M = R.T
    det = float(np.linalg.det(R))
    orth = float(np.min(S) / np.max(S))
    print("\nmeasured WRIST ROTATION map (stream->model), det=%+.2f, "
          "quality %.2f:" % (det, orth))
    print(np.round(M, 3))
    if orth < 0.9:
        print("WARNING: quality below 0.9 for an arm-driven calibration - "
              "the tracker may not be rigidly attached. Consider re-running.")
    try:
        m = json.load(open(MAP_FILE))
    except Exception:
        m = {}
    key = args.side + "_rot"
    m[key] = [[float(v) for v in row] for row in M]
    json.dump(m, open(MAP_FILE, "w"), indent=1)
    print("saved -> %s ['%s']  (Stop+Start Teleop to load)" % (MAP_FILE, key))
    return 0


if __name__ == "__main__":
    sys.exit(main())
