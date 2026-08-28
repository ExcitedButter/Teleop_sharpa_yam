#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Measure one arm's tracker->model axis map DIRECTLY - no sign guessing.

Strap/tape a Vive tracker to THIS arm's wrist (rigidly - a rubber band works).
The script jogs the arm a few cm along each of its model axes (+x, +y, +z) and
records how the tracker actually moves in the tracking frame. From those three
displacement vectors it builds the exact stream->model mapping M for this arm,
INCLUDING any mirror (a mirrored mount shows up as det(M) = -1), and saves it
to vive_arm_map.json. manus_arm_teleop --frame world then uses the measured M
and ignores all the yaw/base-yaw/flip heuristics for this side.

Run per side, with the FOLLOWERS UP and teleop STOPPED (Stop Teleop first):

  /home/yam/gck/i2rt/.venv/bin/python /home/yam/manus/calibrate_arm_axes.py --side left
  /home/yam/gck/i2rt/.venv/bin/python /home/yam/manus/calibrate_arm_axes.py --side right

The arm moves ~8 cm per axis and returns. Clear the workspace, hand on e-stop.
Re-run whenever the base station is moved (the map is station-relative).
"""
import argparse, json, os, socket, sys, time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "i2rt"))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import portal
from manus_arm_teleop import ArmIK, ssik_ik, SIDE_PORT
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml
from ssik.prebuilt.i2rt import yam_ik

MAP_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "vive_arm_map.json")
STREAM_PORT = {"left": 9873, "right": 9874}


def snap_all(rx, secs=0.8):
    """Average BOTH trackers' positions over `secs`; {side: pos} for valid ones."""
    pts = {}
    t_end = time.time() + secs
    while time.time() < t_end:
        try:
            data, _ = rx.recvfrom(65536)
        except socket.timeout:
            continue
        try:
            msg = json.loads(data.decode())
        except Exception:
            continue
        for g in msg.get("gloves", []):
            if g.get("valid") and "p" in g and g.get("side"):
                pts.setdefault(g["side"], []).append(g["p"])
    return {s: np.mean(np.asarray(v), axis=0) for s, v in pts.items()}


def goto(client, q_from, q_to, rate=50.0, max_step=0.02):
    """Slow clamped interpolation to q_to (same style as teleop's step clamp)."""
    q = np.array(q_from, float)
    while np.max(np.abs(q_to - q)) > 1e-4:
        q = q + np.clip(q_to - q, -max_step, max_step)
        client.command_joint_pos(q)
        time.sleep(1.0 / rate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", required=True, choices=["left", "right"],
                    help="which arm (panel port label) to jog and calibrate")
    ap.add_argument("--tracker", choices=["left", "right"], default=None,
                    help="which tracker is strapped to it (default: auto-detect "
                         "= whichever tracker actually moves with the jogs)")
    ap.add_argument("--server-host", default="192.168.1.9")
    ap.add_argument("--delta", type=float, default=0.08, help="jog distance, m")
    ap.add_argument("--yes", action="store_true", help="skip the ENTER prompt")
    args = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", STREAM_PORT[args.side]))
    rx.settimeout(0.3)
    if not snap_all(rx, 1.0):
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

    if not args.yes:
        input("\nA tracker strapped to the %s arm's wrist? The arm will jog "
              "~%.0f cm along 3 axes and return. Clear the space, hand on "
              "e-stop, then ENTER... " % (args.side, args.delta * 100))

    cols, used_sides = [], []
    for i, name in enumerate(["x (arm axis)", "y", "z (up)"]):
        d = np.zeros(3)
        d[i] = args.delta
        Tg = np.eye(4)
        Tg[:3, :3] = R0
        Tg[:3, 3] = p0 + d
        ok, qt = ssik_ik(Tg @ T_g2l, q0, ik.jrange)
        if not ok:
            print("model %s: IK failed for +%.2f m - trying half" % (name, args.delta))
            Tg[:3, 3] = p0 + d / 2
            ok, qt = ssik_ik(Tg @ T_g2l, q0, ik.jrange)
            if not ok:
                print("model %s: unreachable - aborting" % name)
                return 1
        p_a = snap_all(rx)
        print("model +%s: jogging ..." % name)
        goto(client, q0, qt)
        time.sleep(0.8)
        p_b = snap_all(rx)
        goto(client, qt, q0)
        time.sleep(0.4)
        # the tracker ON the arm is whichever one moved with the jog
        cands = {}
        for s in p_a:
            if s in p_b and (args.tracker is None or s == args.tracker):
                cands[s] = np.asarray(p_b[s]) - np.asarray(p_a[s])
        if not cands:
            print("tracker lost during the %s jog - keep it visible and re-run" % name)
            return 1
        s_best = max(cands, key=lambda s: np.linalg.norm(cands[s]))
        u, n = cands[s_best], float(np.linalg.norm(cands[s_best]))
        print("  '%s' tracker moved %s  (|%.3f| m)" % (s_best, np.round(u, 3), n))
        if n < 0.03:
            print("  motion too small - is a tracker strapped to the %s arm?"
                  % args.side)
            return 1
        used_sides.append(s_best)
        cols.append(u / n)
    if len(set(used_sides)) != 1:
        print("inconsistent tracker detection across jogs %s - re-run" % used_sides)
        return 1
    print("(used the '%s' tracker)" % used_sides[0])

    U = np.array(cols).T          # columns = stream direction of each model axis
    W, S, Vt = np.linalg.svd(U)
    R = W @ Vt                    # nearest orthogonal, PRESERVES the mirror sign
    M = R.T                       # stream -> model
    det = float(np.linalg.det(R))
    orth = float(np.min(S) / np.max(S))
    print("\nmeasured map M (stream->model), det=%+.2f%s, axis quality %.2f:"
          % (det, "  [MIRRORED mount]" if det < 0 else "", orth))
    print(np.round(M, 3))
    if orth < 0.7:
        print("WARNING: axes are far from orthogonal - tracker may have "
              "slipped. Consider re-running.")

    try:
        m = json.load(open(MAP_FILE))
    except Exception:
        m = {}
    m[args.side] = [[float(v) for v in row] for row in M]
    json.dump(m, open(MAP_FILE, "w"), indent=1)
    print("saved -> %s ['%s']  (teleop uses it automatically)" % (MAP_FILE, args.side))
    return 0


if __name__ == "__main__":
    sys.exit(main())
