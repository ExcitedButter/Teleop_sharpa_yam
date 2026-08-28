#!/usr/bin/env python3
"""Manus glove wrist -> yam arm teleop (orientation-driven IK), one side per process.

Reads the Manus wrist ORIENTATION quaternion from manus_skeleton_stream.out
(udp/9872, per-glove {"side","q":[w,x,y,z]}), and streams 6 IK'd joint targets
to the follower yam arm's portal RPC server -- exactly the channel the leader
used (ClientRobot in examples/minimum_gello/minimum_gello.py). The leader is
retired; the glove drives the wrist, the Manus finger bridge drives the hand.

IK: ssik's analytical closed-form solver for the yam (ssik.prebuilt.i2rt.yam_ik,
Raghavan-Roth, machine-precision FK closure, all branches). It fully pins the EE
POSE (position + orientation) -- exact, no convergence/drift issues. We keep the
old damped-least-squares Jacobian solver (mujoco.mj_jacSite, orientation-only)
as a FALLBACK for when ssik returns no in-limit branch, and select the ssik
branch nearest q_prev for continuity. NOTE: i2rt's kinematics.ik (mink) is NOT
used -- it fails to converge in this env even on its own example (2026-08-21).

ssik solves in the link_6 frame, but R_ALIGN / the wrist-direction tuning were
validated in the grasp_site frame, so we build the target in grasp_site and
convert to link_6 by the constant offset T_g2l = inv(fk_grasp(q0))@fk_link6(q0),
holding the grasp point fixed (tool-tip pivot). This preserves ALL prior tuning.

Design (see ~/.claude/plans/valiant-weaving-teacup.md):
  * CAPTURE-READY at start: q0 = follower's current joints, T0 = fk(q0) anchor,
    Wq0 = glove neutral wrist. First frame target == T0 -> no jump.
  * per frame: dR = R(Wq0)^T R(Wq_now) (wrist rotation since neutral), mapped
    into the EE frame by R_ALIGN, applied to T0's rotation. POSITION: held at T0
    for glove sources (no absolute position); with the Vive tracker stream
    (vive_wrist_stream.py, "p" in meters) and --pos-scale != 0 the EE tracks
    p0 + pos_scale*(p - p_neutral), clamped to --max-offset around the start.
  * safety: per-joint max step clamp + joint-range clamp; hold on IK failure;
    refuse to start if the follower port is down. --dry-run never commands.

Run in the i2rt venv:
  /home/yam/gck/i2rt/.venv/bin/python manus_arm_teleop.py --side right --dry-run
"""
import argparse, json, os, signal, socket, sys, time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "i2rt"))
import portal
import mujoco
from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

try:                                    # analytical yam IK (primary); DLS is fallback
    from ssik.prebuilt.i2rt import yam_ik
except Exception as _e:                 # pragma: no cover
    yam_ik = None
    print(f"[warn] ssik unavailable ({_e}); using DLS Jacobian IK only", file=sys.stderr)

SIDE_PORT = {"left": 11334, "right": 11333}   # follower ports (panel SIDES)

# Per-side reset/home pose (radians), persisted so the two hands can be made to
# match. The RIGHT arm is mounted mirrored vs the LEFT, so identical joints give
# different world orientations; MIRROR_SIGN negates the joints that flip under a
# left<->right reflection (base yaw + the two wrist rolls). Tune the signs if a
# joint mirrors the wrong way. `--mirror-home` captures the LEFT arm's current
# pose and writes right = MIRROR_SIGN * left so their orientations match.
HOME_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "arm_home.json")
MIRROR_SIGN = np.array([-1.0, 1.0, 1.0, -1.0, 1.0, -1.0])

def load_home(side):
    try:
        d = json.load(open(HOME_FILE))
        return np.asarray(d[side], dtype=np.float64)[:6]
    except Exception:
        return np.zeros(6)

# Stiffness model (2026-08-21):
#   --reset : smoothly drive to HOME (joints 0), stiff, exit.  (Start & Stop use this.)
#   --loose : drop the follower to gravity-comp (kp=kd=0) so the arm is LOOSE and
#             backdrivable, then exit.  (ONLY the panel "Shut down" button uses this.)
#   normal  : live wrist teleop, STIFF. On SIGTERM it just exits cleanly and the
#             follower keeps holding its last pose (stiff) - Stop then resets to home.
# All via the follower's EXISTING command_joint_state RPC; no yambox code change.

def send_loose(client, q):
    """Drop the follower to gravity compensation only (loose/backdrivable)."""
    client.command_joint_state({"pos": np.asarray(q, float), "vel": np.zeros(6),
                                "kp": np.zeros(6), "kd": np.zeros(6)})
    time.sleep(0.3)   # let the RPC land

def _clean_exit(*_):
    sys.exit(0)

# Per-side wrist-frame correction. MANUS reports both gloves' wrist orientation
# in ONE world frame, but the two yam arms are MIRROR-MOUNTED (same reason the
# reset uses a joint mirror), so applying the same wrist rotation to both makes
# left and right turn OPPOSITE ways. The right side uses a REFLECTION matrix
# (det = -1): via the similarity  M @ dR @ M  it reverses the rotation sense
# across one plane, so both arms track the wrist the same way.
#   diag([1,-1,1]) reflects the left-right (Y) plane - the usual bimanual mirror.
# If a specific axis is still reversed after testing, switch the -1 to another
# axis (e.g. [-1,1,1] or [1,1,-1]); the left stays identity.
R_ALIGN = {"left":  np.eye(3),
           "right": np.diag([1.0, -1.0, 1.0])}

# The wrist->EE axis alignment A (per side): the delta wrist rotation is mapped
# into the EE frame as  A @ (W0^T W) @ A^T. With the raw-IMU wrist source the IMU
# frame axes do NOT match the EE frame (wrist ROLL came out as an arm pitch), so
# A must be a real rotation, not just the R_ALIGN reflection. calibrate_wrist.py
# captures clean roll/pitch and writes this file; if absent we fall back to
# R_ALIGN (old behavior).
ALIGN_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "wrist_align.json")

def load_align(side):
    try:
        d = json.load(open(ALIGN_FILE))
        return np.asarray(d[side], dtype=np.float64).reshape(3, 3)
    except Exception:
        return R_ALIGN[side]


def quat_to_mat(q):
    w, x, y, z = np.asarray(q, float) / (np.linalg.norm(q) + 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class ArmIK:
    """Damped least-squares Jacobian IK over a 6-DOF MuJoCo arm model."""
    def __init__(self, xml, site):
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site)
        self.nv = self.model.nv
        self.jrange = self.model.jnt_range[:6].copy()

    def fk(self, q):
        self.data.qpos[:6] = q
        mujoco.mj_forward(self.model, self.data)
        T = np.eye(4)
        T[:3, :3] = self.data.site_xmat[self.sid].reshape(3, 3).copy()
        T[:3, 3] = self.data.site_xpos[self.sid].copy()
        return T

    def ik(self, R_target, q_init, q_rest, iters=100, damping=0.1, tol=1e-3, step=0.5, kpost=0.6):
        """Orientation-only IK (3-DOF task) with NULL-SPACE posture regularization
        toward q_rest. The 6-DOF arm's 3 redundant DOF are used to keep the config
        near the captured ready pose, so the EE reorients while its POSITION barely
        drifts -- fixed-position IK diverges from many configs; orientation-only
        drifts ~20cm; this is the stable middle ground (verified 2026-08-21)."""
        q = np.array(q_init, float)
        jacp = np.zeros((3, self.nv)); jacr = np.zeros((3, self.nv))
        ori_err = np.ones(3)
        for _ in range(iters):
            self.data.qpos[:6] = q
            mujoco.mj_forward(self.model, self.data)
            R_c = self.data.site_xmat[self.sid].reshape(3, 3)
            R_err = R_target @ R_c.T                      # world-frame cur->target
            qe = np.zeros(4); mujoco.mju_mat2Quat(qe, R_err.flatten())
            ang = 2.0 * np.arctan2(np.linalg.norm(qe[1:]), qe[0])
            if ang > np.pi:
                ang -= 2 * np.pi
            nrm = np.linalg.norm(qe[1:])
            ori_err = (ang * qe[1:] / nrm) if nrm > 1e-9 else np.zeros(3)
            if np.linalg.norm(ori_err) < tol:
                break
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.sid)
            Jr = jacr[:, :6]
            Jr_pinv = Jr.T @ np.linalg.solve(Jr @ Jr.T + (damping ** 2) * np.eye(3), np.eye(3))
            dq = Jr_pinv @ ori_err
            null = np.eye(6) - Jr_pinv @ Jr              # null-space projector
            dq = dq + null @ (kpost * (q_rest - q))       # posture toward ready
            q = np.clip(q + step * dq, self.jrange[:, 0], self.jrange[:, 1])
        return np.linalg.norm(ori_err) < tol, q


def ssik_ik(T_link6, q_seed, jrange):
    """Analytical IK via ssik. Returns (True, q) for the in-limit branch nearest
    q_seed (joint-space continuity), or (False, q_seed) if ssik has no solution."""
    if yam_ik is None:
        return False, q_seed
    try:
        sols = yam_ik.solve(np.asarray(T_link6, float), q_seed=q_seed, respect_limits=True)
    except Exception:
        return False, q_seed
    best, bestd = None, None
    for s in sols:
        q = np.asarray(s.q, float)
        if np.any(q < jrange[:, 0] - 1e-6) or np.any(q > jrange[:, 1] + 1e-6):
            continue
        d = np.linalg.norm(q - q_seed)          # nearest branch to previous pose
        if bestd is None or d < bestd:
            best, bestd = q, d
    if best is None:
        return False, q_seed
    return True, best


class WristSource:
    """Latest wrist pose for one side off udp (drain-to-newest). Sources:
    MANUS glove stream (udp/9872, "q" only) or the Vive tracker stream
    (vive_wrist_stream.py, udp 9873/9874, "q" + "p" position in meters)."""
    def __init__(self, side, in_port, glove_id=None):
        self.side, self.glove_id = side, glove_id
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("127.0.0.1", in_port))
        self.rx.setblocking(False)

    def latest_all(self):
        """Newest packet as {side: (quat, pos)}; occluded sides are absent."""
        data = None
        while True:
            try:
                data, _ = self.rx.recvfrom(65536)
            except BlockingIOError:
                break
        if data is None:
            return {}
        try:
            msg = json.loads(data.decode())
        except Exception:
            return {}
        out = {}
        for g in msg.get("gloves", []):
            if "q" not in g:                # tracker occluded this frame
                continue
            q = np.asarray(g["q"], dtype=np.float64)
            p = np.asarray(g["p"], dtype=np.float64) if "p" in g else None
            if self.glove_id is not None and g.get("id") == self.glove_id:
                out[self.side] = (q, p)
            elif g.get("side"):
                out[g["side"]] = (q, p)
        return out

    def latest(self):
        """Newest (quat, pos) for this side; pos is None for sources without
        position (gloves) and for invalid tracker frames (side then holds)."""
        return self.latest_all().get(self.side, (None, None))

    def latest_quat(self):
        return self.latest()[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", required=True, choices=["left", "right"])
    ap.add_argument("--server-host", default="192.168.1.9")
    ap.add_argument("--server-port", type=int, default=None)
    ap.add_argument("--in-port", type=int, default=9872)
    ap.add_argument("--glove-id", type=int, default=None)
    ap.add_argument("--arm", default="yam_ultra")
    ap.add_argument("--site", default="grasp_site")
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--max-step", type=float, default=0.03, help="max rad/joint per update")
    ap.add_argument("--no-ssik", action="store_true",
                    help="force the DLS Jacobian fallback instead of ssik analytical IK")
    ap.add_argument("--pos-scale", type=float, default=0.0,
                    help="wrist->arm position gain (0 = hold position; needs a source "
                         "with 'p', i.e. the Vive tracker stream)")
    ap.add_argument("--max-offset", type=float, default=0.35,
                    help="max EE position offset from the start pose, meters (safety box)")
    ap.add_argument("--align", choices=["file", "identity", "mirror"], default="file",
                    help="wrist->EE axis alignment: 'file' = wrist_align.json (IMU-"
                         "calibrated, old default), 'identity' = axes already match "
                         "(Vive stream, pre-mapped to robot frame), 'mirror' = R_ALIGN")
    ap.add_argument("--frame", choices=["local", "world"], default="local",
                    help="delta composition. 'local' (gloves): wrist delta in the "
                         "neutral-wrist frame, conjugated by --align. 'world' (Vive "
                         "trackers): deltas in the tracking WORLD frame, mapped into "
                         "the arm model frame by M = Rz(-90) @ R(operator yaw) - the "
                         "EE then moves/rotates the SAME direction as the tracker. "
                         "Operator yaw is auto-calibrated at capture from the "
                         "left->right tracker baseline (fallback: operator_yaw_deg "
                         "in vive_trackers.json).")
    ap.add_argument("--wrist-side", choices=["left", "right"], default=None,
                    help="which TRACKER side feeds this arm (default: same as --side)")
    ap.add_argument("--base-yaw-deg", type=float, default=0.0,
                    help="extra yaw (deg) of THIS arm's base vs the rig frame. "
                         "The two arms are mounted 180 deg apart (verified live "
                         "2026-08-25: same M was correct on 11334, opposite on "
                         "11333), so one side gets 0 and the other 180.")
    ap.add_argument("--extend-start", type=float, default=0.0,
                    help="before capturing the neutral, smoothly extend the EE "
                         "this many meters along the arm axis (+x). The folded "
                         "home pose has NO retract room (model -x is blocked by "
                         "the shoulder joint range), so teleop starting at home "
                         "cannot pull the wrist back at all; starting "
                         "mid-workspace gives slack in every direction.")
    ap.add_argument("--lift-start", type=float, default=0.0,
                    help="also lift the ready pose this many meters (+z); the "
                         "home pose is low, leaving almost no downward room. "
                         "0.15/0.15 extend/lift probes 0.2-0.35 m of slack in "
                         "every direction.")
    ap.add_argument("--flip-x", action="store_true",
                    help="negate the arm-axis (model x) POSITION delta only. "
                         "2026-08-26: both arms' 'inward' (along the arm's own "
                         "pointing direction) mapped to model -x, which is "
                         "blocked by the shoulder joint range - the arm froze. "
                         "Inward must extend (+x). Rotation mapping untouched.")
    ap.add_argument("--flip-z", action="store_true",
                    help="MIRROR this arm's mapping across the horizontal plane "
                         "(applied to position AND rotation, consistently). The "
                         "right arm's effective mapping is mirrored (2026-08-26: "
                         "with base yaw 180 its horizontals were correct but "
                         "up/down opposite - impossible for a proper rotation). "
                         "Flipping only the position z left rotation targets "
                         "inconsistent and the IK jumped branches erratically.")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--reset", action="store_true",
                    help="smoothly move the arm to the home pose (all joints 0), stiff, then exit")
    ap.add_argument("--reset-time", type=float, default=3.0, help="seconds for the reset move")
    ap.add_argument("--loose", action="store_true",
                    help="drop the follower to gravity-comp (loose/backdrivable), then exit")
    ap.add_argument("--mirror-home", action="store_true",
                    help="capture the LEFT arm's current pose and save right = mirror(left) "
                         "as the per-side reset homes, then exit")
    args = ap.parse_args()
    port = args.server_port or SIDE_PORT[args.side]

    # ---- MIRROR-HOME: read LEFT arm, write left + mirrored-right reset homes ----
    if args.mirror_home:
        lc = portal.Client(f"{args.server_host}:{SIDE_PORT['left']}")
        ql = np.asarray(lc.get_joint_pos().result(), dtype=np.float64)[:6]
        qr = MIRROR_SIGN * ql
        try:
            d = json.load(open(HOME_FILE))
        except Exception:
            d = {}
        d["left"], d["right"] = [float(x) for x in ql], [float(x) for x in qr]
        json.dump(d, open(HOME_FILE, "w"), indent=2)
        print(f"mirror-home saved: left={np.round(np.degrees(ql),1)} "
              f"-> right={np.round(np.degrees(qr),1)} (deg)", file=sys.stderr)
        return 0

    print(f"[{args.side}] connecting to follower {args.server_host}:{port} ...", file=sys.stderr)
    client = portal.Client(f"{args.server_host}:{port}")
    q0 = np.asarray(client.get_joint_pos().result(), dtype=np.float64)[:6]

    # ---- LOOSE mode: gravity-comp only, arm goes backdrivable (Shut down) ----
    if args.loose:
        if not args.dry_run:
            send_loose(client, q0)
        print(f"[{args.side}] arm LOOSE (gravity-comp) - move it by hand", file=sys.stderr)
        return 0

    # ---- RESET mode: smoothly drive the arm to its per-side HOME pose ----
    # (left default 0; right = mirror(left) if set via --mirror-home, so the two
    #  hands share an orientation despite the mirrored mount)
    if args.reset:
        home = load_home(args.side)
        steps = max(2, int(args.reset_time * args.rate))
        print(f"[{args.side}] reset: moving to home {np.round(np.degrees(home),1)}deg "
              f"over {args.reset_time}s ...", file=sys.stderr)
        for i in range(1, steps + 1):
            q = q0 + (home - q0) * (i / steps)              # stiff interpolation
            if not args.dry_run:
                client.command_joint_pos(q)
            time.sleep(1.0 / args.rate)
        print(f"[{args.side}] reset done - arm at home (stiff)", file=sys.stderr)
        return 0

    ik = ArmIK(combine_arm_and_gripper_xml(getattr(ArmType, args.arm.upper()),
                                           GripperType.NO_GRIPPER), args.site)
    T0 = ik.fk(q0); R0 = T0[:3, :3].copy(); p0 = T0[:3, 3].copy()
    # Constant grasp_site -> link_6 offset so ssik (which solves in link_6) can be
    # driven from a target built in the grasp_site frame (where R_ALIGN is tuned).
    use_ssik = (yam_ik is not None) and not args.no_ssik
    T_g2l = np.linalg.inv(T0) @ np.asarray(yam_ik.fk(q0), float) if use_ssik else None
    print(f"[{args.side}] IK: {'ssik analytical' if use_ssik else 'DLS Jacobian'}", file=sys.stderr)
    # STIFFEN at the captured (home) pose with the follower's own gains.
    if not args.dry_run:
        client.command_joint_pos(q0)
    signal.signal(signal.SIGTERM, _clean_exit)   # Stop just exits; arm holds stiff
    signal.signal(signal.SIGINT, _clean_exit)

    # ---- extend to a mid-workspace ready pose (see --extend-start) ----
    if (args.extend_start > 0.0 or args.lift_start > 0.0) and use_ssik:
        Tg = np.eye(4); Tg[:3, :3] = R0
        Tg[:3, 3] = p0 + np.array([args.extend_start, 0.0, args.lift_start])
        ok_ext, q_ext = ssik_ik(Tg @ T_g2l, q0, ik.jrange)
        if ok_ext:
            steps = max(2, int(2.0 * args.rate))     # ~2 s smooth extend
            for i in range(1, steps + 1):
                q = q0 + (q_ext - q0) * (i / steps)
                if not args.dry_run:
                    client.command_joint_pos(q)
                time.sleep(1.0 / args.rate)
            q0 = q_ext
            T0 = ik.fk(q0); R0 = T0[:3, :3].copy(); p0 = T0[:3, 3].copy()
            print(f"[{args.side}] extended {args.extend_start:.2f} m / lifted "
                  f"{args.lift_start:.2f} m to ready pose q0={np.round(q0,3)}", file=sys.stderr)
        else:
            print(f"[{args.side}] extend-start IK failed - starting from current pose",
                  file=sys.stderr)
    print(f"[{args.side}] captured ready q0={np.round(q0,3)} (stiff)", file=sys.stderr)

    wrist_side = args.wrist_side or args.side
    src = WristSource(wrist_side, args.in_port, args.glove_id)
    Wq0, Wp0, t_end = None, None, time.time() + 30   # ride out tracking flaps
    other = {"left": "right", "right": "left"}[wrist_side]
    Wp0_other = None                       # other wrist's neutral (yaw baseline)
    t_other = None                         # yaw baseline gets a short grace only
    while time.time() < t_end:
        both = src.latest_all()
        if Wq0 is None and wrist_side in both:
            Wq0, Wp0 = both[wrist_side]
            t_other = time.time() + 15     # wait a bit longer for the yaw baseline
        if Wp0_other is None and other in both and both[other][1] is not None:
            Wp0_other = both[other][1]
        if Wq0 is not None and (args.frame != "world" or Wp0_other is not None
                                or time.time() > t_other):
            break
        time.sleep(0.02)
    if Wq0 is None:
        print(f"[{args.side}] no {wrist_side} wrist on udp/{args.in_port} - streamer up? tracker seen?", file=sys.stderr)
        return 2
    R_neutral = quat_to_mat(Wq0)
    R_neutral_inv = R_neutral.T
    track_pos = args.pos_scale != 0.0 and Wp0 is not None
    if args.pos_scale != 0.0 and Wp0 is None:
        print(f"[{args.side}] --pos-scale set but source has no position - holding position", file=sys.stderr)
    print(f"[{args.side}] neutral wrist captured (tracker={wrist_side}, "
          f"pos={'yes' if track_pos else 'no'}); teleop live (dry-run={args.dry_run})",
          file=sys.stderr)

    def Rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    M = None
    M_rot = None    # separate MEASURED rotation map ("<side>_rot", 2026-08-27:
    #                 calibrate_vive_gui.py wrist-rotation mode) - when absent,
    #                 rotations conjugate through the position map M as before
    if args.frame == "world":
        # MEASURED map (calibrate_arm_axes.py: strap the tracker to the wrist,
        # jog the arm along its model axes, record what the tracker does) beats
        # every heuristic below - it captures yaw, mount AND mirror exactly.
        try:
            _mm = json.load(open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "vive_arm_map.json")))
            if args.side in _mm:
                M = np.asarray(_mm[args.side], dtype=np.float64).reshape(3, 3)
                print(f"[{args.side}] using MEASURED arm-axis map "
                      f"(det={np.linalg.det(M):+.2f})\n{np.round(M,3)}", file=sys.stderr)
            if args.side + "_rot" in _mm:
                M_rot = np.asarray(_mm[args.side + "_rot"], dtype=np.float64).reshape(3, 3)
                print(f"[{args.side}] using MEASURED wrist-ROTATION map "
                      f"(det={np.linalg.det(M_rot):+.2f})\n{np.round(M_rot,3)}", file=sys.stderr)
        except Exception:
            pass
    if M is None and args.frame == "world":
        # operator_yaw_deg in vive_trackers.json PINS the yaw (0 verified
        # correct on this rig 2026-08-25, with the per-side base yaws).
        # Delete the key to re-enable auto-calibration from the tracker
        # baseline (operator stands BEHIND the arms; LEFT->RIGHT == +x_rig).
        YAW_SHARE = "/tmp/zhi_operator_yaw.json"   # both arms MUST share one yaw
        yaw_cfg = None
        try:
            _d = json.load(open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "vive_trackers.json")))
            if "operator_yaw_deg" in _d:
                yaw_cfg = float(_d["operator_yaw_deg"])
        except Exception:
            pass
        pL = Wp0 if wrist_side == "left" else Wp0_other
        pR = Wp0 if wrist_side == "right" else Wp0_other
        if yaw_cfg is not None:
            R_rig_stream = Rz(np.radians(yaw_cfg))
            print(f"[{args.side}] using pinned operator yaw {yaw_cfg:.1f} deg", file=sys.stderr)
        elif pL is not None and pR is not None:
            b = np.array([pR[0] - pL[0], pR[1] - pL[1], 0.0])
            if np.linalg.norm(b) > 0.05:
                xr = b / np.linalg.norm(b)
                R_rig_stream = np.array([xr, np.cross([0, 0, 1.0], xr), [0, 0, 1.0]])
                yaw_rz = float(np.degrees(np.arctan2(-xr[1], xr[0])))
                print(f"[{args.side}] operator yaw auto-calibrated: baseline "
                      f"{np.round(b,3)}, Rz yaw {yaw_rz:.1f} deg", file=sys.stderr)
                try:   # publish so the OTHER arm uses the SAME frame if its
                    #    own baseline is missing (mismatch = weird motion)
                    json.dump({"yaw_rz_deg": yaw_rz, "t": time.time()},
                              open(YAW_SHARE, "w"))
                except Exception:
                    pass
            else:
                print(f"[{args.side}] trackers too close for yaw baseline", file=sys.stderr)
                R_rig_stream = None
        else:
            print(f"[{args.side}] other tracker not seen for yaw baseline", file=sys.stderr)
            R_rig_stream = None
        if R_rig_stream is None:
            # prefer the yaw the other arm calibrated THIS run (shared frame),
            # then the config pin, then 0.
            yaw = None
            try:
                d = json.load(open(YAW_SHARE))
                if time.time() - d.get("t", 0) < 120:
                    yaw = float(d["yaw_rz_deg"])
                    print(f"[{args.side}] using other arm's calibrated yaw {yaw:.1f} deg",
                          file=sys.stderr)
            except Exception:
                pass
            if yaw is None:
                try:
                    yaw = float(json.load(open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "vive_trackers.json")))
                                .get("operator_yaw_deg", 0.0))
                except Exception:
                    yaw = 0.0
                print(f"[{args.side}] using config yaw {yaw:.1f} deg", file=sys.stderr)
            R_rig_stream = Rz(np.radians(yaw))
        # model frame = rig frame yawed -90 (model +x == rig +y), plus this
        # arm's own mount yaw (the two arms are mounted 180 deg apart)
        M = Rz(np.radians(args.base_yaw_deg - 90.0)) @ R_rig_stream
        if args.flip_z:
            # mirrored mount: reflect across the horizontal plane. M becomes
            # improper (det -1); conjugating dR with it stays a proper rotation
            # but correctly REVERSES pitch/roll sense while keeping yaw.
            M = np.diag([1.0, 1.0, -1.0]) @ M
        print(f"[{args.side}] world->model map M (flip_z={args.flip_z})=\n"
              f"{np.round(M,3)}", file=sys.stderr)

    if args.align == "identity":
        A = np.eye(3)
    elif args.align == "mirror":
        A = R_ALIGN[args.side]
    else:
        A = load_align(args.side)                        # wrist->EE axis alignment
    if args.frame == "local":
        print(f"[{args.side}] wrist align ({args.align}) A=\n{np.round(A,3)}", file=sys.stderr)
    q_prev = q0.copy()
    period, n = 1.0 / args.rate, 0
    while True:
        wq, wp = src.latest()
        if wq is not None:
            if args.frame == "world":
                # world-frame delta: EE rotates about the SAME world axes as the
                # tracker (mount orientation on the wrist cancels out)
                _Mr = M_rot if M_rot is not None else M
                dR = _Mr @ (quat_to_mat(wq) @ R_neutral_inv) @ _Mr.T
                R_target = dR @ R0
            else:
                dR = A @ (R_neutral_inv @ quat_to_mat(wq)) @ A.T   # wrist-local delta
                R_target = R0 @ dR                           # grasp_site orientation target
            p_target = p0
            if track_pos and wp is not None:
                dp = args.pos_scale * (wp - Wp0)             # wrist translation since neutral
                if args.frame == "world":
                    dp = M @ dp                              # same direction as the tracker
                    if args.flip_x:
                        dp[0] = -dp[0]                       # inward must extend (+x)
                dnorm = np.linalg.norm(dp)
                if dnorm > args.max_offset:                  # safety box around start pose
                    dp *= args.max_offset / dnorm
                p_target = p0 + dp
            ok = False
            if use_ssik:
                Tg = np.eye(4); Tg[:3, :3] = R_target; Tg[:3, 3] = p_target
                ok, q = ssik_ik(Tg @ T_g2l, q_prev, ik.jrange)
            if not ok:                                       # DLS fallback (orientation-only)
                ok, q = ik.ik(R_target, q_prev, q0)
            raw = q - q_prev
            if np.max(np.abs(raw)) > 1.0:              # unreachable/singular -> hold
                if n % 60 == 0:
                    print(f"[{args.side}] IK jump {np.round(raw,2)} - holding", file=sys.stderr)
            else:
                q = np.clip(q_prev + np.clip(raw, -args.max_step, args.max_step),
                            ik.jrange[:, 0], ik.jrange[:, 1])
                if not args.dry_run:
                    client.command_joint_pos(q)     # stiff hold at q (follower gains)
                q_prev = q
                if n % 15 == 0:
                    print(f"[{args.side}] {'ok' if ok else '~'} q={np.round(np.degrees(q),1)}", flush=True)
            n += 1
        time.sleep(period)


if __name__ == "__main__":
    sys.exit(main())
