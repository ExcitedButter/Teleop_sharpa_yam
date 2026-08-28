#!/usr/bin/env python3
"""Vive tracker wrist -> UDP wrist stream (position + orientation).

Reads the two Vive trackers from SteamVR (pyopenvr) and broadcasts each side's
wrist POSE at ~60 Hz as JSON over UDP, one packet per frame per port:

    {"gloves": [{"side": "left",  "q": [w,x,y,z], "p": [x,y,z], "valid": true},
                {"side": "right", ...}]}

The "gloves" key + per-side "q" deliberately match the MANUS wrist stream that
manus_arm_teleop.py already parses (udp/9872); this stream adds "p" (meters)
so the arm can track position too. Each side's teleop process gets its OWN
port (left 9873, right 9874) - two UDP sockets cannot share one port.

Frames: SteamVR world is x-right / y-up / z-toward-viewer. Poses are remapped
here into the robot convention (x-forward / y-left / z-up) by the matrix C in
the config file, so downstream code never sees SteamVR axes:
        p_robot = C p_vr          R_robot = C R_vr C^T
Tune C (or swap the serials) in /home/yam/manus/vive_trackers.json.

A tracker whose pose is invalid this frame (occluded, station lost) is sent
with "valid": false and NO q/p - the teleop side then simply holds pose.

Run with the SYSTEM python3 (openvr lives there, SteamVR must be running):
    python3 /home/yam/manus/vive_wrist_stream.py
"""
import json, os, socket, sys, time

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "vive_trackers.json")
DEFAULT_CONFIG = {
    # which physical tracker is strapped to which wrist - swap if wrong
    "left":  "LHR-124A9764",
    "right": "LHR-F4DCB137",
    # SteamVR -> robot axis map (rows = robot x,y,z in SteamVR coords):
    # robot x (forward) = -z_vr, robot y (left) = -x_vr, robot z (up) = y_vr
    "C": [[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
    "ports": {"left": 9873, "right": 9874},
    "rate_hz": 60.0,
}


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        cfg.update(json.load(open(CONFIG)))
    except FileNotFoundError:
        json.dump(DEFAULT_CONFIG, open(CONFIG, "w"), indent=2)
        print("wrote default config %s" % CONFIG, file=sys.stderr)
    except Exception as e:
        print("bad %s (%s) - using defaults" % (CONFIG, e), file=sys.stderr)
    return cfg


# ---- tiny 3x3 helpers (system python has no numpy) ----
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_T(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def mat_to_quat(m):
    """3x3 -> [w,x,y,z] (Shepperd's method, numerically safe)."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2
        return [0.25 * s, (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = (1.0 + m[0][0] - m[1][1] - m[2][2]) ** 0.5 * 2
        return [(m[2][1] - m[1][2]) / s, 0.25 * s,
                (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    if m[1][1] > m[2][2]:
        s = (1.0 + m[1][1] - m[0][0] - m[2][2]) ** 0.5 * 2
        return [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s,
                0.25 * s, (m[1][2] + m[2][1]) / s]
    s = (1.0 + m[2][2] - m[0][0] - m[1][1]) ** 0.5 * 2
    return [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s, 0.25 * s]


def main():
    import openvr   # import late so a missing SteamVR gives a clean message

    # SINGLETON guard (added 2026-08-27): two streams with different configs
    # interleave on 9873/9874 and the consumers see the tracker "teleport"
    # between the two physical trackers every packet. Holding this port makes
    # a second copy exit immediately instead.
    _lock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        _lock.bind(("127.0.0.1", 9879))
    except OSError:
        print("another vive_wrist_stream is already running - exiting",
              file=sys.stderr)
        return 1

    cfg = load_config()
    C = cfg["C"]
    Ct = mat_T(C)
    serial_side = {cfg["left"]: "left", cfg["right"]: "right"}
    ports = {k: int(v) for k, v in cfg["ports"].items()}
    period = 1.0 / float(cfg["rate_hz"])

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vr = None
    last_report, seen = 0.0, {}

    while True:
        # ---- (re)connect to SteamVR ----
        if vr is None:
            try:
                vr = openvr.init(openvr.VRApplication_Other)
                print("connected to SteamVR", file=sys.stderr)
            except Exception as e:
                print("SteamVR not reachable (%s) - retrying in 5s" % e,
                      file=sys.stderr)
                time.sleep(5)
                continue

        try:
            poses = vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0,
                openvr.k_unMaxTrackedDeviceCount)
        except Exception as e:
            print("lost SteamVR (%s) - reconnecting" % e, file=sys.stderr)
            try:
                openvr.shutdown()
            except Exception:
                pass
            vr = None
            continue

        gloves = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
                continue
            try:
                serial = vr.getStringTrackedDeviceProperty(
                    i, openvr.Prop_SerialNumber_String)
            except Exception:
                continue
            side = serial_side.get(serial)
            if side is None:
                continue
            pose = poses[i]
            if not (pose.bDeviceIsConnected and pose.bPoseIsValid):
                gloves.append({"side": side, "valid": False})
                seen[side] = "no-pose"
                continue
            m = pose.mDeviceToAbsoluteTracking
            R_vr = [[m[r][c] for c in range(3)] for r in range(3)]
            p_vr = [m[0][3], m[1][3], m[2][3]]
            R = mat_mul(mat_mul(C, R_vr), Ct)
            p = mat_vec(C, p_vr)
            gloves.append({"side": side,
                           "q": [round(v, 6) for v in mat_to_quat(R)],
                           "p": [round(v, 5) for v in p],
                           "valid": True})
            seen[side] = "ok"

        if gloves:
            pkt = json.dumps({"gloves": gloves}).encode()
            for port in set(ports.values()):
                tx.sendto(pkt, ("127.0.0.1", port))

        now = time.time()
        if now - last_report > 5.0:
            last_report = now
            print("streaming: %s -> ports %s" %
                  (seen or "no configured trackers found", sorted(set(ports.values()))),
                  file=sys.stderr, flush=True)

        time.sleep(period)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
