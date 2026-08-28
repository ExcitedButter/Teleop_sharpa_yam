#!/usr/bin/env python3
"""Calibrate the tracker->rig yaw by MEASURING the rig with a tracker.

The SteamVR world frame is anchored to the base station, so moving the
station rotates every tracker coordinate. This script pins the mapping to the
physical rig instead of guessing from the operator's stance:

    1. hold ONE tracker against the LEFT yam arm's BASE   (your left, standing
       behind the rig facing the same way the arms reach), press ENTER
    2. hold the SAME tracker against the RIGHT arm's base, press ENTER

The left-base -> right-base vector is the rig's "right" axis in stream
coordinates; from it the exact operator_yaw_deg is computed and PINNED in
vive_trackers.json (a pin disables the auto-yaw). Teleop directions then match
the room: tracker forward == wrist forward, with the hands behind the yams.

Re-run this any time the base station is moved. Needs the vive stream running
(it reads udp/9873). Uses whichever tracker is valid at each ENTER - use the
same physical tracker for both points.

    python3 /home/yam/manus/calibrate_vive_yaw.py
"""
import os
import json, math, socket, sys, time

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)), "params", "vive_trackers.json")
PORT = 9873


def snap(rx, want_secs=1.0):
    """Average valid tracker positions over ~1 s; returns (side, [x,y,z])."""
    acc, side_used = {}, None
    t_end = time.time() + want_secs
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
            if g.get("valid") and "p" in g:
                acc.setdefault(g["side"], []).append(g["p"])
    if not acc:
        return None, None
    side_used = max(acc, key=lambda s: len(acc[s]))
    pts = acc[side_used]
    p = [sum(v[i] for v in pts) / len(pts) for i in range(3)]
    return side_used, p


def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", PORT))
    rx.settimeout(0.5)

    print("== Vive->rig yaw calibration ==")
    print("Stand behind the rig, facing the same way the arms reach.\n")

    input("1) Hold a tracker against the LEFT arm's BASE, keep it still, "
          "then press ENTER... ")
    side1, p_left = snap(rx)
    if p_left is None:
        print("no valid tracker seen - is the vive stream running and the "
              "tracker green in SteamVR?"); return 1
    print("   left base  @ %s  (tracker: %s)" % ([round(v, 3) for v in p_left], side1))

    input("2) Now hold the SAME tracker against the RIGHT arm's BASE, "
          "keep it still, then press ENTER... ")
    side2, p_right = snap(rx)
    if p_right is None:
        print("no valid tracker seen"); return 1
    print("   right base @ %s  (tracker: %s)" % ([round(v, 3) for v in p_right], side2))

    ux, uy = p_right[0] - p_left[0], p_right[1] - p_left[1]
    dist = math.hypot(ux, uy)
    if dist < 0.3:
        print("bases only %.2f m apart horizontally - expected ~0.76 m. "
              "Redo the measurement." % dist); return 1
    # rig 'right' axis u must map to -x after Rz(yaw) (see manus_arm_teleop
    # frame derivation): yaw = 180 - atan2(uy, ux)
    yaw = 180.0 - math.degrees(math.atan2(uy, ux))
    yaw = (yaw + 180.0) % 360.0 - 180.0     # wrap to [-180, 180)

    d = json.load(open(CONFIG))
    d["operator_yaw_deg"] = round(yaw, 2)
    json.dump(d, open(CONFIG, "w"), indent=2)
    print("\nbase separation %.2f m, pinned operator_yaw_deg = %.1f in %s"
          % (dist, yaw, CONFIG))
    print("done - press Start Teleop (no restarts needed).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
