#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Hands-only Vive->arm axis + DISTANCE calibration - the ARM NEVER MOVES.

The companion tool (calibrate_arm_axes.py) jogs the arm; this one is the
inverse: YOU move the tracker by hand along each of the arm's three model
axes, it records the tracker displacement vectors and builds the same
stream->model map M (SVD-orthonormalized, mirror preserved) that
manus_arm_teleop --frame world uses. Because M is orthonormal and the
panel's pos-scale is 1.0, a tracker move of d meters commands an EE move of
exactly d meters in the mapped direction - 1:1 distance by construction
(up to the teleop's --max-offset safety radius around the start pose).

Per side, with vive_wrist_stream running and the tracker in hand:

  ./calibrate_vive_by_hand.py --side left            # calibrate left arm's map
  ./calibrate_vive_by_hand.py --side right
  ./calibrate_vive_by_hand.py --side left --verify   # live 1:1 distance check

For each of the 3 prompts: hold the tracker STILL, press ENTER, move it
30+ cm STRAIGHT along the asked direction, hold STILL, press ENTER.
Directions are the ARM's axes as they sit in the room:
  +x  the arm's own FORWARD (from that arm's base toward its workspace;
      the two arms face opposite ways, so left and right differ by 180 deg)
  +y  the arm's LEFT (90 deg counterclockwise from its forward, horizontal)
  +z  straight UP
Straight, single-direction strokes matter more than exact distance.
"""
import argparse
import json
import os
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
MAP_FILE = os.path.join(HERE, "params", "vive_arm_map.json")
STREAM_PORT = {"left": 9873, "right": 9874}
AXES = [
    ("+x  ARM FORWARD  (from this arm's base toward its workspace)", 0),
    ("+y  ARM LEFT     (90 deg CCW from its forward, horizontal)", 1),
    ("+z  STRAIGHT UP", 2),
]


def open_stream(side):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", STREAM_PORT[side]))
    rx.settimeout(0.3)
    return rx


def snap(rx, tracker, secs=1.0):
    """Average the chosen tracker's position over `secs`; None if not seen."""
    pts = []
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
            if g.get("side") == tracker and g.get("valid") and "p" in g:
                pts.append(g["p"])
    return np.mean(np.asarray(pts), axis=0) if pts else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", required=True, choices=("left", "right"),
                    help="which ARM's map to calibrate")
    ap.add_argument("--tracker", choices=("left", "right"), default=None,
                    help="which tracker you are holding (default: same side)")
    ap.add_argument("--verify", action="store_true",
                    help="no calibration - live-print tracker delta vs the "
                         "mapped model delta to check 1:1 distance")
    args = ap.parse_args()
    tracker = args.tracker or args.side
    rx = open_stream(args.side)

    if snap(rx, tracker, 1.2) is None:
        print("no valid '%s' tracker on udp/%d - stream running? tracker green "
              "in SteamVR?" % (tracker, STREAM_PORT[args.side]))
        return 1

    # ---------------- verify mode: live 1:1 readout ----------------
    if args.verify:
        M = np.asarray(json.load(open(MAP_FILE))[args.side], float)
        input("VERIFY: hold the tracker at your reference point, then ENTER "
              "to capture the neutral ... ")
        p0 = snap(rx, tracker, 1.0)
        print("neutral captured. Move the tracker; Ctrl-C to stop.")
        print("%28s | %28s | %s" % ("tracker delta (m)", "-> model delta (m)", "length"))
        try:
            while True:
                p = snap(rx, tracker, 0.4)
                if p is None:
                    print("(tracker occluded)")
                    continue
                d = p - p0
                md = M @ d
                print("%28s | %28s | %.3f m" %
                      (np.round(d, 3), np.round(md, 3), float(np.linalg.norm(md))))
        except KeyboardInterrupt:
            return 0

    # ---------------- calibration ----------------
    print("\nCalibrating the %s arm's map with the '%s' tracker in hand."
          % (args.side, tracker))
    print("The arm does NOT move. 3 strokes of 30+ cm, straight and steady.\n")
    cols, strokes = [], []
    for label, _ in AXES:
        input("hold the tracker STILL at the start point, then ENTER ... ")
        p_a = snap(rx, tracker, 1.0)
        if p_a is None:
            print("tracker lost - re-run"); return 1
        input("now MOVE it 30+ cm along %s\n  ... hold STILL at the end, then ENTER ... "
              % label)
        p_b = snap(rx, tracker, 1.0)
        if p_b is None:
            print("tracker lost - re-run"); return 1
        u = np.asarray(p_b) - np.asarray(p_a)
        n = float(np.linalg.norm(u))
        print("  stroke: %s  (|%.3f| m)\n" % (np.round(u, 3), n))
        if n < 0.15:
            print("  too short (<15 cm) - longer strokes make a better map; re-run")
            return 1
        cols.append(u / n)
        strokes.append(n)

    U = np.array(cols).T              # columns = stream direction of each model axis
    W, S, Vt = np.linalg.svd(U)
    R = W @ Vt                        # nearest orthogonal, mirror sign preserved
    M = R.T                           # stream -> model
    det = float(np.linalg.det(R))
    orth = float(np.min(S) / np.max(S))
    print("measured map M (stream->model), det=%+.2f%s, axis quality %.2f:"
          % (det, "  [MIRRORED]" if det < 0 else "", orth))
    print(np.round(M, 3))
    if orth < 0.7:
        print("WARNING: strokes were far from perpendicular - consider re-running")
    # distance fidelity: M is orthonormal, so |M d| == |d| exactly; report it
    for (label, _), u, n in zip(AXES, cols, strokes):
        print("  %s: %.0f cm stroke -> EE will move %.0f cm (1:1)"
              % (label.split()[0], n * 100, float(np.linalg.norm(M @ (u * n))) * 100))

    try:
        m = json.load(open(MAP_FILE))
    except Exception:
        m = {}
    m[args.side] = [[float(v) for v in row] for row in M]
    json.dump(m, open(MAP_FILE, "w"), indent=1)
    print("saved -> %s ['%s']  (teleop uses it automatically; run --verify "
          "to sanity-check, then repeat for the other side)" % (MAP_FILE, args.side))
    return 0


if __name__ == "__main__":
    sys.exit(main())
