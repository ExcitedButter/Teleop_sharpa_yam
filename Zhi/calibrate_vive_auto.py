#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Auto-segmenting hands-only Vive axis calibration - no keyboard, no arm motion.

Same math as calibrate_vive_by_hand.py, but instead of ENTER prompts it
watches the tracker and segments the strokes itself:

    hold STILL (>=2 s)  ->  stroke 1 (+x, arm forward)  ->  STILL 2 s
                        ->  stroke 2 (+y, arm left)     ->  STILL 2 s
                        ->  stroke 3 (+z, straight up)  ->  STILL 2 s

"STILL" = tracker position spread < 1 cm for 2 s. Each stroke must be 15+ cm.
Progress and the result go to stdout (run detached, watch the log).
Writes params/vive_arm_map.json[side] like the other calibrators.
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
STILL_S = 2.0          # how long the tracker must hold to count as still
STILL_SPREAD = 0.010   # m: max position spread during a still phase
MIN_STROKE = 0.15      # m: minimum stroke length
AXIS_NAMES = ["+x arm-FORWARD", "+y arm-LEFT", "+z UP"]


def log(msg):
    print("%s  %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def stream_positions(side, tracker, timeout):
    """Yield (t, pos) for the chosen tracker until timeout."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", STREAM_PORT[side]))
    rx.settimeout(0.3)
    t_end = time.time() + timeout
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
                yield time.time(), np.asarray(g["p"], float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", required=True, choices=("left", "right"))
    ap.add_argument("--tracker", choices=("left", "right"), default=None)
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    tracker = args.tracker or args.side

    log("calibrating the %s arm's map with the '%s' tracker (auto-segmenting)"
        % (args.side, tracker))
    log("rhythm: STILL 2s -> stroke -> STILL 2s -> stroke -> STILL 2s -> "
        "stroke -> STILL 2s   (strokes: %s)" % ", ".join(AXIS_NAMES))

    centroids = []          # still-phase positions
    window = []             # (t, pos) sliding window for stillness detection
    state = "waiting-still"
    last_note = 0.0
    for t, p in stream_positions(args.side, tracker, args.timeout):
        window.append((t, p))
        window = [(tt, pp) for tt, pp in window if t - tt <= STILL_S]
        if len(window) < 10 or window[-1][0] - window[0][0] < STILL_S * 0.9:
            continue
        pts = np.asarray([pp for _, pp in window])
        spread = float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
        c = pts.mean(axis=0)
        if state == "waiting-still":
            if spread < STILL_SPREAD:
                if centroids and np.linalg.norm(c - centroids[-1]) < MIN_STROKE:
                    # still again but hasn't moved far enough yet - keep waiting
                    if t - last_note > 5:
                        log("still detected but stroke %d is only %.0f cm - "
                            "move 15+ cm along %s"
                            % (len(centroids),
                               np.linalg.norm(c - centroids[-1]) * 100,
                               AXIS_NAMES[len(centroids) - 1]))
                        last_note = t
                    continue
                centroids.append(c.copy())
                n = len(centroids)
                if n == 1:
                    log("start point captured - now stroke 1: move 30 cm along "
                        "%s, then hold still" % AXIS_NAMES[0])
                elif n < 4:
                    u = centroids[-1] - centroids[-2]
                    log("stroke %d captured: %s (|%.2f| m) - now stroke %d: %s"
                        % (n - 1, np.round(u, 3), np.linalg.norm(u),
                           n, AXIS_NAMES[n - 1]))
                else:
                    u = centroids[-1] - centroids[-2]
                    log("stroke 3 captured: %s (|%.2f| m)"
                        % (np.round(u, 3), np.linalg.norm(u)))
                    break
                state = "waiting-move"
        else:  # waiting-move
            if spread >= STILL_SPREAD or np.linalg.norm(c - centroids[-1]) > 0.04:
                state = "waiting-still"
    else:
        log("TIMEOUT or tracker lost after %d still points - re-run" % len(centroids))
        return 1

    strokes = [centroids[i + 1] - centroids[i] for i in range(3)]
    cols = [u / np.linalg.norm(u) for u in strokes]
    U = np.array(cols).T
    W, S, Vt = np.linalg.svd(U)
    R = W @ Vt
    M = R.T
    det = float(np.linalg.det(R))
    orth = float(np.min(S) / np.max(S))
    log("measured map M (stream->model), det=%+.2f%s, axis quality %.2f:"
        % (det, " [MIRRORED]" if det < 0 else "", orth))
    for row in np.round(M, 3):
        log("   %s" % row)
    if orth < 0.7:
        log("WARNING: strokes far from perpendicular (quality %.2f) - re-run "
            "with straighter strokes" % orth)
    try:
        m = json.load(open(MAP_FILE))
    except Exception:
        m = {}
    m[args.side] = [[float(v) for v in row] for row in M]
    json.dump(m, open(MAP_FILE, "w"), indent=1)
    log("SAVED -> %s ['%s'] - teleop uses it automatically" % (MAP_FILE, args.side))
    return 0


if __name__ == "__main__":
    sys.exit(main())
