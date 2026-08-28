#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Live Sharpa hand JOINT STATE -> UDP JSON for viz_panel.py.

Reads the hands' TRUE joint angles (get_joint_position_degree, works since
SDK 5.0.7 - the old firmware "Invalid Pre-Header" gate is gone) and publishes
{"t": ..., "left": [22 rad], "right": [22 rad]} on udp/59261 at ~15 Hz.
Sides come from the hardware's own DeviceInfo.hand_side, so no relabeling.

Read-only: coexists with the sharpa_hand_driver processes that command the
hands (verified live 2026-08-25). Runs in the sharpa-venv with
cwd=/opt/sharpa-wave-sdk (the SDK loads config.yaml from cwd).
"""
import json, math, os, socket, sys, time

sys.path.insert(0, "/opt/sharpa-wave-sdk/python")
os.chdir("/opt/sharpa-wave-sdk")
import sharpa
from sharpa import SharpaWaveManager, HandSide, DeviceType

OUT_PORT = 59261
RATE_HZ = 15.0


def main():
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = ("127.0.0.1", OUT_PORT)
    mgr = SharpaWaveManager.get_instance()
    time.sleep(2.0)
    hands = {}          # "left"/"right" -> SharpaWave
    period = 1.0 / RATE_HZ
    while True:
        if not hands:
            for d in mgr.get_all_devices():
                if d.device_type != DeviceType.HAND:
                    continue
                side = "left" if d.hand_side == HandSide.LEFT else "right"
                h = mgr.connect(d.sn)
                if h is not None:
                    hands[side] = h
            if not hands:
                time.sleep(3.0)
                continue
            print("reading state:", list(hands), file=sys.stderr, flush=True)
        msg, dead = {"t": time.time()}, []
        for side, h in hands.items():
            try:
                err, ang = h.get_joint_position_degree()
                if err.code == 0 and len(ang) == 22:
                    msg[side] = [math.radians(a) for a in ang]
                else:
                    dead.append(side)
            except Exception:
                dead.append(side)
        for side in dead:   # reconnect lazily on next loop
            hands.pop(side, None)
        if len(msg) > 1:
            out.sendto(json.dumps(msg).encode(), dst)
        time.sleep(period)


if __name__ == "__main__":
    sys.exit(main())
