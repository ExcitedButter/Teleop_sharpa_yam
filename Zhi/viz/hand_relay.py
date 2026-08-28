#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Relay the Sharpa retargeting optimizer's HandAction (ZMQ :6668, protobuf)
to a plain-JSON UDP stream for viz_panel.py.

Runs in the `sharpamanus` conda env (the only env with pyzmq + protobuf);
viz_panel.py (i2rt venv) then needs no extra packages. ZMQ SUB is
multi-subscriber, so this taps the same stream sharpa_zmq_to_driver.py uses
without disturbing it.

    /home/yam/miniconda3/envs/sharpamanus/bin/python hand_relay.py
        [--in-addr tcp://127.0.0.1:6668] [--out-port 59260] [--rate 30]
"""
import argparse, json, socket, sys, time, os

sys.path.insert(0, "/home/yam/manus/sharpa-manus-sdk/retargeting_alg_release_V4.0/include/proto_hand")
# local clone fallback (2026-08-27): hermes' copy is gone on jdw-Lambda-Vector
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    "sharpa-manus-sdk", "retargeting_alg_release_V4.0", "include", "proto_hand"))
import zmq
try:
    import sharpa_hand_pb2
except ImportError:
    # the sharpa-manus SDK (hermes stack) isn't deployed on this machine yet -
    # idle instead of exiting so the viz watchdog doesn't respawn us every 3 s
    print("sharpa_hand_pb2 unavailable (sharpa-manus SDK not installed) - "
          "hand relay idle", file=sys.stderr, flush=True)
    while True:
        time.sleep(3600)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-addr", default="tcp://127.0.0.1:6668")
    ap.add_argument("--out-port", type=int, default=59260)
    ap.add_argument("--rate", type=float, default=30.0)
    args = ap.parse_args()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 1000)
    sub.connect(args.in_addr)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = ("127.0.0.1", args.out_port)
    print(f"relaying {args.in_addr} -> udp/{args.out_port}", file=sys.stderr, flush=True)

    period = 1.0 / args.rate
    last = 0.0
    while True:
        try:
            data = sub.recv()
        except zmq.error.Again:
            continue
        now = time.time()
        if now - last < period:
            continue  # rate-limit; always keeps the newest message
        msg = sharpa_hand_pb2.HandAction()
        try:
            msg.ParseFromString(data)
        except Exception:
            continue
        left = list(msg.joint_left.position)
        right = list(msg.joint_right.position)
        if len(left) == 22 or len(right) == 22:
            out.sendto(json.dumps({"t": now, "left": left, "right": right}).encode(), dst)
            last = now


if __name__ == "__main__":
    sys.exit(main())
