#!/usr/bin/env python3
"""Bridge Sharpa's official retargeting output -> our sharpa_hand_driver.

The sharpa-manus-sdk retargeting demo ALWAYS publishes the optimized 22-joint
targets (radians, Sharpa/URDF order) as a HandAction protobuf on ZMQ (default
tcp://*:6668), regardless of its -wave/-sdk hardware mode. This subscribes to
that and forwards each hand's 22-vector as JSON UDP to the existing
sharpa_hand_driver.py (left :59201, right :59202) - which reliably drives the
hands (and coexists with SharpaPilot) where the demo's own -wave/-sdk gets
blocked by SharpaPilot holding the hand connection.

Run in the sharpamanus env:
  python sharpa_zmq_to_driver.py            # 6668 -> 59201/59202
"""
import argparse, json, socket, sys, os

RETARGET_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "sharpa-manus-sdk", "retargeting_alg_release_V4.0")
sys.path.insert(0, os.path.join(RETARGET_DIR, "include", "proto_hand"))
import zmq
import sharpa_hand_pb2

# Per-joint overrides on top of the official retargeting output. Both empty:
# every joint (fourth finger included) is forwarded exactly as the official
# algorithm produces it.
# INFER maps dst_index -> src_index (copy src onto dst); ZERO forces indices to 0.
INFER = {"left": {}, "right": {}}
ZERO  = {"left": set(), "right": set()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-addr", default="tcp://127.0.0.1:6668", help="demo HandAction ZMQ")
    ap.add_argument("--left", default="127.0.0.1:59201")
    ap.add_argument("--right", default="127.0.0.1:59202")
    ap.add_argument("--print", dest="do_print", action="store_true")
    args = ap.parse_args()

    lh = (args.left.split(":")[0], int(args.left.split(":")[1]))
    rh = (args.right.split(":")[0], int(args.right.split(":")[1]))
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(args.in_addr)
    print(f"reading Sharpa HandAction on {args.in_addr}; "
          f"forwarding left->{args.left} right->{args.right}", file=sys.stderr)

    n = 0
    while True:
        data = sub.recv()
        msg = sharpa_hand_pb2.HandAction()
        try:
            msg.ParseFromString(data)
        except Exception:
            continue
        left = list(msg.joint_left.position)     # 22 radians, Sharpa order
        right = list(msg.joint_right.position)
        if len(left) == 22:
            for dst, src in INFER["left"].items():
                left[dst] = left[src]
            for i in ZERO["left"]:
                left[i] = 0.0
            out.sendto(json.dumps(left).encode(), lh)
        if len(right) == 22:
            for dst, src in INFER["right"].items():
                right[dst] = right[src]
            for i in ZERO["right"]:
                right[i] = 0.0
            out.sendto(json.dumps(right).encode(), rh)
        if args.do_print and n % 30 == 0 and len(left) == 22:
            import math
            print("L:", " ".join(f"{math.degrees(v):+4.0f}" for v in left), flush=True)
        n += 1


if __name__ == "__main__":
    sys.exit(main())
