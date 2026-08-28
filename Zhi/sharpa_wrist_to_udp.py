#!/usr/bin/env python3
"""Wrist adapter: Sharpa keypoint stream (ZMQ 2044) -> wrist quaternion on udp 9872.

The Sharpa client owns the Manus gloves exclusively (integrated SDK), so our
own manus_skeleton_stream (which fed the arm teleop's wrist on udp 9872) can't
run at the same time. This subscribes to the Sharpa client's MocapKeypoints
(25 hand-keypoint POSITIONS per hand on 2044), computes each wrist's ORIENTATION
from the keypoint geometry (positions-only stream -> the proto orientations are
identity), and republishes it on udp 9872 in the exact format manus_arm_teleop
expects: {"gloves":[{"side","q":[w,x,y,z]}]}. So arm teleop + Sharpa finger
retargeting can run together off the one glove owner.

MANUS 25-node order: 0 wrist, thumb 1-4, index 5-9, middle 10-14, ring 15-19,
pinky 20-24. Wrist frame: forward = middle_base - wrist; across = index_base -
pinky_base; normal = forward x across.
"""
import json, socket, sys, os
import numpy as np

RETARGET_DIR = "/home/yam/manus/sharpa-manus-sdk/retargeting_alg_release_V4.0"
sys.path.insert(0, os.path.join(RETARGET_DIR, "include", "proto_hand"))
import zmq
import sharpa_hand_pb2

WRIST, IDX_BASE, MID_BASE, PNK_BASE = 0, 5, 10, 20


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return [float(w), float(x), float(y), float(z)]


def wrist_quat(poses):
    if len(poses) <= PNK_BASE:
        return None
    def p(i):
        q = poses[i].position
        return np.array([q.x, q.y, q.z])
    w, idx, mid, pnk = p(WRIST), p(IDX_BASE), p(MID_BASE), p(PNK_BASE)
    fwd = mid - w
    if np.linalg.norm(fwd) < 1e-6:
        return None
    fwd /= np.linalg.norm(fwd)
    across = idx - pnk
    if np.linalg.norm(across) < 1e-6:
        return None
    normal = np.cross(fwd, across)
    if np.linalg.norm(normal) < 1e-6:
        return None
    normal /= np.linalg.norm(normal)
    across = np.cross(normal, fwd)          # re-orthogonalize
    R = np.column_stack([across, normal, fwd])
    return mat_to_quat(R)


def main():
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect("tcp://127.0.0.1:2044")
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = ("127.0.0.1", 9872)
    print("Sharpa keypoints 2044 -> wrist quat udp 9872", file=sys.stderr)
    while True:
        data = sub.recv()
        m = sharpa_hand_pb2.MocapKeypoints()
        try:
            m.ParseFromString(data)
        except Exception:
            continue
        gloves = []
        for side, poses in (("left", m.left_mocap_pose), ("right", m.right_mocap_pose)):
            q = wrist_quat(poses)
            if q is not None:
                gloves.append({"side": side, "q": q})
        if gloves:
            out.sendto(json.dumps({"gloves": gloves}).encode(), dst)


if __name__ == "__main__":
    sys.exit(main())
