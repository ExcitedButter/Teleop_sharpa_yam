#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Local rebuild of hermes' /home/yam/sharpa-venv/sharpa_hand_driver.py for
jdw-Lambda-Vector, on the SharpaWave SDK bundled with the sharpa-manus-sdk
clone (include/SharpaWaveSDK_4.6.6).

One driver per hand. It:
  - discovers the Sharpa Wave hands on the LAN (SDK UDP heartbeat), connects
    to the one whose device info matches --side, and logs the same lines the
    panel's driver_connected_side() parses:
        Creating new connection for device: <sn>
        sn = <sn>, hand_side = <Left|Right>
  - homes the hand to the fully-open pose (all-zeros 22-vector, interpolated)
    on connect - an all-zeros target is the calibrated open/home hand;
  - listens on --listen (udp, JSON list of 22 joint radians, Sharpa order)
    and streams each clamped target to the hand (SDK interpolation on);
  - clamps every command to LIMITS_DEG (the hermes driver's conservative
    limits - viz_panel.py carries the same table for the sim);
  - with --tee, forwards {"t": ..., "side": ..., "hand": [22 clamped rad]}
    to the recorder;
  - on SIGINT/SIGTERM homes the hand, stops it and releases the connection.

If the hand is not on the network yet (cable/power), it keeps retrying every
3 s - the tile shows the driver up and the hand connects when it appears.
"""
import argparse
import json
import math
import os
import signal
import socket
import sys
import time

_SDK_PY = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                       "sharpa-manus-sdk", "retargeting_alg_release_V4.0",
                       "include", "SharpaWaveSDK_4.6.6", "python")
sys.path.insert(0, _SDK_PY)
from sharpa import SharpaWaveManager, ControlMode, ControlSource  # noqa: E402

# hermes driver's conservative clamp, degrees, Sharpa 22-joint order
# (thumb 5, index 4, middle 4, ring 4, pinky 5)
LIMITS_DEG = [
    (0, 60), (-15, 15), (0, 60), (-15, 15), (0, 80),
    (0, 100), (-20, 20), (0, 110), (0, 90),
    (0, 100), (-20, 20), (0, 110), (0, 90),
    (0, 100), (-20, 20), (0, 110), (0, 90),
    (0, 30), (0, 100), (-20, 20), (0, 110), (0, 90),
]
LIMITS_RAD = [(math.radians(lo), math.radians(hi)) for lo, hi in LIMITS_DEG]
HOME = [0.0] * 22          # fully-open calibrated pose
HOME_HOLD_S = 2.5          # let the interpolated homing move arrive


def log(msg):
    print(msg, flush=True)


def device_side(hand, sn):
    """'left'/'right' from the device info: hand_side 0=left, 1=right
    (verified on this rig 2026-08-27: CE52913DCE52 side 0 @ .10,
    CE5E9131CE5E side 1 @ .20). Falls back to the known static IPs."""
    try:
        info = hand.get_device_info_json()
        if isinstance(info, tuple):
            info = info[-1]
        d = json.loads(info) if isinstance(info, str) else dict(info)
    except Exception as e:
        log("device info failed for %s: %s" % (sn, e))
        return None
    log("device %s: ip=%s hand_side=%s fw=%s fault=%s"
        % (sn, d.get("ip"), d.get("hand_side"), d.get("firmware_version"),
           (d.get("status") or {}).get("fault_code")))
    hs = d.get("hand_side")
    if hs in (0, 1):
        return "left" if hs == 0 else "right"
    return {"192.168.10.10": "left", "192.168.10.20": "right"}.get(d.get("ip", ""))


def connect_side(mgr, side):
    """Connect to the hand whose reported side matches; None if not found."""
    try:
        sns = list(mgr.get_all_device_sn())
    except Exception as e:
        log("device scan failed: %s" % e)
        return None, None
    if not sns:
        return None, None
    log("devices on the wire: %s" % ", ".join(sns))
    for sn in sns:
        try:
            hand = mgr.connect(sn)
        except Exception as e:
            log("connect %s failed: %s" % (sn, e))
            continue
        got = device_side(hand, sn)
        if got == side or (got is None and len(sns) == 1):
            # the panel greps for exactly these two lines
            log("Creating new connection for device: %s" % sn)
            log("sn = %s, hand_side = %s" % (sn, side.capitalize()))
            if got is None:
                log("WARNING: device info did not state a side - single device "
                    "on the wire, assuming it is the %s hand" % side)
            return hand, sn
        log("%s is the %s hand - not ours, disconnecting" % (sn, got))
        try:
            mgr.disconnect(sn)
        except Exception:
            pass
    return None, None


def init_hand(hand):
    for name, call in (("control mode", lambda: hand.set_control_mode(ControlMode.POSITION)),
                       ("speed coeff", lambda: hand.set_speed_coeff(0.3)),
                       ("current coeff", lambda: hand.set_current_coeff(0.6)),
                       ("control source", lambda: hand.set_control_source(ControlSource.SDK))):
        err = call()
        if getattr(err, "code", 0) != 0:
            log("failed to set %s: %s" % (name, getattr(err, "message", err)))
            return False
    hand.start()
    err = hand.set_joint_position(list(HOME), True)   # home = open, interpolated
    if getattr(err, "code", 0) != 0:
        log("homing command failed: %s" % getattr(err, "message", err))
    return True


def clamp(vec):
    return [min(max(float(v), lo), hi) for v, (lo, hi) in zip(vec, LIMITS_RAD)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="127.0.0.1:59201")   # host:port
    ap.add_argument("--side", required=True, choices=("left", "right"))
    ap.add_argument("--tee", default=None)                   # host:port
    args = ap.parse_args()

    host, port = args.listen.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, int(port)))
    sock.settimeout(0.5)
    tee_sock, tee_dst = None, None
    if args.tee:
        th, tp = args.tee.rsplit(":", 1)
        tee_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tee_dst = (th, int(tp))

    stop = {"flag": False}
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *a: stop.__setitem__("flag", True))

    mgr = SharpaWaveManager.get_instance()
    time.sleep(1.0)   # let discovery hear the first heartbeats
    hand, sn = None, None
    last_try = 0.0
    log("%s sharpa driver listening on udp %s" % (args.side, args.listen))
    try:
        while not stop["flag"]:
            if hand is None and time.time() - last_try > 3.0:
                last_try = time.time()
                hand, sn = connect_side(mgr, args.side)
                if hand is not None:
                    if init_hand(hand):
                        log("%s hand %s connected + homing to open pose" % (args.side, sn))
                    else:
                        log("init failed for %s - disconnecting, will retry" % sn)
                        try:
                            mgr.disconnect(sn)
                        except Exception:
                            pass
                        hand, sn = None, None
            try:
                data, _ = sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                continue
            try:
                vec = json.loads(data.decode())
            except ValueError:
                continue
            if not (isinstance(vec, list) and len(vec) == 22):
                continue
            tgt = clamp(vec)
            if hand is not None:
                try:
                    hand.set_joint_position(tgt, True)
                except Exception as e:
                    log("command failed (%s) - reconnecting" % e)
                    try:
                        mgr.disconnect(sn)
                    except Exception:
                        pass
                    hand, sn = None, None
            if tee_sock is not None:
                tee_sock.sendto(json.dumps(
                    {"t": time.time(), "side": args.side, "hand": tgt}).encode(),
                    tee_dst)
    finally:
        if hand is not None:
            log("homing + releasing the %s hand ..." % args.side)
            try:
                hand.set_joint_position(list(HOME), True)
                time.sleep(HOME_HOLD_S)
                hand.stop()
            except Exception:
                pass
        try:
            mgr.disconnect_all()
        except Exception:
            pass
        log("%s driver stopped" % args.side)
    return 0


if __name__ == "__main__":
    sys.exit(main())
