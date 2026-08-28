#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
# ============================================================
# teleop_panel.py  -  one-window control panel for the teleop rig
#
#   ~/Desktop/Zhi/teleop_panel.py    (on hermes; system python3 has no tkinter
#                                     and apt python3-tk is uninstallable there,
#                                     so the shebang uses miniconda's python)
#   python3 ~/Desktop/Zhi/teleop_panel.py --check    verify the deploy: display,
#                                     yambox ssh, i2rt/sharpa/wuji paths, manus
#                                     dongles (present+writable), sharpa hand
#                                     pings, docker - no GUI, touches nothing,
#                                     exits READY / NOT READY
#
# A 4x2 grid of devices, LEFT and RIGHT fully separate, each with its
# own Start / Quit button, status light and progress line:
#
#     Yambox follower   the arm on the yambox (can1:11334 / can0:11333)
#     Teacher arm       the local leader (can_leader_l / can_leader_r)
#     Sharpa hand       right = sharpa_hand_driver.py (teaching-handle UDP)
#                       left  = the Sharpa desktop app (wuji-glove path)
#     Wuji glove        Wuji Studio + the wuji_sdk glove connection. Start =
#                       connect: studio up, glove verified, and the glove->hand
#                       bridge resumed if a Quit here had stopped it. Quit =
#                       disconnect: the wuji-bridge SDK stream is killed (hand
#                       stops following at once, holds pose) and Studio closes
#                       unless the other side's glove is still active.
#
# plus per-side "Glove->hand" tiles: the wuji-bridge containers
# (wuji_sharpa_bridge.py) that read each glove via wuji_sdk (pinned by SN)
# and stream the retargeted 22-joint vector to that hand's
# sharpa_hand_driver (left udp 59201, right 59202) - the piece that
# actually makes a glove drive its Sharpa hand. Start brings up the side's
# sharpa_hand_driver itself if missing (the "Sharpa hand" L tile is only
# the Pilot desktop app); it just needs the Wuji glove side up first.
#
# Every ~6 s a poller re-checks reality: a device that dies goes RED and
# its Start button brings it back - nothing else is torn down. Closing
# the window leaves everything running (use "Quit ALL" to stop the rig).
#
# Status lights:  grey OFF · yellow STARTING · green LIVE · red DEAD
#                 orange STUCK (teacher-arm control loop alive but its
#                 io worker stopped advancing - "process alive" is not
#                 health, same trick as start_teleop.sh)
#
# Suggested start order:  yambox -> sharpa / wuji glove -> teacher arm
# (the teacher arm refuses to start until its follower port is up).
#
# Env overrides:
#   YAMBOX / YAMBOX_IP  follower host (DHCP moves it), I2RT= its repo path
#   PANEL_AUTOSTART   comma list like "yambox:left,sharpa:right,leader:left"
#                     (the glove->hand bridges are "bridge:left" / "bridge:right")
#                     - the panel presses Start on each in order, waiting
#                     for one to finish before the next (a start_teleop.sh
#                     style bring-up, but with the UI watching everything)
#   WUJI_STUDIO_CMD   launch command for Wuji Studio (else PATH, /opt,
#                     AppImages and .desktop files are searched)
#   WUJI_PY           python that can `import wuji_sdk`
#   WUJI_LEFT_SN / WUJI_RIGHT_SN   pin each glove's serial
#   SHARPA_APP_CMD    command to launch the Sharpa desktop app; without
#                     it the left-hand Start just waits for you to open it
#   SHARPA_APP_PATTERN  ps regex for the app (default "[Ss]harpa",
#                     sharpa_hand_driver is always excluded)
# ============================================================
import glob
import json
import os
import queue
import sys
import re
import shlex
import shutil
import socket
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext

# ---------------- config (env-overridable: DHCP moves the yambox) ----------------
YAMBOX = os.environ.get("YAMBOX", "yambox@192.168.1.9")
YAMBOX_IP = os.environ.get("YAMBOX_IP", "192.168.1.9")
YAMBOX_PW = "root"
HERMES_PW = "yam"
I2RT = os.environ.get("I2RT", "/home/yam/gck/i2rt")
PY = I2RT + "/.venv/bin/python"
GELLO = I2RT + "/examples/minimum_gello/minimum_gello.py"
PING = I2RT + "/i2rt/motor_config_tool/ping_motors.py"
GELLO_REMOTE = "examples/minimum_gello/minimum_gello.py"
PING_REMOTE = "i2rt/motor_config_tool/ping_motors.py"
LOGDIR = os.path.expanduser("~/teleop_logs")
os.makedirs(LOGDIR, exist_ok=True)

SIDES = {
    "left":  dict(leader_can="can_leader_l", fol_can="can1", port=11334),
    "right": dict(leader_can="can_leader_r", fol_can="can0", port=11333),
}
# both grippers were replaced by Sharpa hands (Ethernet, not CAN);
# put linear_4310 back here if an original gripper is ever reinstalled
FOLLOWER_GRIPPER = {"left": "no_gripper", "right": "no_gripper"}
# neither leader streams its teaching handle anymore: BOTH hands are now
# glove-driven (left glove since 08-17, right glove added 08-18). To hand a
# side back to its teaching handle put ["--gripper-udp", "127.0.0.1:5920X"]
# here - but never while that side's glove bridge runs, or the two sources
# fight over the hand at 60 Hz each.
LEADER_EXTRA = {"left": [], "right": []}

SHARPA_PY = os.environ.get("SHARPA_PY", "/home/yam/sharpa-venv/bin/python")
SHARPA_DRIVER = os.environ.get("SHARPA_DRIVER", "/home/yam/sharpa-venv/sharpa_hand_driver.py")
SHARPA_SDK_DIR = os.environ.get("SHARPA_SDK_DIR", "/opt/sharpa-wave-sdk")
if not os.path.exists(SHARPA_DRIVER):
    # jdw-Lambda-Vector (2026-08-27): hermes' driver is gone - use the local
    # rebuild next to this file, on the SharpaWave SDK bundled in the
    # sharpa-manus-sdk clone (same 22-joint UDP contract, LIMITS_DEG clamp)
    _here = os.path.dirname(os.path.abspath(__file__))
    SHARPA_PY = sys.executable
    SHARPA_DRIVER = os.path.join(_here, "sharpa_hand_driver.py")
    SHARPA_SDK_DIR = os.path.join(_here, "sharpa-manus-sdk",
                                  "retargeting_alg_release_V4.0", "include",
                                  "SharpaWaveSDK_4.6.6")
SHARPA_UDP = {"left": "127.0.0.1:59201", "right": "127.0.0.1:59202"}  # per-hand drivers
DRIVER_LOG = {"left": "sharpa_driver.log", "right": "sharpa_driver_right.log"}
REC_TEE_UDP = "127.0.0.1:59250"   # driver tees final hand actions here for the recorder
RECORDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "record_episode.py")
REC_STATUS = "/tmp/zhi_record_status.json"
SHARPA_APP_CMD = os.environ.get("SHARPA_APP_CMD", "")
SHARPA_APP_PAT = os.environ.get("SHARPA_APP_PATTERN", "[Ss]harpa")

WUJI_STUDIO_PAT = r"[Ww]uji.?[Ss]tudio"
WUJI_IMAGE = os.environ.get("WUJI_IMAGE", "wuji:latest")
# one bridge container per glove; "wuji-bridge" is the legacy left name -
# keep it so a live left bridge survives panel upgrades
BRIDGE_NAME = {"left": "wuji-bridge", "right": "wuji-bridge-right"}
# extra bridge args per side: BOTH hands run the v0 raw-angle mapping
# (no rest-pose auto-zero) since 2026-08-18 - the left is the sign-reversed
# twin of the right's measured config (see wuji_sharpa_bridge.py main())
BRIDGE_EXTRA = {"left": " --no-zero", "right": " --no-zero"}
BRIDGE_SCRIPT_DIR = "/home/yam/wuji"  # wuji_sharpa_bridge.py lives here
# default SNs pinned 2026-08-18: BOTH gloves are now online at once, and an
# unpinned bridge connects to whichever scans first (it grabbed the right
# glove while the left one was being worn - "bridge doesn't work")
WUJI_SN = {"left": os.environ.get("WUJI_LEFT_SN", "WG1JA02260417005"),
           "right": os.environ.get("WUJI_RIGHT_SN", "WG1KA06260701528")}

# ---- MANUS: an ALTERNATIVE glove source for the same two Sharpa hands ----
# Pipeline (see ~/manus/manus_sharpa/README.md):
#   manus_ergo_stream.out  (C++, ManusSDK integrated) reads BOTH gloves -> udp/9871
#   manus_sharpa_bridge.py (miniconda)  retarget -> udp 59201 (L) / 59202 (R)
#   sharpa_hand_driver.py --side left/right  (the same drivers the wuji path uses)
# Manus and Wuji both feed 59201/59202, so only ONE source may drive a hand at
# a time (two sources = the hand fights itself). The source binary uses Core
# Integrated = sole ownership of the dongles, so no other integrated MANUS
# process (dashboard, manus_license.out) can run alongside it.
MANUS_PY = "/home/yam/miniconda3/bin/python"
MANUS_DIR = "/home/yam/manus/manus_sharpa"
# unified Manus source (2026-08-21): one Core-Integrated process emits BOTH the
# ergonomics (fingers, udp 9871) AND the raw skeleton + wrist quaternion (arm
# wrist, udp 9872). Replaces the old manus_ergo_stream.out - only one integrated
# process may own the dongles, so fingers and wrist must come from one binary.
MANUS_ERGO = "/home/yam/manus/manus_skeleton_stream.out"
MANUS_BRIDGE = os.path.join(MANUS_DIR, "manus_sharpa_bridge.py")
MANUS_ERGO_PORT = 9871          # source -> finger bridge
MANUS_WRIST_PORT = 9872         # source -> arm teleop
MANUS_SRC_PROC = r"manus_skeleton_stream"   # ps pattern for the unified source
# arm teleop: glove wrist -> IK -> follower (runs in the i2rt venv)
GCK_PY = "/home/yam/gck/i2rt/.venv/bin/python"
MANUS_ARM_TELEOP = "/home/yam/manus/manus_arm_teleop.py"   # still used with --reset to home the FOLLOWERS
if not os.path.exists(MANUS_ARM_TELEOP):
    # jdw-Lambda-Vector (2026-08-27): /home/yam is gone - use the ORIGINAL
    # hermes manus_arm_teleop.py (recovered 2026-08-27, deployed next to this
    # file with only its /home/yam paths rewritten to Zhi/params + Zhi/i2rt).
    # Full original behavior: ssik analytical IK, --reset/--loose/--mirror-home
    # and live vive teleop. Runs in the panel's venv (numpy/portal/mujoco/ssik).
    MANUS_ARM_TELEOP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "manus_arm_teleop.py")
    GCK_PY = sys.executable
LEADER_RESET = "/home/yam/manus/leader_reset.py"           # match a leader arm to its follower's pose

# OFFICIAL Sharpa-Manus SDK retargeting (github.com/sharpa-robotics/sharpa-manus-sdk):
# the Manus "Retarget" tile runs Sharpa's own optimizer instead of the custom
# linear bridge - a headless client (gloves -> ZMQ 2044) + the retargeting demo
# (-wave -> hands over UDP). Needs OUR manus source stopped (both use the
# integrated SDK / dongles). Coexists with SharpaPilot (UDP, not exclusive SDK).
SHARPA_CLIENT_DIR = "/home/yam/manus/sharpa-manus-sdk/client"
SHARPA_CLIENT = SHARPA_CLIENT_DIR + "/SharpaManusClient.out"
SHARPA_RETARGET_DIR = "/home/yam/manus/sharpa-manus-sdk/retargeting_alg_release_V4.0"
SHARPA_RETARGET = SHARPA_RETARGET_DIR + "/retargeting_manus_demo_multiprocess.py"
SHARPAMANUS_PY = "/home/yam/miniconda3/envs/sharpamanus/bin/python"
if not os.path.exists(SHARPA_CLIENT):
    # jdw-Lambda-Vector (2026-08-27): hermes' copy is gone - use the local
    # clone of github.com/sharpa-robotics/sharpa-manus-sdk next to this file
    # (client BUILT from source there; venv from its requirements.txt)
    _sdk = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sharpa-manus-sdk")
    SHARPA_CLIENT_DIR = _sdk + "/client"
    SHARPA_CLIENT = SHARPA_CLIENT_DIR + "/SharpaManusClient.out"
    SHARPA_RETARGET_DIR = _sdk + "/retargeting_alg_release_V4.0"
    SHARPA_RETARGET = SHARPA_RETARGET_DIR + "/retargeting_manus_demo_multiprocess.py"
if not os.path.exists(SHARPAMANUS_PY):
    SHARPAMANUS_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "sharpamanus-venv/bin/python")
SHARPA_CLIENT_PROC = r"SharpaManusClient\.out"
SHARPA_RETARGET_PROC = r"retargeting_manus_demo"
HAND_IP = {"left": "192.168.10.10", "right": "192.168.10.20"}
# The demo's own -wave/-sdk hand output is blocked by SharpaPilot holding the
# hands, so we run the demo in pure-ZMQ mode (it always publishes the optimized
# 22-joint targets on tcp://*:6668) and route those through OUR sharpa_hand_driver
# (which drives the hands + coexists with SharpaPilot) via this small bridge.
SHARPA_ZMQ_BRIDGE = "/home/yam/manus/sharpa_zmq_to_driver.py"
if not os.path.exists(SHARPA_ZMQ_BRIDGE):   # local rebuild (2026-08-27)
    SHARPA_ZMQ_BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "sharpa_zmq_to_driver.py")
SHARPA_ZMQ_BRIDGE_PROC = r"sharpa_zmq_to_driver"
# The arm wrist teleop reads the wrist quaternion on udp 9872. Our own manus
# source can't run while the Sharpa client owns the gloves (single-owner
# integrated SDK), so when Sharpa finger retargeting is active this adapter
# derives the wrist orientation from the Sharpa keypoint stream (ZMQ 2044) and
# republishes it on 9872 - letting arm teleop + Sharpa fingers share one owner.
SHARPA_WRIST_ADAPTER = "/home/yam/manus/sharpa_wrist_to_udp.py"
if not os.path.exists(SHARPA_WRIST_ADAPTER):   # original, recovered 2026-08-27
    SHARPA_WRIST_ADAPTER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "sharpa_wrist_to_udp.py")
SHARPA_WRIST_PROC = r"sharpa_wrist_to_udp"
# LIVE wrist source for the arm teleop = the MANUS raw-IMU (RawDeviceData.rotation),
# covered by the Core Lite / IMU license even though the full raw-SKELETON stream is
# "Unsupported"/frozen. Standalone integrated daemon -> wrist quat on udp 9872. It
# owns the dongles, so it can't run alongside the Sharpa finger client (whose
# skeleton is frozen anyway). Needs the gloves' Quantum battery CHARGED.
MANUS_IMU_DIR = "/home/yam/manus/ManusSDK_v3.1.1/SDKMinimalClient_Linux"
MANUS_IMU_WRIST = MANUS_IMU_DIR + "/manus_imu_wrist.out"
MANUS_IMU_PROC = r"manus_imu_wrist"
# ---- VIVE trackers: the CURRENT wrist source for arm teleop (2026-08-25) ----
# Two Vive trackers (one per wrist) tracked by SteamVR running HEADLESS (null
# HMD driver). vive_wrist_stream.py (SYSTEM python3 - openvr lives there, not
# in a venv) reads both trackers and streams wrist pose - orientation AND
# position - as JSON over UDP, one port per side (two teleop processes cannot
# share one UDP port). Serial<->side mapping + SteamVR->robot axis map live in
# /home/yam/manus/vive_trackers.json. Start Teleop = reset home -> 5 s countdown
# (pose your wrists) -> capture neutral -> arms track the trackers (position
# clamped to VIVE_MAX_OFFSET m around the start pose). Fingers stay on the
# MANUS->Sharpa pipeline (its own tiles) - Start Teleop never touches them.
SYS_PY = "/usr/bin/python3"
VIVE_STREAM = "/home/yam/manus/vive_wrist_stream.py"
if not os.path.exists(VIVE_STREAM):
    # jdw-Lambda-Vector (2026-08-27): local rebuild (openvr lives in the
    # panel's venv, so run it with our own interpreter, not /usr/bin/python3)
    VIVE_STREAM = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "vive_wrist_stream.py")
    SYS_PY = sys.executable
VIVE_PROC = r"vive_wrist_stream"
VIVE_PORTS = {"left": 9873, "right": 9874}
VIVE_POS_SCALE = 1.0            # tracker meters -> EE meters (1:1)
VIVE_MAX_OFFSET = 0.35          # max EE excursion from the start pose, meters
# the two arms are mounted 180 deg apart (verified live 2026-08-25: identical
# mapping was correct on "left"/11334 and exactly opposite on "right"/11333)
VIVE_BASE_YAW = {"left": 0.0, "right": 180.0}
# the right arm's mapping is MIRRORED: with base yaw 180 its horizontals were
# correct but up/down opposite (2026-08-26) -> flip its vertical delta
VIVE_FLIP_Z = {"left": False, "right": True}
# DO NOT flip x: "cannot go inward" (2026-08-26) was the folded HOME pose
# having no retract room (model -x blocked by joint ranges from home), not a
# sign error - flipping x just moved the blocked axis onto "front". The real
# fix is VIVE_EXTEND_START: extend to mid-workspace before capturing.
VIVE_FLIP_X = {"left": False, "right": False}
# the automatic pre-capture extend move was disliked ("stop that", 2026-08-26)
# - set to 0. Tradeoff: from folded home the arm has NO room in the retract
# direction until the user first moves it forward. Re-enable with 0.15/0.15
# for balanced slack in all directions.
# kept at 0 - the operator does not want ANY automatic extend move
# (re-confirmed 2026-08-27; the workspace-slack tradeoff above still applies:
# from folded home the arm has little room until first moved forward by hand)
VIVE_EXTEND_START = 0.0    # m along the arm axis before neutral capture
VIVE_LIFT_START = 0.0      # m upward too
STEAMVR_PROC = r"bin/linux64/vrserver"
# Sharpa hand INITIAL pose used on reset / Start Teleop: the driver treats an
# all-zeros 22-vector as the fully-open/home hand (sharpa_hand_driver.py:42,156).
# On reset we drive the hands here and hold, so teleop always begins from a known
# calibrated pose instead of wherever the last glove frame left them.
SHARPA_INIT_POSE = [0.0] * 22
# how long to hold the init pose so the driver's interpolation actually REACHES
# it before the finger stream resumes. From a fully-curled hand the open move
# takes a few seconds; 1.5s was too short (left hand re-curled before arriving).
SHARPA_INIT_HOLD_S = 4.0
# any process that streams finger targets into the drivers (59201/59202) - paused
# (SIGSTOP) while we command the init pose so it isn't immediately overridden.
SHARPA_FEEDERS = [SHARPA_ZMQ_BRIDGE_PROC, r"manus_sharpa_bridge", r"wuji_sharpa_bridge"]
TELEOP_COUNTDOWN = 5            # seconds to pose the wrist / set the arms before capture
MANUS_USB_VID = "3325"          # Manus dongle USB vendor id

SSH_OPTS = ["-n", "-o", "ConnectTimeout=4", "-o", "ControlMaster=auto",
            "-o", "ControlPath=/tmp/zhi_panel_ssh", "-o", "ControlPersist=120",
            "-o", "StrictHostKeyChecking=accept-new"]

COLORS = {"OFF": "#8a8a8a", "START": "#d4a017", "LIVE": "#2e8b57",
          "STUCK": "#e07b00", "DEAD": "#c0392b"}
STATE_TXT = {"OFF": "off", "START": "starting", "LIVE": "LIVE",
             "STUCK": "STUCK", "DEAD": "DEAD"}

# ---------------- shell helpers ----------------
def sh(cmd, timeout=25, **kw):
    """Run a command, never raise - a timeout comes back as rc 124."""
    try:
        return subprocess.run(cmd, shell=isinstance(cmd, str), text=True,
                              capture_output=True, timeout=timeout, **kw)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", "timeout")
    except OSError as e:   # missing binary, lost exec bit (wuji-py 2026-08-26)
        return subprocess.CompletedProcess(cmd, 126, "", str(e))

def ssh(remote_cmd, timeout=25):
    return sh(["ssh"] + SSH_OPTS + [YAMBOX, remote_cmd], timeout)

def local_procs(pattern):
    out = []
    for line in sh(["ps", "-eo", "pid,args"], 10).stdout.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        pid, args = parts
        if re.search(pattern, args) and "teleop_panel" not in args:
            out.append((int(pid), args))
    return out

def kill_local(pattern, sig=signal.SIGTERM, group=False):
    """group=True kills each match's whole process GROUP - needed for the
    leaders: minimum_gello's multiprocessing child owns the --gripper-udp
    sender socket and survives a plain SIGTERM to its parent, then keeps
    streaming handle packets forever (two of these orphans fought the glove
    bridges on 2026-08-18 - 'hand is shaking'). spawn() starts children in
    their own session, so the group is exactly the leader and its workers."""
    n = 0
    for pid, _ in local_procs(pattern):
        try:
            if group:
                os.killpg(os.getpgid(pid), sig)
            else:
                os.kill(pid, sig)
            n += 1
        except (ProcessLookupError, PermissionError):
            pass
    return n

def spawn(cmd, logfile, cwd=None):
    """Detached child: survives the panel closing, output to logfile."""
    lf = open(logfile, "w")
    return subprocess.Popen(cmd, cwd=cwd, stdout=lf, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)

def yambox_snapshot():
    """One ssh for everything the poller needs; None = unreachable."""
    cp = ssh("ss -tln 2>/dev/null; ps -eo args | grep '[m]inimum_gello'; exit 0", 10)
    return cp.stdout if cp.returncode == 0 else None

def has_motors_16(s):
    return all(str(m) in s for m in range(1, 7))

# ---------------- wuji helpers ----------------
WUJI_SCAN_SNIPPET = ("from wuji_sdk import SdkManager\n"
                     "for d in SdkManager.instance().scan():\n"
                     "    print(d.sn, d.address, d.transport_type)\n")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def find_wuji_py():
    cands = [os.environ.get("WUJI_PY"),
             os.path.join(SCRIPT_DIR, "wuji-py"),   # docker shim (wuji_sdk lives in wuji:latest)
             os.path.join(SCRIPT_DIR, "wuji-venv/bin/python"),   # setup_wuji.sh
             os.path.expanduser("~/wuji-venv/bin/python"),
             "/home/yam/wuji-venv/bin/python",
             shutil.which("python3")]
    for c in cands:
        if c and (os.path.exists(c) or shutil.which(c)):
            if sh([c, "-c", "import wuji_sdk"], 15).returncode == 0:
                return c
    return None

def find_wuji_studio():
    if os.environ.get("WUJI_STUDIO_CMD"):
        return os.environ["WUJI_STUDIO_CMD"]
    for name in ("wuji-studio", "WujiStudio", "wuji_studio"):
        p = shutil.which(name)
        if p:
            return p
    local = os.path.join(SCRIPT_DIR, "wuji-studio/usr/bin/wuji-studio")  # setup_wuji.sh extract
    if os.access(local, os.X_OK):
        return local
    pats = [os.path.join(SCRIPT_DIR, "[Ww]uji*[Ss]tudio*.AppImage"),
            "/opt/[Ww]uji*/[Ww]uji*", "/opt/[Ww]uji*[Ss]tudio*/*",
            os.path.expanduser("~/[Ww]uji*[Ss]tudio*.AppImage"),
            os.path.expanduser("~/Applications/[Ww]uji*[Ss]tudio*.AppImage"),
            os.path.expanduser("~/Downloads/[Ww]uji*[Ss]tudio*.AppImage")]
    for pat in pats:
        for c in glob.glob(pat):
            if os.access(c, os.X_OK) and not os.path.isdir(c):
                return c
    for d in (os.path.expanduser("~/.local/share/applications"),
              "/usr/share/applications"):
        for f in glob.glob(d + "/*.desktop"):
            try:
                txt = open(f).read()
            except OSError:
                continue
            if re.search(r"wuji.?studio", txt, re.I):
                m = re.search(r"^Exec=(.+)$", txt, re.M)
                if m:
                    return re.sub(r"%[A-Za-z]", "", m.group(1)).strip()
    return None

def wuji_scan(wpy):
    cp = sh([wpy, "-c", WUJI_SCAN_SNIPPET], 20)
    return [l for l in cp.stdout.splitlines() if l.strip()]

def sharpa_app_procs():
    return [(p, a) for p, a in local_procs(SHARPA_APP_PAT)
            if "sharpa_hand_driver" not in a]

# ---------------- --check: deploy verification, no GUI, no side effects ----
if "--check" in sys.argv:
    ok = True
    def chk(label, good, detail=""):
        global ok
        if not good:
            ok = False
        print("  [%s] %-26s %s" % ("PASS" if good else "FAIL", label, detail))
    print("teleop_panel deploy check on %s:" % os.uname().nodename)
    try:
        _r = tk.Tk(); _r.withdraw(); _r.destroy()
        chk("tkinter / display", True)
    except tk.TclError as e:
        chk("tkinter / display", False, str(e).splitlines()[0])
    _cp = ssh("true", 8)
    if _cp.returncode == 0:
        chk("yambox ssh", True, YAMBOX)
        # the follower start uses ~/i2rt on the yambox (not I2RT, which is
        # the LOCAL leader-side repo) - verify those remote files too
        _r = ssh("ls ~/i2rt/.venv/bin/python ~/i2rt/%s ~/i2rt/%s"
                 " >/dev/null 2>&1 && echo OK" % (GELLO_REMOTE, PING_REMOTE), 12)
        chk("yambox ~/i2rt repo", "OK" in _r.stdout,
            "venv + minimum_gello + ping_motors on the yambox")
    elif sh(["ping", "-c1", "-W1", YAMBOX_IP], 8).returncode == 0:
        chk("yambox ssh", False,
            "%s pings but ssh is refused - no key on this machine? "
            "run: ssh-copy-id %s" % (YAMBOX_IP, YAMBOX))
    else:
        chk("yambox ssh", False, "%s unreachable - power? DHCP moved IP?" % YAMBOX)
    for _p, _what in ((PY, "leader i2rt python"), (GELLO, "leader minimum_gello"),
                      (PING, "leader ping_motors")):
        chk(_what, os.path.exists(_p), _p)
    for _side, _cfg in SIDES.items():
        _c = _cfg["leader_can"]
        chk("CAN %s" % _c, sh(["ip", "link", "show", _c], 8).returncode == 0,
            "leader %s" % _side)
    chk("sharpa driver", os.path.exists(SHARPA_DRIVER), SHARPA_DRIVER)
    chk("sharpa venv python", os.path.exists(SHARPA_PY), SHARPA_PY)
    _wpy = find_wuji_py()
    chk("wuji python (wuji_sdk)", bool(_wpy), _wpy or "run setup_wuji.sh, or set WUJI_PY=")
    _stu = find_wuji_studio()
    chk("wuji studio", bool(_stu), _stu or "run setup_wuji.sh, or set WUJI_STUDIO_CMD=")
    # Manus dongles: present on local USB, and writable for the integrated SDK
    # (manus_dongle_nodes() is defined below this early-exit block, so inline)
    _nodes = []
    for _b in glob.glob("/sys/bus/usb/devices/*"):
        try:
            if open(_b + "/idVendor").read().strip() == MANUS_USB_VID:
                _nodes.append("/dev/bus/usb/%03d/%03d" % (
                    int(open(_b + "/busnum").read()), int(open(_b + "/devnum").read())))
        except OSError:
            pass
    chk("manus dongles (x2)", len(_nodes) >= 2,
        "%d plugged in (VID %s)" % (len(_nodes), MANUS_USB_VID))
    if _nodes:
        _ro = [n for n in _nodes if not os.access(n, os.W_OK)]
        chk("manus dongle perms", not _ro,
            "all writable" if not _ro else
            "SDK needs write on %s - udev rule for VID %s, or the panel's "
            "docker chmod" % (", ".join(_ro), MANUS_USB_VID))
    # Sharpa hands are Ethernet devices on the 192.168.10.x link
    for _side in ("left", "right"):
        _hip = HAND_IP[_side]
        chk("sharpa hand %s ping" % _side,
            sh(["ping", "-c1", "-W1", _hip], 8).returncode == 0,
            "%s - hand powered? 192.168.10.x NIC up with an address?" % _hip)
    if sh(["docker", "ps"], 10).returncode == 0:
        chk("docker access", True, "ok")
    elif sh(["sg", "docker", "-c", "docker ps"], 10).returncode == 0:
        chk("docker access", False,
            "group added but not active in this login - log out/in "
            "(or launch the panel via: sg docker -c ./teleop_panel.py)")
    else:
        chk("docker access", False,
            "needed by wuji-py shim + dongle chmod - add this user to the docker group")
    print("result: %s" % ("READY" if ok else "NOT READY - fix the FAILs above"))
    sys.exit(0 if ok else 1)

# ---------------- device model ----------------
class Device:
    def __init__(self, kind, side, start_fn, stop_fn):
        self.kind, self.side = kind, side
        self.tag = "%s %s" % (kind, side[0].upper())
        self.start_fn, self.stop_fn = start_fn, stop_fn
        self.state, self.busy, self.cancel = "OFF", False, False
        self.dot = self.state_lbl = self.msg_lbl = None   # set by build_ui

devices = {}
ui_q = queue.Queue()

def dlog(dev, msg):
    ui_q.put(("log", "[%s] [%s] %s" % (time.strftime("%H:%M:%S"),
                                       dev.tag if dev else "panel", msg)))
    if dev:
        ui_q.put(("msg", dev, msg))

def set_state(dev, state, msg=None):
    dev.state = state
    ui_q.put(("state", dev, state, msg))

# ---------------- start / stop actions ----------------
def start_follower(dev, log):
    cfg = SIDES[dev.side]
    fol, port, grip = cfg["fol_can"], cfg["port"], FOLLOWER_GRIPPER[dev.side]
    log("checking the yambox ...")
    if ssh("true", 8).returncode != 0:
        log("cannot reach the yambox (%s) - is it on? did DHCP move its IP?" % YAMBOX_IP)
        return False
    snap = yambox_snapshot() or ""
    if "--can-channel %s" % fol in snap and re.search(r":%d\b" % port, snap):
        log("follower already running on :%d" % port)
        return True
    log("clearing any stale follower on %s ..." % fol)
    ssh("pkill -f 'minimum_gello.*%s'" % fol)
    time.sleep(3)
    log("preflight: pinging motors on %s (up to 4 cold sweeps) ..." % fol)
    online = ""
    for _ in range(4):
        if dev.cancel:
            return False
        cp = ssh("cd ~/i2rt && timeout 40 .venv/bin/python %s --channel %s 2>/dev/null"
                 " | sed -n 's/^online motors: //p' | tail -1" % (PING_REMOTE, fol), 60)
        online = cp.stdout.strip() or online
        if has_motors_16(online):
            break
    log("online motors: %s" % (online or "[]"))
    if not has_motors_16(online):
        log("arm motors 1-6 not responding - try 'Reset CAN', then Start again")
        return False
    log("starting follower %s -> :%d (gripper %s) ..." % (fol, port, grip))
    ssh("cd ~/i2rt && setsid nohup .venv/bin/python %s --mode follower"
        " --can-channel %s --arm yam_ultra --gripper %s --bilateral-kp 0.2"
        " --server-port %d > /tmp/follower_%s.log 2>&1 < /dev/null &"
        % (GELLO_REMOTE, fol, grip, port, fol), 25)
    for _ in range(10):
        time.sleep(3)
        if re.search(r":%d\b" % port, ssh("ss -tln", 8).stdout):
            log("follower listening on :%d" % port)
            return True
    log("port :%d never came up - see yambox /tmp/follower_%s.log" % (port, fol))
    return False

def stop_follower(dev, log):
    fol = SIDES[dev.side]["fol_can"]
    ssh("pkill -f 'minimum_gello.*%s'" % fol)
    log("follower on %s stopped" % fol)

def start_leader(dev, log):
    cfg = SIDES[dev.side]
    chan, port = cfg["leader_can"], cfg["port"]
    if not re.search(r":%d\b" % port, ssh("ss -tln", 8).stdout):
        log("follower port :%d is not up - start the yambox %s follower first"
            % (port, dev.side))
        return False
    cp = sh(["ip", "link", "show", chan], 8)
    if cp.returncode != 0:
        log("%s does not exist on this machine" % chan)
        return False
    if "UP" not in cp.stdout:
        log("%s was down - bringing it up (sudo)" % chan)
        sh("echo %s | sudo -S -p '' ip link set %s up type can bitrate 1000000"
           % (HERMES_PW, chan), 15)
        time.sleep(2)
    logf = os.path.join(LOGDIR, "leader_%s.log" % dev.side)
    for attempt in range(1, 6):
        if dev.cancel:
            log("start cancelled")
            return False
        kill_local(r"python.*minimum_gello.*%s\b" % chan, group=True)
        time.sleep(4)
        log("attempt %d/5: warm-up ping on %s ..." % (attempt, chan))
        sh([PY, PING, "--channel", chan], 25)
        spawn([PY, "-u", GELLO, "--mode", "leader", "--can-channel", chan,
               "--arm", "yam", "--gripper", "yam_teaching_handle",
               "--bilateral-kp", "0.2", "--server-host", YAMBOX_IP,
               "--server-port", str(port)] + LEADER_EXTRA[dev.side], logf)
        log("attempt %d/5: waiting ~35 s for the leader to sync ..." % attempt)
        time.sleep(35)
        txt = open(logf).read() if os.path.exists(logf) else ""
        if "Current follower joint pos" in txt:
            io_prev[dev.side] = -1   # fresh log file - don't flag STUCK off stale counts
            log("teacher arm UP (attempt %d)" % attempt)
            return True
        err = re.findall(r"fail to communicate with the motor \d+|No encoders found", txt)
        log("attempt %d failed: %s" % (attempt, err[-1] if err else "no sync line in log"))
    log("teacher arm would not start - power/CAN cable? see %s" % logf)
    return False

def stop_leader(dev, log):
    chan = SIDES[dev.side]["leader_can"]
    n = kill_local(r"python.*minimum_gello.*%s\b" % chan, group=True)
    log("stopped %d teacher-arm process group(s)" % n)

def driver_connected_side(side):
    """Which hand (LEFT/RIGHT) this side's driver grabbed, or None if unknown.
    Parsed from its log: DeviceInfo lines map sn->side, the last 'Creating new
    connection' line says which sn it actually connected."""
    try:
        txt = open(os.path.join(LOGDIR, DRIVER_LOG[side])).read()
    except OSError:
        return None
    sides = dict(re.findall(r"sn = (\w+),.*?hand_side = (\w+)", txt))
    conns = re.findall(r"Creating new connection for device: (\w+)", txt)
    return sides.get(conns[-1]) if conns else None

def ensure_sharpa_driver(log, side):
    """Start this side's sharpa_hand_driver if not already running. True when up."""
    missing = [p for p in (SHARPA_PY, SHARPA_DRIVER, SHARPA_SDK_DIR)
               if not os.path.exists(p)]
    if missing:
        log("sharpa driver is NOT DEPLOYED on this machine - missing %s "
            "(the hermes /home/yam stack). Restore it or point SHARPA_PY/"
            "SHARPA_DRIVER/SHARPA_SDK_DIR at a local install."
            % ", ".join(missing))
        return False
    pat = r"sharpa_hand_driver.*" + re.escape(SHARPA_UDP[side])
    # A second driver pinned to the same --side but a DIFFERENT port also holds
    # an SDK connection to the same physical hand -> the two command streams
    # fight and the hand SHAKES (seen 2026-08-21: stray --side left on :59211
    # from another user). We can't kill another user's process, so warn loudly.
    others = [a for _, a in local_procs(r"sharpa_hand_driver")
              if re.search(r"--side %s\b" % side, a)
              and SHARPA_UDP[side] not in a]
    if others:
        log("WARNING: another '--side %s' driver is running on a different port "
            "- it fights this one for the hand (SHAKING). Kill it: %s"
            % (side, others[0].split("sharpa_hand_driver")[-1].strip()[:60]))
    if local_procs(pat):
        return True
    log("starting %s sharpa hand driver (udp %s) ..." % (side, SHARPA_UDP[side]))
    # --side pins the driver to this hand: both hands broadcast and an
    # unpinned driver grabs whichever heartbeat lands first ("wrong hand").
    # --tee only on the left: record_episode.py listens on one port.
    spawn([SHARPA_PY, SHARPA_DRIVER, "--listen", SHARPA_UDP[side], "--side", side]
          + (["--tee", REC_TEE_UDP] if side == "left" else []),
          os.path.join(LOGDIR, DRIVER_LOG[side]), cwd=SHARPA_SDK_DIR)
    time.sleep(3)
    if local_procs(pat):
        got = driver_connected_side(side)
        if got:
            log("driver up - connected the %s hand (homing it)" % got)
        elif local_procs(r"pilot_sdk"):
            log("driver up, no hand yet - note SharpaPilot's pilot_sdk is running "
                "and can hold the hand's ports; close SharpaPilot if it never connects")
        return True
    log("driver died right away - see %s/%s" % (LOGDIR, DRIVER_LOG[side]))
    return False

# Each hand tile (Left hand / Right hand): Start = connect that hand's driver
# (grabs the hand) and it homes to the initial/open pose on connect. Stop =
# SIGINT the driver (it homes + releases the hand on the way out).
def start_sharpa_side(side):
    def start(dev, log):
        if not ensure_sharpa_driver(log, side):
            return False
        log("%s hand connected + homed to initial pose" % side)
        return True
    return start

def stop_sharpa_side(side):
    def stop(dev, log):
        n = kill_local(r"sharpa_hand_driver.*" + re.escape(SHARPA_UDP[side]),
                       signal.SIGINT)
        log("%s hand driver stopped (%d) - hand homes + releases" % (side, n))
    return stop

start_sharpa_right, stop_sharpa_right = start_sharpa_side("right"), stop_sharpa_side("right")
start_sharpa_left, stop_sharpa_left = start_sharpa_side("left"), stop_sharpa_side("left")

# set when Quit on a wuji tile kills that side's running glove->hand bridge,
# so the next wuji Start restores exactly the state that Quit tore down
BRIDGE_RESUME = {"left": False, "right": False}

def mk_start_wuji(side):
    def start(dev, log):
        if local_procs(WUJI_STUDIO_PAT):
            log("wuji studio already running")
        else:
            cmd = find_wuji_studio()
            if not cmd:
                log("cannot find Wuji Studio - set WUJI_STUDIO_CMD=/path/to/it")
                return False
            log("starting wuji studio: %s" % cmd)
            spawn(shlex.split(cmd), os.path.join(LOGDIR, "wuji_studio.log"))
            time.sleep(6)
        wpy = find_wuji_py()
        if not wpy:
            log("no python with wuji_sdk (pip install wuji-sdk, or set WUJI_PY=)")
            return False
        sn = WUJI_SN[side]
        log("scanning for the %s glove%s - power it on ..." %
            (side, " (%s)" % sn if sn else ""))
        for _ in range(12):
            if dev.cancel:
                return False
            lines = wuji_scan(wpy)
            for l in lines:
                log("  seen: " + l)
            if lines and (not sn or any(sn in l for l in lines)):
                log("%s glove connected (calibration: Wuji Studio -> Device -> Calibrate)" % side)
                b = devices.get(("bridge", side))
                if BRIDGE_RESUME[side] and b and not bridge_running(side):
                    BRIDGE_RESUME[side] = False
                    log("resuming the glove->hand bridge that Quit disconnected ...")
                    on_start(b)
                return True
            time.sleep(3)
        log("glove not seen - check power / receiver, then press Start to rescan")
        return False
    return start

def mk_stop_wuji(side):
    other = "right" if side == "left" else "left"
    def stop(dev, log):
        # the glove's live SDK stream is its bridge container: quitting the
        # glove disconnects it first, so the hand stops following immediately
        if bridge_running(side):
            sh(["docker", "rm", "-f", BRIDGE_NAME[side]], 20)
            BRIDGE_RESUME[side] = True
            b = devices.get(("bridge", side))
            if b:
                set_state(b, "OFF", "disconnected with the wuji glove")
            log("glove SDK stream (%s) disconnected - hand holds its last pose"
                % BRIDGE_NAME[side])
        o = devices.get(("wuji", other))
        if o and o.state in ("LIVE", "START"):
            log("%s glove still active - leaving wuji studio running" % other)
        else:
            n = kill_local(WUJI_STUDIO_PAT)
            log("closed wuji studio (%d process(es)) - glove disconnected" % n)
    return stop

def bridge_running(side):
    cp = sh(["docker", "ps", "--filter", "name=^%s$" % BRIDGE_NAME[side],
             "--format", "{{.Names}}"], 10)
    return BRIDGE_NAME[side] in cp.stdout.split()

def bridge_logs(side):
    cp = sh(["docker", "logs", BRIDGE_NAME[side]], 10)
    return cp.stdout + cp.stderr

def start_bridge(dev, log):
    """Glove -> hand: run wuji_sharpa_bridge.py in the wuji container, which
    reads this side's glove (wuji_sdk, pinned by SN) and streams 22-joint
    vectors to that hand's sharpa_hand_driver (left :59201, right :59202)."""
    side, name, udp = dev.side, BRIDGE_NAME[dev.side], SHARPA_UDP[dev.side]
    BRIDGE_RESUME[side] = False   # explicit Start supersedes any pending auto-resume
    if bridge_running(side):
        log("glove bridge already running")
        return True
    # the bridge streams to sharpa_hand_driver - bring it up ourselves if needed
    # (the "Sharpa hand" L tile is only the Pilot desktop app, not the driver)
    if not ensure_sharpa_driver(log, side):
        return False
    if not local_procs(WUJI_STUDIO_PAT):
        log("wuji studio is not running - Start 'Wuji glove' first (the glove streams through it)")
        return False
    if local_procs(r"minimum_gello.*--gripper-udp %s" % re.escape(udp)):
        log("WARNING: a teacher arm is streaming its teaching handle to the same "
            "driver - handle and glove will fight over the hand while both run")
    log("starting the glove->hand bridge (container %s) ..." % name)
    sh(["docker", "rm", "-f", name], 15)
    cp = sh(["docker", "run", "-d", "--name", name, "--network", "host",
             "-v", BRIDGE_SCRIPT_DIR + "/home:/root",
             "-v", BRIDGE_SCRIPT_DIR + ":/work", WUJI_IMAGE, "bash", "-lc",
             "cd /work && python3 wuji_sharpa_bridge.py --udp %s%s%s"
             % (udp, " --sn " + WUJI_SN[side] if WUJI_SN[side] else "",
                BRIDGE_EXTRA[side])], 30)
    if cp.returncode != 0:
        detail = cp.stderr.strip().splitlines()[-1] if cp.stderr.strip() else "rc %d" % cp.returncode
        log("docker run failed: %s" % detail)
        return False
    log("waiting for the glove to connect (up to 30 s) ...")
    for _ in range(30):
        if dev.cancel:
            sh(["docker", "rm", "-f", name], 15)
            return False
        time.sleep(1)
        lg = bridge_logs(side)
        if "connected to glove" in lg:
            m = re.search(r"connected to glove sn=\S+", lg)
            log("%s - glove is driving the hand (Quit here stops it; the hand "
                "holds its last pose)" % (m.group(0) if m else "glove connected"))
            return True
        if "ERROR" in lg or not bridge_running(side):
            errs = [l for l in lg.splitlines() if "ERROR" in l]
            log("bridge failed: %s" % (errs[-1] if errs else
                "container died - check 'docker logs %s'" % name))
            sh(["docker", "rm", "-f", name], 15)
            return False
    log("glove never connected - is it powered on and streaming in Wuji Studio?")
    sh(["docker", "rm", "-f", name], 15)
    return False

def stop_bridge(dev, log):
    sh(["docker", "rm", "-f", BRIDGE_NAME[dev.side]], 20)
    log("glove bridge stopped - hand holds its last pose; the driver keeps "
        "listening")

# ---------------- Wuji: unified Start (both gloves) + Retarget (both bridges) --
def start_wuji(dev, log):
    """Start = Wuji Studio + connect BOTH gloves."""
    if local_procs(WUJI_STUDIO_PAT):
        log("wuji studio already up")
    else:
        cmd = find_wuji_studio()
        if not cmd:
            log("Wuji Studio not found - set WUJI_STUDIO_CMD="); return False
        log("starting wuji studio ...")
        spawn(shlex.split(cmd), os.path.join(LOGDIR, "wuji_studio.log"))
        time.sleep(6)
    wpy = find_wuji_py()
    if not wpy:
        log("no python with wuji_sdk (set WUJI_PY=)"); return False
    seen = 0
    for side in ("left", "right"):
        sn = WUJI_SN[side]
        for _ in range(6):
            if dev.cancel:
                return False
            lines = wuji_scan(wpy)
            if lines and (not sn or any(sn in l for l in lines)):
                log("%s glove connected" % side); seen += 1; break
            time.sleep(2)
        else:
            log("%s glove not seen" % side)
    return seen > 0

def stop_wuji(dev, log):
    for side in ("left", "right"):
        if bridge_running(side):
            sh(["docker", "rm", "-f", BRIDGE_NAME[side]], 15)
    n = kill_local(WUJI_STUDIO_PAT)
    log("wuji studio closed (%d) - gloves disconnected" % n)

def start_wuji_retarget(dev, log):
    """Retarget = bridge BOTH gloves to the two Sharpa hands (59201/59202)."""
    ok = True
    for side in ("left", "right"):
        proxy = Device("bridge", side, start_bridge, stop_bridge)
        proxy.cancel = dev.cancel
        if not start_bridge(proxy, log):
            ok = False
    return ok

def stop_wuji_retarget(dev, log):
    for side in ("left", "right"):
        sh(["docker", "rm", "-f", BRIDGE_NAME[side]], 20)
    log("wuji retarget stopped - both hands hold last pose")

# ---------------- MANUS source + bridge ----------------
def manus_dongle_nodes():
    """usbfs device nodes for the plugged-in Manus dongles (VID 3325)."""
    nodes = []
    for base in glob.glob("/sys/bus/usb/devices/*"):
        try:
            if open(os.path.join(base, "idVendor")).read().strip() != MANUS_USB_VID:
                continue
            bus = open(os.path.join(base, "busnum")).read().strip()
            dev = open(os.path.join(base, "devnum")).read().strip()
            nodes.append("/dev/bus/usb/%03d/%03d" % (int(bus), int(dev)))
        except OSError:
            pass
    return nodes

def manus_fix_usb_perms(log):
    """chmod 666 the dongle nodes (root-only-write on replug). Uses the docker
    trick from the setup notes - sudo needs a password here."""
    nodes = manus_dongle_nodes()
    if not nodes:
        log("no Manus dongle found (VID %s) - plug it into the helmet" % MANUS_USB_VID)
        return False
    for n in nodes:
        if not os.access(n, os.W_OK):
            sh(["docker", "run", "--rm", "-v", "/dev/bus/usb:/dev/bus/usb",
                "busybox", "chmod", "666", n], 20)
    log("%d Manus dongle(s) present, permissions ok" % len(nodes))
    return True

def manus_ergo_hz():
    """Callback count from the ergo_stream heartbeat, or -1 if unknown."""
    try:
        txt = open(os.path.join(LOGDIR, "manus_ergo.log")).read()
    except OSError:
        return -1
    m = re.findall(r"callbacks so far: (\d+)", txt)
    return int(m[-1]) if m else -1

def start_manus_source(dev, log):
    """Gloves Start = CONNECT the gloves. The only licensed dongle owner is the
    Sharpa client, so this brings that up (headless) and waits until BOTH gloves
    stream. It deliberately does NOT run the old manus_skeleton_stream.out - that
    binary is unlicensed and would steal the dongles from the Sharpa client.
    After this, 'Retarget' starts the optimizer instantly (client already up)."""
    return ensure_sharpa_client(dev, log)

def stop_manus_source(dev, log):
    n = kill_local(MANUS_SRC_PROC, group=True)
    n += kill_local(MANUS_IMU_PROC, group=True)   # the live IMU wrist daemon owns the dongles too
    n += kill_local(SHARPA_CLIENT_PROC, group=True)
    log("stopped Manus glove source (%d process(es)) - gloves disconnected, "
        "dongles free. Retargeting (if running) is now starved of keypoints." % n)

VIZ_PANEL = os.path.expanduser("~/Desktop/Zhi/viz/viz_panel.py")
VIZ_PROC = r"viz_panel\.py"

def start_embedded_viz():
    """Spawn the MuJoCo rig mirror (viz_panel.py) embedded into the panel's
    right-side container frame; auto-run at panel startup. Read-only taps."""
    if local_procs(VIZ_PROC):
        return
    cmd = [VIZ_PANEL, "--size", "880x500"]
    try:
        cmd += ["--embed", "0x%x" % viz_holder.winfo_id()]
    except Exception:
        pass   # container not up yet -> standalone window fallback
    spawn(cmd, os.path.expanduser("~/Desktop/Zhi/viz/viz.log"))

def stop_viz():
    n = kill_local(VIZ_PROC, group=True)
    kill_local(r"hand_relay\.py", group=True)
    return n

def toggle_viz():
    """The '◇ Viz' button: restart/stop the embedded viz."""
    if not stop_viz():
        start_embedded_viz()


def sharpa_glove_live(side, within=4.0):
    """True if the Sharpa client published a `side` ('Left'/'Right') skeleton
    within the last `within` seconds. Uses the 'System time' epoch stamp on the
    client's per-frame log lines, so a stale log from an earlier run (or a glove
    that streamed once and went to sleep) does NOT count as live."""
    try:
        txt = open(os.path.join(LOGDIR, "sharpa_client.log")).read()
    except OSError:
        return False
    ts = re.findall(r"glove: %s is published\. - System time: ([0-9.]+)s" % side, txt)
    return bool(ts) and time.time() - float(ts[-1]) < within


def ensure_sharpa_client(dev, log, timeout=120):
    """Bring up the licensed Sharpa client (gloves -> ZMQ keypoints :2044) and
    wait until BOTH gloves stream real frames. Safe to call when the client is
    already running - then it only verifies liveness. Returns True when both
    gloves are live."""
    if not os.path.exists(SHARPA_CLIENT):
        log("Manus glove pipeline is NOT DEPLOYED on this machine - missing %s "
            "(the hermes /home/yam stack: ManusSDK client + sharpa-manus-sdk). "
            "The dongles themselves are plugged in and ready." % SHARPA_CLIENT)
        return False
    # unlicensed integrated apps steal the dongles from the client - clear them
    kill_local(MANUS_SRC_PROC, group=True)
    kill_local(MANUS_IMU_PROC, group=True)
    if not manus_fix_usb_perms(log):
        return False
    if not local_procs(SHARPA_CLIENT_PROC):
        log("starting Sharpa client (gloves -> keypoints) ...")
        spawn(["env", "SHARPA_HEADLESS=1", SHARPA_CLIENT],
              os.path.join(LOGDIR, "sharpa_client.log"), cwd=SHARPA_CLIENT_DIR)
    log("waiting for BOTH gloves to stream (power them on now if they aren't) ...")
    last_state, deadline = None, time.time() + timeout
    while True:
        if dev.cancel:
            log("cancelled while waiting for gloves (client left running)")
            return False
        live = {s: sharpa_glove_live(s) for s in ("Left", "Right")}
        if all(live.values()):
            log("BOTH gloves live - Sharpa client streaming keypoints (ZMQ :2044)")
            return True
        state = tuple(sorted(s for s, ok in live.items() if ok))
        if state != last_state:
            missing = [s for s, ok in live.items() if not ok]
            log("glove(s) not streaming yet: %s" % ", ".join(missing))
            last_state = state
        if time.time() > deadline:
            log("gave up after %ds - %s glove(s) never streamed. Client is left "
                "running: power the glove(s) on and press Start/Retarget again. "
                "(see sharpa_client.log)"
                % (timeout, ", ".join(s for s, ok in live.items() if not ok)))
            return False
        time.sleep(1)


def start_manus_bridge(dev, log):
    """Retarget = the OFFICIAL sharpa-manus-sdk optimizer. Runs the headless
    Sharpa client (gloves -> ZMQ keypoints :2044) and the retargeting demo
    (-wave -> hands over UDP). Optimization/keypoint-based; no manual sign/gain
    tuning. Our custom source/bridge/drivers are stopped first (dongle + hands).
    Order-independent: the optimizer is started only after BOTH gloves are
    verified live, and Retarget waits for gloves that are still powering on."""
    if local_procs(SHARPA_RETARGET_PROC):
        log("Sharpa retargeting already running")
        return True
    # stop the custom linear bridges (ensure_sharpa_client frees the dongles)
    kill_local(r"manus_sharpa_bridge", group=True)
    kill_local(r"wuji_sharpa_bridge", group=True)
    time.sleep(2)
    # 1) Sharpa client + BOTH gloves live (checked every time, also on retries
    # where the client is already up - Retarget never goes half-live)
    if not ensure_sharpa_client(dev, log):
        return False
    # 2) our hand drivers (they drive the hands + coexist with SharpaPilot)
    if not ensure_sharpa_driver(log, "left") or not ensure_sharpa_driver(log, "right"):
        return False
    # 3) retargeting optimizer in pure-ZMQ mode -> joints on :6668
    log("starting Sharpa retargeting optimizer ...")
    spawn([SHARPAMANUS_PY, "-u", SHARPA_RETARGET],
          os.path.join(LOGDIR, "sharpa_retarget.log"), cwd=SHARPA_RETARGET_DIR)
    time.sleep(6)
    if not local_procs(SHARPA_RETARGET_PROC):
        log("Sharpa retargeting died - see sharpa_retarget.log")
        return False
    # 4) bridge the optimizer's :6668 output to our drivers (59201/59202)
    log("starting ZMQ->driver bridge (:6668 -> 59201/59202) ...")
    spawn([SHARPAMANUS_PY, "-u", SHARPA_ZMQ_BRIDGE],
          os.path.join(LOGDIR, "sharpa_zmq_bridge.log"))
    time.sleep(2)
    if not local_procs(SHARPA_ZMQ_BRIDGE_PROC):
        log("ZMQ->driver bridge died - see sharpa_zmq_bridge.log")
        return False
    log("Sharpa OFFICIAL retargeting LIVE - gloves driving the hands (optimizer -> our drivers)")
    return True

def stop_manus_bridge(dev, log):
    n = kill_local(SHARPA_RETARGET_PROC, group=True)
    kill_local(SHARPA_ZMQ_BRIDGE_PROC, group=True)
    kill_local(SHARPA_WRIST_PROC, group=True)   # wrist adapter feeds off the client
    kill_local(SHARPA_CLIENT_PROC, group=True)
    log("stopped Sharpa retargeting (%d) + client + bridge - hands hold last pose" % n)

# ---------------- glove-driven ARM teleop (wrist -> IK -> follower) ----------
ARM_TELEOP_PROC = r"manus_arm_teleop\.py"

def arm_teleop_live():
    """Live wrist-teleop processes only (exclude transient --reset / --loose runs)."""
    return [(p, a) for p, a in local_procs(ARM_TELEOP_PROC)
            if "--reset" not in a and "--loose" not in a]

def restart_sharpa_hands(log):
    """Calibrate the Sharpa hands to the INITIAL (open) pose as part of a reset.
    A finger stream (retarget bridge) usually feeds the drivers continuously, so
    we PAUSE those feeders (SIGSTOP), command the init pose to each driver and
    hold it (the hand interpolates open and stays), then RESUME the feeders - so
    teleop always starts from a known pose instead of wherever the glove was."""
    drivers = [s for s in ("left", "right")
               if local_procs(r"sharpa_hand_driver.*" + re.escape(SHARPA_UDP[s]))]
    if not drivers:
        return
    # pause anything streaming finger targets into the drivers
    paused = []
    for pat in SHARPA_FEEDERS:
        for pid, _ in local_procs(pat):
            try:
                os.kill(pid, signal.SIGSTOP); paused.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
    # command the init pose repeatedly (~1.5s) so the hand reaches and holds it
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = json.dumps(SHARPA_INIT_POSE).encode()
    try:
        t_end = time.time() + SHARPA_INIT_HOLD_S
        while time.time() < t_end:
            for s in drivers:
                host, port = SHARPA_UDP[s].split(":")
                sock.sendto(payload, (host, int(port)))
            time.sleep(0.05)
    finally:
        sock.close()
        for pid in paused:                       # resume the feeders
            try:
                os.kill(pid, signal.SIGCONT)
            except (ProcessLookupError, PermissionError):
                pass
    log("sharpa hand(s) calibrated to initial open pose (%s)" % ", ".join(drivers))

def reset_arms_sync(log):
    """A full reset: re-home the Sharpa hands AND drive both arms to HOME (joints
    0), stiff. Waits until the arm moves finish."""
    restart_sharpa_hands(log)                  # reset the hand orientation too
    kill_local(ARM_TELEOP_PROC, group=True)   # nothing else should command while resetting
    time.sleep(0.3)
    for side in ("left", "right"):
        spawn([GCK_PY, "-u", MANUS_ARM_TELEOP, "--side", side, "--reset",
               "--server-host", YAMBOX_IP, "--server-port", str(SIDES[side]["port"])],
              os.path.join(LOGDIR, "arm_reset_%s.log" % side))
    for _ in range(60):   # wait up to 30 s for the reset procs to exit
        time.sleep(0.5)
        if not [1 for _, a in local_procs(ARM_TELEOP_PROC) if "--reset" in a]:
            break
    log("arms reset to HOME (stiff)")

def udp_alive(port, timeout=3.0):
    """True if a datagram arrives on 127.0.0.1:port within timeout. Only safe to
    call while nothing else holds the port (preflight, before arm teleop binds)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("127.0.0.1", port))
        s.settimeout(timeout)
        s.recvfrom(4096)
        return True
    except OSError:
        return False
    finally:
        s.close()

def leaders_live():
    """Sides whose bilateral leader (minimum_gello) process is running."""
    return [s for s in ("left", "right")
            if local_procs(r"python.*minimum_gello.*%s\b" % SIDES[s]["leader_can"])]

def ensure_vive_stream(log):
    """Make sure SteamVR + the tracker->UDP wrist stream are up and actually
    delivering wrist packets. Starts the stream if needed; leaves it running."""
    if not os.path.exists(VIVE_STREAM):
        log("vive wrist stream is NOT DEPLOYED on this machine - missing %s "
            "(the hermes /home/yam stack), so tracker teleop cannot start"
            % VIVE_STREAM)
        return False
    if not local_procs(STEAMVR_PROC):
        log("SteamVR is not running - launch it first (steam -applaunch 250820)")
        return False
    if not local_procs(VIVE_PROC):
        log("starting the Vive wrist stream ...")
        spawn([SYS_PY, "-u", VIVE_STREAM], os.path.join(LOGDIR, "vive_stream.log"))
        time.sleep(2)
        if not local_procs(VIVE_PROC):
            log("vive stream died right away - see %s/vive_stream.log" % LOGDIR)
            return False
    # only probe the port while no arm teleop holds it
    if not arm_teleop_live() and not udp_alive(VIVE_PORTS["left"], 3.0):
        log("no tracker packets on udp/%d - trackers tracking? (green in SteamVR; "
            "see vive_stream.log)" % VIVE_PORTS["left"])
        return False
    log("vive trackers streaming")
    return True

def teleop_preflight(log):
    """Tracker wrist teleop preflight: both followers up (yambox) and the Vive
    tracker stream delivering wrist poses."""
    snap = yambox_snapshot()
    if snap is None:
        log("yambox unreachable - Start the followers first"); return False
    for side, cfg in SIDES.items():
        if not (("--can-channel %s" % cfg["fol_can"]) in snap
                and re.search(r":%d\b" % cfg["port"], snap)):
            log("%s follower (:%d) is not up - Start it first" % (side, cfg["port"]))
            return False
    if not ensure_vive_stream(log):
        return False
    log("preflight ok - followers up, trackers streaming")
    return True

def teleop_launch(dev, log):
    """Start the per-side wrist-teleop processes (runs at the END of the
    countdown, so each side captures the tracker pose of THAT moment as its
    neutral). Arms track tracker position + orientation; the Sharpa hands stay
    on whatever MANUS finger pipeline is already running (separate tiles)."""
    kill_local(ARM_TELEOP_PROC, group=True)      # no stale teleop may fight us
    time.sleep(0.3)
    started = []
    # Tracker->port pairing verified LIVE 2026-08-25 (user observation): the
    # RIGHT tracker on port 11334 moved the arm the operator calls LEFT, so the
    # straight pairing below is correct: left tracker -> "left"/11334, right
    # tracker -> "right"/11333. (The viz notes' 11333-is-left claim does NOT
    # apply to this pairing - do not "fix" it back.) --frame world makes the EE
    # move/rotate the SAME world direction as the tracker (operator yaw
    # auto-calibrates at capture from the two-tracker baseline).
    tracker_for = {"left": "left", "right": "right"}   # port label -> tracker side
    for side in ("left", "right"):
        if dev.cancel:
            log("teleop start cancelled"); return False
        spawn([GCK_PY, "-u", MANUS_ARM_TELEOP, "--side", side,
               "--server-host", YAMBOX_IP, "--server-port", str(SIDES[side]["port"]),
               "--wrist-side", tracker_for[side],
               "--in-port", str(VIVE_PORTS[side]),
               "--pos-scale", str(VIVE_POS_SCALE),
               "--max-offset", str(VIVE_MAX_OFFSET),
               "--base-yaw-deg", str(VIVE_BASE_YAW[side]),
               "--frame", "world"]
              + (["--flip-z"] if VIVE_FLIP_Z[side] else [])
              + (["--flip-x"] if VIVE_FLIP_X[side] else [])
              + ["--extend-start", str(VIVE_EXTEND_START),
                 "--lift-start", str(VIVE_LIFT_START)],
              os.path.join(LOGDIR, "arm_teleop_%s.log" % side))
        started.append(side)
    time.sleep(4)   # let them connect + capture the neutral
    live = [s for s in started
            if [1 for _, a in arm_teleop_live() if re.search(r"--side\s+%s\b" % s, a)]]
    dead = [s for s in started if s not in live]
    for s in dead:
        log("%s arm teleop died - see arm_teleop_%s.log" % (s, s))
    if not live:
        return False
    log("TELEOP LIVE (%s) - arms tracking the wrist trackers (max offset %.2f m)"
        % (", ".join(live), VIVE_MAX_OFFSET))
    return len(live) == 2

def stop_teleop(dev, log):
    # Stop = stop the wrist-teleop processes only. The followers keep holding
    # their last pose (stiff, on the follower's own control). The vive stream
    # and the finger pipeline keep running - they are harmless without teleop.
    n = kill_local(ARM_TELEOP_PROC, group=True)
    log("teleop stopped (%d procs) - followers hold last pose" % n)

def on_mirror_home():
    """Capture the LEFT arm's current pose and save right = mirror(left) as the
    per-side reset homes, so both hands share an orientation after a reset."""
    if not messagebox.askyesno(
            "Mirror L->R home", "Pose the LEFT arm where you want it, then confirm.\n"
            "I'll capture the left pose and set the right arm's reset home to a "
            "MIRROR of it. Reset afterward to move the right there."):
        return
    def run():
        snap = yambox_snapshot() or ""
        for side, cfg in SIDES.items():
            if not (("--can-channel %s" % cfg["fol_can"]) in snap
                    and re.search(r":%d\b" % cfg["port"], snap)):
                dlog(None, "%s follower not up - Start it first" % side); return
        dlog(None, "capturing LEFT pose, mirroring to RIGHT home ...")
        cp = sh([GCK_PY, "-u", MANUS_ARM_TELEOP, "--side", "left", "--mirror-home",
                 "--server-host", YAMBOX_IP], 20)
        out = (cp.stderr + cp.stdout).strip().splitlines()
        line = [l for l in out if "mirror-home saved" in l]
        dlog(None, line[-1] if line else
             (out[-1] if out else "mirror-home failed - see log"))
    threading.Thread(target=run, daemon=True).start()

def on_shutdown_arms():
    """The ONLY path to loose: drop both arms to gravity-comp (backdrivable)."""
    if not messagebox.askyesno(
            "Shut down arms", "Make BOTH arms LOOSE (gravity-comp, backdrivable)?\n"
            "They will stop holding position - support them before they go slack."):
        return
    def run():
        kill_local(ARM_TELEOP_PROC, group=True)   # stop teleop/reset if any
        time.sleep(0.5)
        snap = yambox_snapshot() or ""
        for side, cfg in SIDES.items():
            if not (("--can-channel %s" % cfg["fol_can"]) in snap
                    and re.search(r":%d\b" % cfg["port"], snap)):
                dlog(None, "%s follower not up - cannot set it loose" % side); return
        dlog(None, "shutting down - arms going LOOSE (gravity-comp) ...")
        for side in ("left", "right"):
            spawn([GCK_PY, "-u", MANUS_ARM_TELEOP, "--side", side, "--loose",
                   "--server-host", YAMBOX_IP, "--server-port", str(SIDES[side]["port"])],
                  os.path.join(LOGDIR, "arm_loose_%s.log" % side))
        dlog(None, "arms LOOSE - move them by hand")
        st = devices.get(("teleop", "both"))
        if st:
            set_state(st, "OFF", "arms loose (shut down)")
    threading.Thread(target=run, daemon=True).start()

def on_reset_arms():
    """Move both yam arms to the home pose (all joints 0), stiff, smoothly."""
    if not messagebox.askyesno(
            "Reset arms", "Move BOTH yam arms to the HOME pose (all joints 0)?\n"
            "The arms will sweep there under power - clear the workspace and "
            "keep a hand on the e-stop."):
        return
    def run():
        if local_procs(ARM_TELEOP_PROC):     # don't let live teleop fight the reset
            dlog(None, "stopping arm teleop before reset ...")
            kill_local(ARM_TELEOP_PROC, group=True)
            time.sleep(1)
        snap = yambox_snapshot() or ""
        for side, cfg in SIDES.items():
            if not (("--can-channel %s" % cfg["fol_can"]) in snap
                    and re.search(r":%d\b" % cfg["port"], snap)):
                dlog(None, "%s follower not up - Start it before Reset" % side)
                return
        dlog(None, "resetting: re-homing hands + driving both arms to home ...")
        reset_arms_sync(lambda m: dlog(None, m))   # re-homes hands AND arms
    threading.Thread(target=run, daemon=True).start()

def start_record(dev, log):
    if local_procs(r"record_episode\.py"):
        log("already recording")
        return True
    log("starting episode recorder (cams + arm state + hand actions) ...")
    # sys.executable, not PY: the hermes i2rt venv doesn't exist here and the
    # panel's own venv (= the recorder's shebang) has portal+h5py+numpy
    spawn([sys.executable, RECORDER], os.path.join(LOGDIR, "record.log"))
    time.sleep(4)
    if not local_procs(r"record_episode\.py"):
        log("recorder died right away - see %s/record.log" % LOGDIR)
        return False
    try:
        st = json.load(open(REC_STATUS))
        log("REC episode %d -> %s" % (st["episode"], st["dir"]))
    except Exception:
        log("REC started")
    if not local_procs(r"sharpa_hand_driver.*--tee"):
        log("note: hand driver has no --tee - hand actions will be EMPTY; "
            "Quit+Start 'Sharpa hand' R once to fix")
    return True

def stop_record(dev, log):
    if not kill_local(r"record_episode\.py", signal.SIGINT):
        log("no recorder running")
        return
    log("finalizing the episode (closing videos, copying head cam) ...")
    for _ in range(25):
        time.sleep(1)
        if not local_procs(r"record_episode\.py"):
            break
    try:
        last = open(os.path.join(LOGDIR, "record.log")).read().strip().splitlines()[-1]
    except Exception:
        last = ""
    log(last or "episode saved")

# ---------------- button handlers ----------------
def on_start(dev):
    if dev.busy or dev.state == "START":
        return
    dev.busy, dev.cancel = True, False
    set_state(dev, "START", "starting ...")
    def run():
        ok = False
        try:
            ok = dev.start_fn(dev, lambda m: dlog(dev, m))
        except Exception as e:
            dlog(dev, "ERROR: %s: %s" % (type(e).__name__, e))
        if dev.cancel:
            set_state(dev, "OFF", "stopped")
        else:
            set_state(dev, "LIVE" if ok else "DEAD",
                      "up" if ok else "start failed - see log")
        dev.busy = False
    threading.Thread(target=run, daemon=True).start()

def on_quit(dev):
    if dev.busy and dev.state != "START":
        return
    dev.cancel = True
    def run():
        dev.busy = True
        try:
            dev.stop_fn(dev, lambda m: dlog(dev, m))
        except Exception as e:
            dlog(dev, "ERROR: %s: %s" % (type(e).__name__, e))
        set_state(dev, "OFF", "stopped")
        dev.busy = False
    threading.Thread(target=run, daemon=True).start()

def on_teleop_start(dev):
    """Start tracker wrist teleop: preflight (followers + trackers) -> reset the
    arms to HOME -> 5 s countdown (pose your wrists where teleop should start)
    -> launch, which captures that pose as the neutral and goes live."""
    if dev.busy or dev.state == "START":
        return
    dev.busy, dev.cancel = True, False
    set_state(dev, "START", "preflight ...")
    def run():
        ok = False
        try:
            if not teleop_preflight(lambda m: dlog(dev, m)):
                set_state(dev, "DEAD", "preflight failed - see log"); return
            if dev.cancel:
                set_state(dev, "OFF", "cancelled"); return
            dlog(dev, "resetting arms + hands to home before teleop ...")
            reset_arms_sync(lambda m: dlog(dev, m))
            if dev.cancel:
                set_state(dev, "OFF", "cancelled"); return
            ok = True
        finally:
            if not ok:
                dev.busy = False
        # countdown runs on the UI thread; it launches teleop at 0 and
        # clears dev.busy itself (see _teleop_countdown)
        ui_q.put(("teleop_countdown", dev, TELEOP_COUNTDOWN))
    threading.Thread(target=run, daemon=True).start()

def on_teleop_stop(dev):
    dev.cancel = True
    def run():
        dev.busy = True
        try:
            stop_teleop(dev, lambda m: dlog(dev, m))
        except Exception as e:
            dlog(dev, "ERROR: %s" % e)
        set_state(dev, "OFF", "stopped")
        dev.busy = False
    threading.Thread(target=run, daemon=True).start()

def on_reset_can():
    live = [d for (k, _), d in devices.items() if k == "yambox" and d.state == "LIVE"]
    if live and not messagebox.askyesno(
            "Reset CAN", "Followers are LIVE and the reset cycles ALL buses - "
            "they will die and need a restart. Reset anyway?"):
        return
    def run():
        dlog(None, "resetting yambox CAN buses (all of them) ...")
        ssh("echo %s | sudo -S -p '' sh ~/i2rt/scripts/reset_all_can.sh >/dev/null 2>&1"
            % YAMBOX_PW, 60)
        time.sleep(4)
        dlog(None, "CAN reset done - Start the followers again")
    threading.Thread(target=run, daemon=True).start()

def on_quit_all():
    if not messagebox.askyesno("Quit ALL", "Stop every device on the rig?"):
        return
    def run():
        dlog(None, "quitting everything ...")
        order = [("record", "both"),          # save the episode before the rig goes down
                 ("teleop", "both"),          # stop glove arm teleop before the followers go down
                 ("leader", "left"), ("leader", "right"),
                 ("manusbridge", "both"),     # retargets before the drivers, so their SIGINT still homes the hands
                 ("wujiretarget", "both"),
                 ("sharpa", "right"), ("sharpa", "left"),
                 ("manus", "both"),           # free the dongles after its retarget is down
                 ("wuji", "both"),
                 ("yambox", "left"), ("yambox", "right")]
        for key in order:
            dev = devices[key]
            dev.cancel = True
            try:
                dev.stop_fn(dev, lambda m, d=dev: dlog(d, m))
            except Exception as e:
                dlog(dev, "ERROR: %s" % e)
            set_state(dev, "OFF", "stopped")
        dlog(None, "rig is down")
    threading.Thread(target=run, daemon=True).start()

# ---------------- health poller ----------------
RUNNING = True
io_prev = {}

def promote(dev, alive, dead_msg="went down - press Start to restart"):
    if dev.busy or dev.state == "START" or alive is None:
        return
    if alive:
        if dev.state != "LIVE":
            set_state(dev, "LIVE", "up")
    elif dev.state in ("LIVE", "STUCK"):
        set_state(dev, "DEAD", dead_msg)
        dlog(dev, dead_msg)

def poller():
    while RUNNING:
        snap = yambox_snapshot()
        for side, cfg in SIDES.items():
            dev = devices[("yambox", side)]
            if snap is None:
                promote(dev, False, "yambox unreachable")
            else:
                alive = ("--can-channel %s" % cfg["fol_can"] in snap
                         and bool(re.search(r":%d\b" % cfg["port"], snap)))
                promote(dev, alive)

            ldev = devices[("leader", side)]
            procs = local_procs(r"python.*minimum_gello.*%s\b" % cfg["leader_can"])
            if not procs:
                promote(ldev, False)
            elif not (ldev.busy or ldev.state == "START"):
                logf = os.path.join(LOGDIR, "leader_%s.log" % side)
                try:
                    c = open(logf).read().count("web-port io")
                except OSError:
                    c = 0
                prev = io_prev.get(side, -1)
                io_prev[side] = c
                if prev >= 0 and c <= prev:
                    if ldev.state != "STUCK":
                        set_state(ldev, "STUCK", "control loop stalled (io not advancing)")
                elif ldev.state != "LIVE":
                    set_state(ldev, "LIVE", "up")

        promote(devices[("wuji", "both")],
                bool(local_procs(WUJI_STUDIO_PAT)) if devices[("wuji", "both")].state
                in ("LIVE", "STUCK", "DEAD") else None,
                "wuji studio gone - press Start")
        promote(devices[("wujiretarget", "both")],
                bridge_running("left") or bridge_running("right"),
                "wuji bridges gone - press Retarget")
        # Manus gloves tile = the licensed Sharpa client (the actual glove
        # owner), with per-glove liveness from its log timestamps
        msrc = devices[("manus", "both")]
        src_alive = bool(local_procs(SHARPA_CLIENT_PROC))
        promote(msrc, src_alive, "glove client stopped - press Start")
        if src_alive and msrc.state == "LIVE":
            live = [s for s in ("Left", "Right") if sharpa_glove_live(s)]
            ui_q.put(("msg", msrc, "client up · gloves live: %s"
                      % (", ".join(live) if live else "NONE - power the gloves on")))
        promote(devices[("manusbridge", "both")], bool(local_procs(SHARPA_RETARGET_PROC)),
                "Sharpa retargeting stopped - press Retarget to relaunch")
        promote(devices[("teleop", "both")], len(arm_teleop_live()) == 2,
                "wrist teleop stopped - press Start Teleop")
        rdev = devices[("record", "both")]
        rec = bool(local_procs(r"record_episode\.py"))
        promote(rdev, rec, "recorder stopped")
        if rec and rdev.state == "LIVE":
            try:
                st = json.load(open(REC_STATUS))
                ui_q.put(("msg", rdev, "REC episode %d · %d s · %d frames · %d hand actions"
                          % (st["episode"], int(st["elapsed"]),
                             st["frames"], st["hand_msgs"])))
            except Exception:
                pass
        for side in SIDES:
            promote(devices[("sharpa", side)],
                    bool(local_procs(r"sharpa_hand_driver.*" + re.escape(SHARPA_UDP[side]))))
        time.sleep(6)

# ---------------- UI ----------------
BG, TILE_BG, FG = "#1e1e1e", "#2a2a2a", "#e8e8e8"
ROWS = [("Follower", "yambox", start_follower, stop_follower),
        ("Leader", "leader", start_leader, stop_leader),
        ("Sharpa", "sharpa", None, None)]   # per-side: Left hand / Right hand

root = tk.Tk()
root.title("Zhi teleop panel")
root.configure(bg=BG)

TILE_BORDER = "#3d3d3d"
BTN_BG, BTN_HOVER = "#3a3a3a", "#4d4d4d"
START_BG, START_HOVER = "#245c40", "#2e7350"
QUIT_BG, QUIT_HOVER = "#5c2b27", "#733630"

def mk_btn(parent, text, cmd, bg=BTN_BG, hover=BTN_HOVER):
    b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=FG,
                  activebackground=hover, activeforeground=FG,
                  bd=0, relief="flat", highlightthickness=0,
                  padx=16, pady=4, cursor="hand2", font=("TkDefaultFont", 9))
    b.bind("<Enter>", lambda e: b.configure(bg=hover))
    b.bind("<Leave>", lambda e: b.configure(bg=bg))
    return b

def build_tile(parent, dev, title, start_label="Start"):
    f = tk.Frame(parent, bg=TILE_BG, bd=0, padx=12, pady=10,
                 highlightbackground=TILE_BORDER, highlightcolor=TILE_BORDER,
                 highlightthickness=1)
    top = tk.Frame(f, bg=TILE_BG)
    top.pack(fill="x")
    dev.dot = tk.Canvas(top, width=16, height=16, bg=TILE_BG, highlightthickness=0)
    dev.dot_id = dev.dot.create_oval(3, 3, 13, 13, fill=COLORS["OFF"], outline="")
    dev.dot.pack(side="left")
    tk.Label(top, text=title, bg=TILE_BG, fg=FG,
             font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(8, 0))
    dev.state_lbl = tk.Label(top, text="off", bg=TILE_BG, fg=COLORS["OFF"],
                             font=("TkDefaultFont", 10, "bold"))
    dev.state_lbl.pack(side="right")
    dev.msg_lbl = tk.Label(f, text="not started", bg=TILE_BG, fg="#8f8f8f",
                           anchor="w", justify="left", wraplength=270,
                           font=("TkDefaultFont", 8))
    dev.msg_lbl.pack(fill="x", pady=(5, 8))
    btns = tk.Frame(f, bg=TILE_BG)
    btns.pack(fill="x")
    mk_btn(btns, start_label, lambda d=dev: on_start(d),
           START_BG, START_HOVER).pack(side="left")
    mk_btn(btns, "Quit", lambda d=dev: on_quit(d),
           QUIT_BG, QUIT_HOVER).pack(side="right")
    return f

# one window: tiles on the left, the embedded MuJoCo viz (viz_panel.py
# --embed, XEmbed) on the right - they open and close together
content = tk.Frame(root, bg=BG)
content.pack(fill="both", expand=True)
grid = tk.Frame(content, bg=BG, padx=14, pady=12)
grid.pack(side="left", fill="both", expand=True)
viz_holder = tk.Frame(content, bg=BG, container=True, width=900, height=900)
viz_holder.pack(side="right", fill="y")
viz_holder.pack_propagate(False)
tk.Label(grid, text="", bg=BG).grid(row=0, column=0)
for c, side in enumerate(("left", "right"), start=1):
    tk.Label(grid, text=side.upper(), bg=BG, fg="#6f9bd1",
             font=("TkDefaultFont", 11, "bold")).grid(row=0, column=c, pady=(0, 8))

for r, (title, kind, start_fn, stop_fn) in enumerate(ROWS, start=1):
    tk.Label(grid, text=title, bg=BG, fg="#b0b0b0", anchor="e",
             font=("TkDefaultFont", 10)).grid(row=r, column=0, sticky="e", padx=(0, 12))
    for c, side in enumerate(("left", "right"), start=1):
        if kind == "sharpa":
            sfn = start_sharpa_left if side == "left" else start_sharpa_right
            qfn = stop_sharpa_left if side == "left" else stop_sharpa_right
            sub = "Left hand" if side == "left" else "Right hand"
        else:
            sfn, qfn = start_fn, stop_fn
            sub = ("%s : %d" % (SIDES[side]["fol_can"], SIDES[side]["port"])
                   if kind == "yambox" else SIDES[side]["leader_can"])
        dev = Device(kind, side, sfn, qfn)
        devices[(kind, side)] = dev
        build_tile(grid, dev, sub).grid(row=r, column=c, sticky="nsew", padx=5, pady=5)

# A glove system = one Start (connect the gloves) + one Retarget (bridge both
# gloves to the two Sharpa hands). Same shape for Wuji and Manus.
def glove_row(row, name, start_dev, retarget_dev):
    tk.Label(grid, text=name, bg=BG, fg="#b0b0b0", anchor="e",
             font=("TkDefaultFont", 10)).grid(row=row, column=0, sticky="e", padx=(0, 12))
    build_tile(grid, start_dev, "Gloves", start_label="Start")\
        .grid(row=row, column=1, sticky="nsew", padx=5, pady=5)
    build_tile(grid, retarget_dev, "To hands", start_label="Retarget")\
        .grid(row=row, column=2, sticky="nsew", padx=5, pady=5)

_wj = len(ROWS) + 1
wuji_dev = Device("wuji", "both", start_wuji, stop_wuji)
wuji_rt = Device("wujiretarget", "both", start_wuji_retarget, stop_wuji_retarget)
devices[("wuji", "both")] = wuji_dev
devices[("wujiretarget", "both")] = wuji_rt
glove_row(_wj, "Wuji", wuji_dev, wuji_rt)

_mn = _wj + 1
manus_src = Device("manus", "both", start_manus_source, stop_manus_source)
manus_br = Device("manusbridge", "both", start_manus_bridge, stop_manus_bridge)
devices[("manus", "both")] = manus_src
devices[("manusbridge", "both")] = manus_br
glove_row(_mn, "Manus", manus_src, manus_br)

# full-width episode recorder tile: Start = new take, Quit = stop & save.
# Custom build: a big always-visible counter that pulses red while recording
# and turns green on save, so takes can't silently run or silently die.
_rr = _mn + 1
tk.Label(grid, text="Record", bg=BG, fg="#b0b0b0", anchor="e",
         font=("TkDefaultFont", 10)).grid(row=_rr, column=0, sticky="e", padx=(0, 12))
record_dev = Device("record", "both", start_record, stop_record)
devices[("record", "both")] = record_dev

REC_RED, REC_DIMRED, REC_GRN = "#ff5f52", "#8a3a34", "#5fbf87"

def build_record_tile(parent, dev):
    f = tk.Frame(parent, bg=TILE_BG, bd=0, padx=12, pady=10,
                 highlightbackground=TILE_BORDER, highlightcolor=TILE_BORDER,
                 highlightthickness=1)
    top = tk.Frame(f, bg=TILE_BG)
    top.pack(fill="x")
    dev.dot = tk.Canvas(top, width=16, height=16, bg=TILE_BG, highlightthickness=0)
    dev.dot_id = dev.dot.create_oval(3, 3, 13, 13, fill=COLORS["OFF"], outline="")
    dev.dot.pack(side="left")
    tk.Label(top, text="episode · cams + arm state + hand action → h5/mkv",
             bg=TILE_BG, fg=FG,
             font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(8, 0))
    dev.state_lbl = tk.Label(top, text="off", bg=TILE_BG, fg=COLORS["OFF"],
                             font=("TkDefaultFont", 10, "bold"))
    dev.state_lbl.pack(side="right")
    dev.big_lbl = tk.Label(f, text="—", bg=TILE_BG, fg="#6a6a6a", anchor="w",
                           font=("TkFixedFont", 18, "bold"))
    dev.big_lbl.pack(fill="x", pady=(6, 2))
    dev.msg_lbl = tk.Label(f, text="not recording", bg=TILE_BG, fg="#8f8f8f",
                           anchor="w", justify="left", wraplength=560,
                           font=("TkDefaultFont", 8))
    dev.msg_lbl.pack(fill="x", pady=(0, 8))
    btns = tk.Frame(f, bg=TILE_BG)
    btns.pack(fill="x")
    mk_btn(btns, "●  Record", lambda d=dev: on_start(d),
           "#7a2622", "#96322c").pack(side="left")
    mk_btn(btns, "■  Stop & save", lambda d=dev: on_quit(d)).pack(side="right")
    return f

build_record_tile(grid, record_dev)\
    .grid(row=_rr, column=1, columnspan=2, sticky="nsew", padx=5, pady=5)

# full-width glove wrist-teleop tile: Start Teleop -> countdown -> both arms
# track the glove wrists (IK) while the hands track the fingers.
_tr = _rr + 1
tk.Label(grid, text="Teleop", bg=BG, fg="#b0b0b0", anchor="e",
         font=("TkDefaultFont", 10)).grid(row=_tr, column=0, sticky="e", padx=(0, 12))
teleop_dev = Device("teleop", "both", None, stop_teleop)   # start via on_teleop_start (countdown)
devices[("teleop", "both")] = teleop_dev

def build_teleop_tile(parent, dev):
    f = tk.Frame(parent, bg=TILE_BG, bd=0, padx=12, pady=10,
                 highlightbackground=TILE_BORDER, highlightcolor=TILE_BORDER,
                 highlightthickness=1)
    top = tk.Frame(f, bg=TILE_BG); top.pack(fill="x")
    dev.dot = tk.Canvas(top, width=16, height=16, bg=TILE_BG, highlightthickness=0)
    dev.dot_id = dev.dot.create_oval(3, 3, 13, 13, fill=COLORS["OFF"], outline="")
    dev.dot.pack(side="left")
    tk.Label(top, text="tracker wrist → IK → both arms  ·  fingers → hands (manus)",
             bg=TILE_BG, fg=FG, font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(8, 0))
    dev.state_lbl = tk.Label(top, text="off", bg=TILE_BG, fg=COLORS["OFF"],
                             font=("TkDefaultFont", 10, "bold"))
    dev.state_lbl.pack(side="right")
    dev.big_lbl = tk.Label(f, text="—", bg=TILE_BG, fg="#6a6a6a", anchor="w",
                           font=("TkFixedFont", 18, "bold"))
    dev.big_lbl.pack(fill="x", pady=(6, 2))
    dev.msg_lbl = tk.Label(f, text="Start = reset to home → countdown → teleop (STIFF). "
                           "Stop = reset to home (stays STIFF). Shut down = arms go LOOSE.",
                           bg=TILE_BG, fg="#8f8f8f", anchor="w", justify="left",
                           wraplength=560, font=("TkDefaultFont", 8))
    dev.msg_lbl.pack(fill="x", pady=(0, 8))
    btns = tk.Frame(f, bg=TILE_BG); btns.pack(fill="x")
    mk_btn(btns, "▶  Start Teleop", lambda d=dev: on_teleop_start(d),
           START_BG, START_HOVER).pack(side="left")
    mk_btn(btns, "⟲  Reset arms", on_reset_arms).pack(side="left", padx=(8, 0))
    mk_btn(btns, "◇  Viz", toggle_viz).pack(side="left", padx=(8, 0))
    mk_btn(btns, "⏻  Shut down", on_shutdown_arms,
           QUIT_BG, QUIT_HOVER).pack(side="right")
    mk_btn(btns, "■  Stop Teleop", lambda d=dev: on_teleop_stop(d)).pack(side="right", padx=(0, 8))
    return f

build_teleop_tile(grid, teleop_dev)\
    .grid(row=_tr, column=1, columnspan=2, sticky="nsew", padx=5, pady=5)

def _teleop_countdown(dev, remaining):
    if dev.cancel:
        dev.big_lbl.config(text="cancelled", fg="#8f8f8f")
        set_state(dev, "OFF", "cancelled"); dev.busy = False
        return
    if remaining > 0:
        dev.big_lbl.config(text="TELEOP IN  %d" % remaining, fg=REC_RED)
        dev.msg_lbl.config(text="arms reset to home — now hold your wrists (trackers) where "
                                "teleop should start; that pose is captured as neutral at 0")
        root.after(1000, lambda: _teleop_countdown(dev, remaining - 1))
    else:
        dev.big_lbl.config(text="● TELEOP LIVE", fg=REC_GRN)
        def run():
            ok = teleop_launch(dev, lambda m: dlog(dev, m))
            set_state(dev, "LIVE" if ok else "DEAD",
                      "arms tracking wrist trackers" if ok else "launch failed - see log")
            dev.busy = False
        threading.Thread(target=run, daemon=True).start()

_rec_tick = {"last": None, "pulse": False}
def record_tick():
    """0.5 s UI heartbeat for the recorder: live counter (starts at 0:01),
    pulsing REC dot, green 'saved' banner when the take finishes."""
    try:
        if os.path.exists(REC_STATUS):
            st = json.load(open(REC_STATUS))
            _rec_tick["last"] = st
            _rec_tick["pulse"] = not _rec_tick["pulse"]
            secs = max(1, int(st.get("elapsed", 0)))   # counter starts at 1 s
            dot = "●" if _rec_tick["pulse"] else "○"
            record_dev.big_lbl.config(
                text="%s REC  %d:%02d   episode %d" % (dot, secs // 60, secs % 60,
                                                       st.get("episode", 0)),
                fg=REC_RED if _rec_tick["pulse"] else REC_DIMRED)
        elif _rec_tick["last"] is not None:
            st, _rec_tick["last"] = _rec_tick["last"], None
            secs = max(1, int(st.get("elapsed", 0)))
            record_dev.big_lbl.config(
                text="✔ episode %d saved   %d:%02d" % (st.get("episode", 0),
                                                       secs // 60, secs % 60),
                fg=REC_GRN)
    except Exception:
        pass
    root.after(500, record_tick)
record_tick()
grid.columnconfigure(1, weight=1)
grid.columnconfigure(2, weight=1)

bar = tk.Frame(root, bg=BG, padx=14, pady=4)
bar.pack(fill="x")
mk_btn(bar, "Reset CAN (yambox)", on_reset_can).pack(side="left")
mk_btn(bar, "⇋ Mirror L→R home", on_mirror_home).pack(side="left", padx=(8, 0))
mk_btn(bar, "Quit ALL", on_quit_all, QUIT_BG, QUIT_HOVER).pack(side="right")

logbox = scrolledtext.ScrolledText(root, height=12, bg="#141414", fg="#c8c8c8",
                                   font=("TkFixedFont", 9), state="disabled",
                                   bd=0, relief="flat", highlightthickness=1,
                                   highlightbackground=TILE_BORDER,
                                   insertbackground="#c8c8c8")
logbox.pack(fill="both", expand=True, padx=14, pady=(6, 14))
logbox.tag_configure("err", foreground="#ff6b5e")
logbox.tag_configure("ok", foreground="#5fbf87")
logbox.tag_configure("dim", foreground="#7a7a7a")

def pump():
    try:
        while True:
            item = ui_q.get_nowait()
            if item[0] == "log":
                line = item[1]
                logbox.configure(state="normal")
                m = re.match(r"(\[[^\]]+\] \[[^\]]+\] )(.*)", line, re.S)
                pre, body = (m.group(1), m.group(2)) if m else ("", line)
                if re.search(r"ERROR|WARNING|failed|not responding|unreachable|gone|died|DEAD|never", body):
                    tag = ("err",)
                elif re.search(r"connected|listening on|driver up|\bUP\b|is driving", body):
                    tag = ("ok",)
                else:
                    tag = ()
                if pre:
                    logbox.insert("end", pre, ("dim",))
                logbox.insert("end", body + "\n", tag)
                logbox.see("end")
                logbox.configure(state="disabled")
            elif item[0] == "msg":
                _, dev, msg = item
                dev.msg_lbl.configure(text=msg)
            elif item[0] == "state":
                _, dev, state, msg = item
                dev.dot.itemconfigure(dev.dot_id, fill=COLORS[state])
                dev.state_lbl.configure(text=STATE_TXT[state], fg=COLORS[state])
                if msg:
                    dev.msg_lbl.configure(text=msg)
            elif item[0] == "teleop_countdown":
                _, dev, secs = item
                _teleop_countdown(dev, secs)   # UI-thread countdown then launch
    except queue.Empty:
        pass
    root.after(200, pump)

def on_close():
    global RUNNING
    if messagebox.askokcancel("Exit panel",
                              "Close the panel? Running devices KEEP running "
                              "(use Quit ALL first to stop the rig)."):
        RUNNING = False
        stop_viz()   # the embedded viz lives and dies with the panel
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.after(1200, start_embedded_viz)   # the viz opens with the panel
dlog(None, "panel up - suggested order: yambox -> sharpa / wuji glove -> teacher arm")
dlog(None, "already-running devices will light up green within ~6 s")
threading.Thread(target=poller, daemon=True).start()
pump()

# PANEL_AUTOSTART: press Start on each listed device in order, waiting for
# the previous one to leave STARTING before moving on.
def autostart(seq):
    seq = [s.strip() for s in seq if s.strip()]
    if not seq:
        dlog(None, "autostart done")
        return
    try:
        kind, side = seq[0].split(":")
        dev = devices[(kind, side)]
    except (ValueError, KeyError):
        dlog(None, "autostart: bad entry %r (want kind:side, e.g. yambox:left)" % seq[0])
        root.after(500, lambda: autostart(seq[1:]))
        return
    dlog(None, "autostart: %s" % seq[0])
    on_start(dev)
    def waiter():
        if dev.state == "START":
            root.after(1000, waiter)
        else:
            root.after(500, lambda: autostart(seq[1:]))
    root.after(1000, waiter)

if os.environ.get("PANEL_AUTOSTART"):
    root.after(2000, lambda: autostart(os.environ["PANEL_AUTOSTART"].split(",")))

# --selftest: auto-press buttons through the failure paths, dump the log
# and states, screenshot to $SELFTEST_PNG, then exit. WARNING: on a machine
# that can reach the rig this presses REAL Start buttons (yambox left!) -
# it is meant for headless debugging away from the robots. On hermes use
# --check instead.
if "--selftest" in sys.argv:
    def _st_finish():
        png = os.environ.get("SELFTEST_PNG")
        if png and shutil.which("import"):
            subprocess.run(["import", "-window", "root", png], timeout=15)
        print("---- final states ----")
        for (k, s), d in sorted(devices.items()):
            print("%-8s %-6s %-6s | %s" % (k, s, d.state, d.msg_lbl.cget("text")))
        print("---- log pane ----")
        print(logbox.get("1.0", "end").rstrip())
        RUNNING = False  # noqa: F841 - poller is a daemon, root.destroy ends us
        root.destroy()
    def _st_3():
        dlog(None, "SELFTEST: Start sharpa L, then Quit 4 s in (cancel path)")
        on_start(devices[("sharpa", "left")])
        root.after(4000, lambda: on_quit(devices[("sharpa", "left")]))
        root.after(10000, _st_finish)
    def _st_2():
        dlog(None, "SELFTEST: Start wuji (expect studio-not-found -> DEAD)")
        on_start(devices[("wuji", "both")])
        dlog(None, "SELFTEST: Start teacher L (expect follower-port check fail)")
        on_start(devices[("leader", "left")])
        root.after(12000, _st_3)
    def _st_1():
        dlog(None, "SELFTEST: Start yambox L (expect unreachable -> DEAD)")
        on_start(devices[("yambox", "left")])
        root.after(14000, _st_2)
    root.after(1500, _st_1)

root.mainloop()
