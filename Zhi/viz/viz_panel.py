#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Real-time MuJoCo visualization of the YAM + Sharpa teleop rig.

A second window alongside teleop_panel.py: a MuJoCo scene of the two YAM
follower arms (official i2rt meshes) with a Sharpa Wave hand (official Sharpa
meshes) attached at each wrist, mirroring the REAL rig live, plus a proprio
readout (arm joint angles + per-finger hand angles) below the 3D view.

Data sources (read-only taps, nothing on the rig is disturbed):
    arms  - portal get_joint_pos on the yambox followers
            (left :11334, right :11333), same call the recorder uses
    hands - the retargeting optimizer's HandAction on ZMQ :6668, relayed to
            udp/59260 by hand_relay.py (spawned automatically in the
            sharpamanus env, which owns pyzmq+protobuf)

Run:  ./viz_panel.py            # the live window
      ./viz_panel.py --shot f.png   # headless one-frame self-test

Keys: arrows orbit the camera, +/- zoom.
"""
import argparse, json, math, os, socket, subprocess, sys, threading, time

os.environ.setdefault("MUJOCO_GL", "glfw")  # egl broken on this box (nvidia 580); glfw renders on the 3090
import mujoco
import numpy as np
import PIL.Image

# ---------------- scene: Jeffrey's calibrated bench, verbatim ----------------
# The WHOLE environment (table, lighting, white skybox, black YAM arms, sharpa
# hands, home keyframe) is bench_scene.xml from
# github.com/JeffreyWang060303/egodata_preprocessing_pipeline sim_setup/,
# copied to ./bench/ - a calibrated digital twin of THIS rig (see rig.json).
# In that scene each hand's ORIENTATION is carried by six wrist DOFs
# (pos_x..rot_z, intrinsic XYZ, rig frame), welded position-only to the arm's
# gripper flange. We drive the arm joints from the live followers and make the
# wrist ride the flange rigidly through the scene's own home-pose mount.
BENCH_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "bench", "bench_scene.xml")
# bench "robot_left"/"robot_right" are named in the rig frame (+x toward its
# "right" base), which is MIRRORED vs the operator's sense - the physical left
# arm is the bench's robot_right (verified live: real left arm moved the sim
# right arm). Keys: "L"/"R" = PHYSICAL left/right throughout this file.
SIDE_NAME = {"L": "right", "R": "left"}
from scipy.spatial.transform import Rotation as SciRot

# ---------------- live data ----------------
YAMBOX_IP = os.environ.get("YAMBOX_IP", "192.168.1.9")
# NOTE: start_teleop.sh / record_episode.py label can1:11334 "left" and
# can0:11333 "right", but on the PHYSICAL rig it is the other way around
# (verified live 2026-08-24: moving the real left arm changes 11333).
ARM_PORTS = {"L": 11333, "R": 11334}
HAND_UDP = 59260        # commanded targets (retargeting optimizer via relay)
HAND_STATE_UDP = 59261  # TRUE hand joint state (hand_state_reader.py)
# the relay needs protobuf 3.12 (the SDK's generated sharpa_hand_pb2) - that
# lives in the sharpamanus venv, not the panel venv (2026-08-27)
RELAY_PY = "/home/zhicao/Desktop/Zhi/Zhi/sharpamanus-venv/bin/python"
RELAY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_relay.py")
STATE_READER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "hand_state_reader.py")
ARM_READER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_reader.py")
CAM_PREVIEW = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cam_preview.py")
ARM_UDP = 59262         # follower joint state (arm_reader.py)
STALE = 1.5   # s without data -> shown as stale

ORDER22 = [
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
]
FINGERS = [("thumb", 0, 5), ("index", 5, 9), ("middle", 9, 13),
           ("ring", 13, 17), ("pinky", 17, 22)]

# What the REAL hand executes: sharpa_hand_driver.py clamps every commanded
# joint to these conservative limits (degrees, Sharpa 22-joint order) before
# streaming to the hand. The sim applies the same clamp - intersected with the
# official Sharpa model's joint ranges - so the sim pose matches the real hand
# instead of showing the optimizer's raw (sometimes out-of-range) targets.
# Source: /home/yam/sharpa-venv/sharpa_hand_driver.py LIMITS_DEG.
DRIVER_LIMITS_DEG = [
    (0, 60), (-15, 15), (0, 60), (-15, 15), (0, 80),
    (0, 100), (-20, 20), (0, 110), (0, 90),
    (0, 100), (-20, 20), (0, 110), (0, 90),
    (0, 100), (-20, 20), (0, 110), (0, 90),
    (0, 30), (0, 100), (-20, 20), (0, 110), (0, 90),
]


def build_scene():
    model = mujoco.MjModel.from_xml_path(BENCH_XML)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, 1280)
    model.vis.global_.offheight = max(model.vis.global_.offheight, 720)
    # FK at the scene's own calibrated "home" keyframe gives the exact rigid
    # flange->hand mount for each side
    d0 = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d0, 0)
    mujoco.mj_forward(model, d0)
    addr = {}
    for side in ("L", "R"):
        s = SIDE_NAME[side]
        hadr, hlim = [], []
        for k, n in enumerate(ORDER22):
            j = model.joint(f"robot_{s}_{s}_{n}")
            hadr.append(j.qposadr[0])
            dlo, dhi = (math.radians(x) for x in DRIVER_LIMITS_DEG[k])
            mlo, mhi = (j.range if j.range[0] < j.range[1] else (-9.9, 9.9))
            hlim.append((max(dlo, float(mlo)), min(dhi, float(mhi))))
        grip = model.body(f"robot_{s}_arm_gripper").id
        hb = model.body(f"robot_{s}_{s}_hand_C_MC").id
        Rg = d0.xmat[grip].reshape(3, 3).copy()
        pg = d0.xpos[grip].copy()
        Rh = d0.xmat[hb].reshape(3, 3).copy()
        ph = d0.xpos[hb].copy()
        addr[side] = {
            "arm": [model.joint(f"robot_{s}_arm_dof_joint{i+1}").qposadr[0]
                    for i in range(6)],
            "hand": hadr,
            "hand_lim": hlim,
            "wrist": [model.joint(f"robot_{s}_{s}_{n}").qposadr[0]
                      for n in ("pos_x", "pos_y", "pos_z",
                                "rot_x", "rot_y", "rot_z")],
            "grip": grip,
            "R_mount": Rg.T @ Rh,
            "p_mount": Rg.T @ (ph - pg),
        }
    return model, addr


class ArmView:
    """Latest state of one arm, filled by ArmListener from arm_reader.py's
    UDP stream. (The portal pollers live in that separate process because
    portal os._exit(1)s its whole process - children included - when the
    yambox becomes unreachable; it killed the embedded viz live 2026-08-25.)"""

    def __init__(self):
        self.q, self.t, self._mov = None, 0.0, 0.0

    def moving(self):
        return self._mov


class ArmListener(threading.Thread):
    """arm_reader.py UDP JSON (:59262) -> ArmView per physical side."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.views = {"L": ArmView(), "R": ArmView()}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
                m = json.loads(data.decode())
                for s in ("L", "R"):
                    if s in m and len(m[s].get("q", [])) == 6:
                        v = self.views[s]
                        v.q = np.asarray(m[s]["q"], float)
                        v.t = m[s]["t"]
                        v._mov = float(m[s].get("mov", 0.0))
            except Exception:
                time.sleep(0.05)


class HandListener(threading.Thread):
    """Latest hand vectors from a UDP JSON stream ({"t","left","right"})."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.left = self.right = None
        self.t = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
                m = json.loads(data.decode())
                if len(m.get("left", [])) == 22:
                    self.left = m["left"]
                if len(m.get("right", [])) == 22:
                    self.right = m["right"]
                self.t = time.time()
            except Exception:
                time.sleep(0.05)


def fmt_deg(vals, width=6):
    if vals is None:
        return "  --  " * 1
    return " ".join(f"{math.degrees(v):+{width}.1f}" for v in vals)


def wrist_poses(model, data):
    """Live wrist (hand base) pose per side from the YAM arm FK: world/rig
    position (m) + intrinsic-XYZ euler (deg)."""
    out = {}
    for side in ("L", "R"):
        s = SIDE_NAME[side]
        b = model.body(f"robot_{s}_{s}_hand_C_MC").id
        eul = SciRot.from_matrix(data.xmat[b].reshape(3, 3)).as_euler("XYZ", degrees=True)
        out[side] = (data.xpos[b], eul)
    return out


def proprio_text(arms, hands, tips=None, wrists=None):
    now = time.time()
    lines = []
    for side, label in (("L", "LEFT "), ("R", "RIGHT")):
        a = arms[side]
        ok = a.q is not None and now - a.t < STALE
        mov = "MOVING" if a.moving() > 0.01 else "still "
        lines.append("YAM %s :%d %-5s %s  %s" % (
            label, ARM_PORTS[side], "LIVE" if ok else "down", mov,
            fmt_deg(a.q) if a.q is not None else "(no data)"))
        if wrists:
            p, e = wrists[side]
            lines.append("   wrist pos(m) %+.3f %+.3f %+.3f   rot XYZ(deg) %+6.1f %+6.1f %+6.1f"
                         % (p[0], p[1], p[2], e[0], e[1], e[2]))
    for side, label in (("L", "LEFT "), ("R", "RIGHT")):
        h = hands["L"] if side == "L" else hands["R"]
        ok = h is not None and now - hands["t"] < STALE
        lines.append("HAND %s %-5s %s" % (
            label, "LIVE" if ok else ("stale" if h is not None else "down"),
            "[true state]" if hands.get("src") == "state" else "[commanded]"))
        if h is not None:
            for name, a0, a1 in FINGERS:
                lines.append("   %-6s %s" % (name, fmt_deg(h[a0:a1])))
            if tips:
                lines.append("   tip/hand cm " + " ".join(
                    "%s(%+.1f,%+.1f,%+.1f)" % ((f[0].upper(),) + tuple(v))
                    for f, v in tips[side].items()))
        else:
            lines.append("   (no retargeting stream - press Retarget on the panel)")
    return "\n".join(lines)


def apply_state(model, data, addr, arms, hands):
    for side in ("L", "R"):
        a = arms[side]
        if a.q is not None:
            for adr, v in zip(addr[side]["arm"], a.q):
                data.qpos[adr] = v
        h = hands["L"] if side == "L" else hands["R"]
        if h is None:
            # no retargeting stream: show the driver's reset pose (all-zeros =
            # fully-open hand, SHARPA_INIT_POSE in teleop_panel.py) instead of
            # the bench keyframe's curled fingers - matches a reset real hand
            h = [0.0] * 22
        for adr, v, (lo, hi) in zip(addr[side]["hand"], h, addr[side]["hand_lim"]):
            data.qpos[adr] = min(max(v, lo), hi)   # what the real hand does
    mujoco.mj_forward(model, data)     # pass 1: arm FK with the new joints
    # the wrist rides the flange rigidly: hand pose = flange pose x home mount
    for side in ("L", "R"):
        A = addr[side]
        Rg = data.xmat[A["grip"]].reshape(3, 3)
        pg = data.xpos[A["grip"]]
        Rh = Rg @ A["R_mount"]
        ph = pg + Rg @ A["p_mount"]
        eul = SciRot.from_matrix(Rh).as_euler("XYZ")
        for adr, v in zip(A["wrist"], list(ph) + list(eul)):
            data.qpos[adr] = float(v)
    mujoco.mj_forward(model, data)     # pass 2: final pose incl. wrist + hands


def fingertips(model, data):
    """Per-finger distal (fingertip) positions RELATIVE to the hand base, in
    the hand frame (cm) - straight from the official Sharpa model's forward
    kinematics."""
    out = {}
    for side in ("L", "R"):
        s = SIDE_NAME[side]
        base = model.body(f"robot_{s}_{s}_hand_C_MC").id
        Rb = data.xmat[base].reshape(3, 3)
        p0 = data.xpos[base]
        tips = {}
        for fing in ("thumb", "index", "middle", "ring", "pinky"):
            b = model.body(f"robot_{s}_{s}_{fing}_DP").id
            tips[fing] = (Rb.T @ (data.xpos[b] - p0)) * 100.0
        out[side] = tips
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shot", help="render one frame to this PNG and exit (self-test)")
    ap.add_argument("--size", default="880x500")
    ap.add_argument("--embed", help="X window id (hex) to embed the UI into "
                                    "(the teleop panel's container frame)")
    args = ap.parse_args()
    W, H = (int(x) for x in args.size.split("x"))

    model, addr = build_scene()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)   # start at the calibrated home
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, H, W)
    cam = mujoco.MjvCamera()
    # default = rig.json's bench_cam (over the operator's shoulder, along +y,
    # pitched 35.9 deg down at the work). Mouse: drag=orbit, right-drag=pan,
    # wheel=zoom; arrows/+- still work.
    cam.azimuth, cam.elevation, cam.distance = 90, -32, 2.4
    cam.lookat = [0, 0.425, 0]

    al = ArmListener(ARM_UDP)
    al.start()
    arms = al.views
    hl = HandListener(HAND_UDP)          # commanded targets (fallback)
    hl.start()
    hs = HandListener(HAND_STATE_UDP)    # TRUE hand state (preferred)
    hs.start()

    # helper subprocesses, with a watchdog that respawns any that die
    # (arm_reader can be nuked by portal when the yambox drops; the others
    # can be killed by strays - the viz heals them all automatically)
    HELPERS = {
        "arm_reader": [ARM_READER],
        "hand_relay": [RELAY_PY, RELAY],
        "hand_state_reader": [STATE_READER],
        "cam_preview": [CAM_PREVIEW],   # idle grabbers -> /tmp/zhi_prev jpgs
    }
    helper_procs = {}

    def spawn_helper(name):
        helper_procs[name] = subprocess.Popen(
            HELPERS[name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL, start_new_session=True)

    for name in HELPERS:
        spawn_helper(name)

    def watch_helpers():
        for name, p in list(helper_procs.items()):
            if p.poll() is not None:
                spawn_helper(name)

    def hands_view():
        # Prefer the hands' OWN measured joint state (hand_state_reader,
        # sides straight from the hardware). Fall back to commanded targets -
        # whose left/right labels are SWAPPED on purpose: the optimizer's
        # joint_left drives the hand on the physical RIGHT side (verified
        # live 2026-08-24).
        if hs.t and time.time() - hs.t < STALE:
            return {"L": hs.left, "R": hs.right, "t": hs.t, "src": "state"}
        return {"L": hl.right, "R": hl.left, "t": hl.t, "src": "cmd"}

    if args.shot:
        time.sleep(2.0)  # let the pollers get a first sample
        apply_state(model, data, addr, arms, hands_view())
        renderer.update_scene(data, camera=cam)
        PIL.Image.fromarray(renderer.render()).save(args.shot)
        print(proprio_text(arms, hands_view(), fingertips(model, data), wrist_poses(model, data)))
        for p in helper_procs.values():
            p.terminate()
        return 0

    import signal
    import tkinter as tk
    from PIL import ImageTk
    if args.embed:
        ui = tk.Tk(use=args.embed)   # XEmbed into the teleop panel's container
    else:
        ui = tk.Tk()
        ui.title("Zhi viz panel - YAM + Sharpa live")
    ui.configure(bg="#1c1c22")
    img_label = tk.Label(ui, bg="#1c1c22")
    img_label.pack(padx=8, pady=(8, 4))
    # live camera strip: cam_preview.py (idle) or record_episode.py (during a
    # take) publishes latest-frame jpgs under /tmp/zhi_prev - a camera whose
    # jpg goes stale (>3 s) drops to "no signal"
    PREV_DIR = "/tmp/zhi_prev"
    cam_row = tk.Frame(ui, bg="#1c1c22")
    cam_widgets = {}
    for name, cap in (("head", "head cam (D435)"),
                      ("wrist_left", "wrist L (yambox)"),
                      ("wrist_right", "wrist R (yambox)")):
        f = tk.Frame(cam_row, bg="#1c1c22")
        lbl = tk.Label(f, bg="#101014", fg="#6a6a72", text="no signal")
        lbl.pack()
        tk.Label(f, text=cap, fg="#9a9aa4", bg="#1c1c22",
                 font=("TkDefaultFont", 8)).pack()
        f.pack(side="left", padx=4)
        cam_widgets[name] = lbl
    cam_state = {"shown": False}
    txt = tk.Label(ui, font=("DejaVu Sans Mono", 10), justify="left", anchor="w",
                   fg="#d8d8e0", bg="#1c1c22")
    txt.pack(fill="x", padx=10, pady=(0, 8))

    def update_cams():
        fresh_any = False
        for name, lbl in cam_widgets.items():
            p = os.path.join(PREV_DIR, name + ".jpg")
            try:
                if time.time() - os.path.getmtime(p) > 3:
                    raise OSError("stale")
                im = PIL.Image.open(p)
                im.load()
                holder["cam_" + name] = ImageTk.PhotoImage(im)
                lbl.configure(image=holder["cam_" + name], text="")
                fresh_any = True
            except Exception:
                lbl.configure(image="", text="no signal")
        if fresh_any and not cam_state["shown"]:
            cam_row.pack(after=img_label, pady=(0, 4))
            cam_state["shown"] = True
        elif not fresh_any and cam_state["shown"]:
            cam_row.pack_forget()
            cam_state["shown"] = False

    def key(ev):
        if ev.keysym == "Left":
            cam.azimuth -= 8
        elif ev.keysym == "Right":
            cam.azimuth += 8
        elif ev.keysym == "Up":
            cam.elevation = max(-89, cam.elevation - 5)
        elif ev.keysym == "Down":
            cam.elevation = min(89, cam.elevation + 5)
        elif ev.char == "+":
            cam.distance = max(0.3, cam.distance - 0.15)
        elif ev.char == "-":
            cam.distance += 0.15
    ui.bind("<Key>", key)

    # free camera: left-drag orbit, right-drag pan, wheel zoom
    drag = {"x": 0, "y": 0}

    def press(ev):
        drag["x"], drag["y"] = ev.x, ev.y
        img_label.focus_set()

    def orbit(ev):
        dx, dy = ev.x - drag["x"], ev.y - drag["y"]
        drag["x"], drag["y"] = ev.x, ev.y
        cam.azimuth -= dx * 0.4
        cam.elevation = max(-89.0, min(89.0, cam.elevation - dy * 0.4))

    def pan(ev):
        dx, dy = ev.x - drag["x"], ev.y - drag["y"]
        drag["x"], drag["y"] = ev.x, ev.y
        az = math.radians(cam.azimuth)
        # camera right/up vectors in the world xy/z frame
        rightv = [math.sin(az), -math.cos(az)]
        scale = cam.distance * 0.0015
        cam.lookat[0] -= (dx * rightv[0]) * scale
        cam.lookat[1] -= (dx * rightv[1]) * scale
        cam.lookat[2] += dy * scale

    def wheel(ev):
        up = getattr(ev, "num", 0) == 4 or getattr(ev, "delta", 0) > 0
        cam.distance = max(0.3, cam.distance + (-0.12 if up else 0.12))

    for w_ in (img_label, ui):
        w_.bind("<ButtonPress-1>", press)
        w_.bind("<B1-Motion>", orbit)
        w_.bind("<ButtonPress-3>", press)
        w_.bind("<B3-Motion>", pan)
        w_.bind("<Button-4>", wheel)
        w_.bind("<Button-5>", wheel)
        w_.bind("<MouseWheel>", wheel)

    def die(*_):
        try:
            for p in helper_procs.values():
                p.terminate()
        finally:
            os._exit(0)
    signal.signal(signal.SIGTERM, die)
    signal.signal(signal.SIGINT, die)

    holder = {}

    tickn = {"n": 0}

    def tick():
        apply_state(model, data, addr, arms, hands_view())
        renderer.update_scene(data, camera=cam)
        frame = PIL.Image.fromarray(renderer.render())
        holder["img"] = ImageTk.PhotoImage(frame)
        img_label.configure(image=holder["img"])
        txt.configure(text=proprio_text(arms, hands_view(), fingertips(model, data), wrist_poses(model, data)))
        tickn["n"] += 1
        if tickn["n"] % 5 == 0:   # camera previews at ~4 Hz
            update_cams()
        if tickn["n"] % 60 == 0:  # heal dead helper processes every ~3 s
            watch_helpers()
        ui.after(50, tick)  # ~20 Hz

    def on_close():
        for p in helper_procs.values():
            p.terminate()
        ui.destroy()

    ui.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    ui.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
