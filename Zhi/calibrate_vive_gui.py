#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Live Vive calibration panel (Wuji-calibration style) - POSITION and WRIST
ROTATION modes.

POSITION mode: 3 hand strokes along the arm's model axes -> stream->model
map M (vive_arm_map.json[side]); teleop position AND (absent a rotation map)
orientation use it.

WRIST ROTATION mode (2026-08-27): 3 hand ROTATIONS about the arm's model
axes -> a separate rotation map (vive_arm_map.json[side + "_rot"]) that
manus_arm_teleop conjugates wrist deltas with, so every tracker angle change
translates exactly to the yam wrist. Right-hand rule about each axis; each
rotation is validated for perpendicularity on the spot and rejected with the
measured angle if off.

Common to both modes:
  - AUTO-DETECTS which tracker is in your hand (the one that moves/rotates)
  - stage checklist, hold bar, green flash + beep on every capture
  - tracker health footer with a duplicate-stream alarm
  - nothing written until SAVE (previous file kept at .prev)

The ARM NEVER MOVES.
"""
import json
import os
import socket
import time

import numpy as np
import tkinter as tk
from scipy.spatial.transform import Rotation as SciRot

HERE = os.path.dirname(os.path.realpath(__file__))
MAP_FILE = os.path.join(HERE, "params", "vive_arm_map.json")
STREAM_PORT = {"left": 9873, "right": 9874}
STILL_S = 2.0
POS_SPREAD = 0.010       # m: position stillness (position mode)
POS_SPREAD_ROT = 0.030   # m: looser position hold while rotating in place
ORI_SPREAD = 3.0         # deg: orientation stillness
MIN_STROKE = 0.15        # m
MIN_ROT = 30.0           # deg per rotation stroke
MAX_ROT = 120.0          # deg: near 180 the rotation AXIS SIGN is ambiguous
#                          (a 178 deg yaw produced a det -1 map, 2026-08-27)
PERP_COS = 0.5           # reject axes >30 deg off perpendicular

POS_STAGES = [
    ("Start point", "Pick up ONE tracker and hold it STILL for 2 seconds"),
    ("Stroke 1: arm FORWARD (+x)",
     "Move 30+ cm along this arm's FORWARD direction\n"
     "(from the arm's base toward its workspace), then hold STILL 2 s"),
    ("Stroke 2: arm LEFT (+y)",
     "Move 30+ cm to this arm's LEFT\n"
     "(90° counterclockwise from its forward, horizontal), then hold STILL 2 s"),
    ("Stroke 3: straight UP (+z)",
     "Move 30+ cm STRAIGHT UP, then hold STILL 2 s"),
    ("Review & save", "Check quality below, then Save or Discard"),
]
ROT_STAGES = [
    ("Start pose", "Pick up ONE tracker and hold it STILL for 2 seconds"),
    ("Rotation 1: ROLL about FORWARD (+x)",
     "Twist the tracker 45°+ about the arm's FORWARD axis:\n"
     "tip its LEFT edge UP (keep the tracker in place), then hold STILL 2 s"),
    ("Rotation 2: PITCH about LEFT (+y)",
     "Rotate 45°+ about the arm's LEFT axis:\n"
     "nod the tracker's TOP toward the arm's FORWARD, then hold STILL 2 s"),
    ("Rotation 3: YAW about UP (+z)",
     "Turn the tracker 45°+ about VERTICAL, toward the arm's LEFT\n"
     "(counterclockwise seen from above), then hold STILL 2 s"),
    ("Review & save", "Check quality below, then Save or Discard"),
]

BG, PANEL, FG, DIM = "#1c1c22", "#232329", "#e8e8ee", "#9a9aa4"
GOOD, WARN, BAD, ACCENT = "#2e8b57", "#d4a017", "#c0392b", "#3d6fb4"


def pos_spread(pts, c):
    d = np.linalg.norm(pts - c, axis=1)
    return float(np.percentile(d, 90))


def mean_rot(quats):
    """Mean rotation of wxyz quats (scipy wants xyzw)."""
    q = np.asarray(quats)
    return SciRot.from_quat(np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])).mean()


def ori_spread_deg(quats, mean):
    q = np.asarray(quats)
    rs = SciRot.from_quat(np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]]))
    rel = (mean.inv() * rs).magnitude()
    return float(np.degrees(np.percentile(rel, 90)))


class App:
    def __init__(self):
        self.ui = tk.Tk()
        self.ui.title("Vive arm calibration")
        self.ui.configure(bg=BG)
        self.ui.geometry("920x520")

        top = tk.Frame(self.ui, bg=BG); top.pack(fill="x", padx=14, pady=(12, 4))
        tk.Label(top, text="Arm:", bg=BG, fg=DIM).pack(side="left")
        self.side = tk.StringVar(value="left")
        for s in ("left", "right"):
            tk.Radiobutton(top, text=s, value=s, variable=self.side, bg=BG, fg=FG,
                           selectcolor="#33333d", activebackground=BG,
                           activeforeground=FG, highlightthickness=0).pack(side="left")
        tk.Label(top, text="   Mode:", bg=BG, fg=DIM).pack(side="left")
        self.mode = tk.StringVar(value="position")
        for m, lbl in (("position", "position"), ("rotation", "wrist rotation")):
            tk.Radiobutton(top, text=lbl, value=m, variable=self.mode, bg=BG, fg=FG,
                           selectcolor="#33333d", activebackground=BG,
                           activeforeground=FG, highlightthickness=0).pack(side="left")
        self.start_btn = tk.Button(top, text="▶  Start", command=self.start,
                                   bg="#245c40", fg=FG, bd=0, padx=16, pady=4)
        self.start_btn.pack(side="right")
        self.save_btn = tk.Button(top, text="\U0001f4be  Save", command=self.save,
                                  bg=ACCENT, fg=FG, bd=0, padx=16, pady=4,
                                  state="disabled")
        self.save_btn.pack(side="right", padx=(0, 8))
        self.discard_btn = tk.Button(top, text="Discard", command=self.reset_idle,
                                     bg="#5c2b27", fg=FG, bd=0, padx=12, pady=4,
                                     state="disabled")
        self.discard_btn.pack(side="right", padx=(0, 8))

        body = tk.Frame(self.ui, bg=BG); body.pack(fill="both", expand=True,
                                                   padx=14, pady=6)
        steps = tk.Frame(body, bg=PANEL, padx=12, pady=10)
        steps.pack(side="left", fill="y")
        tk.Label(steps, text="STEPS", bg=PANEL, fg=DIM,
                 font=("TkDefaultFont", 9, "bold")).pack(anchor="w", pady=(0, 6))
        self.step_lbls = [tk.Label(steps, text="", bg=PANEL, fg=DIM, anchor="w",
                                   font=("TkDefaultFont", 10)) for _ in range(5)]
        for l in self.step_lbls:
            l.pack(anchor="w", pady=2)

        main = tk.Frame(body, bg=BG); main.pack(side="left", fill="both",
                                                expand=True, padx=(14, 0))
        self.stage_lbl = tk.Label(main, text="press Start", bg=BG, fg=FG,
                                  font=("TkDefaultFont", 18, "bold"))
        self.stage_lbl.pack(pady=(16, 2))
        self.instr_lbl = tk.Label(main, text="pick arm + mode, then Start",
                                  bg=BG, fg=DIM, font=("TkDefaultFont", 12),
                                  justify="center")
        self.instr_lbl.pack(pady=(0, 10))

        self.hold_canvas = tk.Canvas(main, width=520, height=30, bg="#101014",
                                     highlightthickness=0)
        self.hold_canvas.pack(pady=4)
        self.hold_fill = self.hold_canvas.create_rectangle(0, 0, 0, 30,
                                                           fill=GOOD, width=0)
        self.hold_txt = self.hold_canvas.create_text(260, 15, text="", fill=FG,
                                                     font=("TkDefaultFont", 10, "bold"))
        self.live_lbl = tk.Label(main, text="", bg=BG, fg=DIM,
                                 font=("DejaVu Sans Mono", 11), justify="center")
        self.live_lbl.pack(pady=8)
        self.result_lbl = tk.Label(main, text="", bg=BG, fg=FG,
                                   font=("DejaVu Sans Mono", 10), justify="left")
        self.result_lbl.pack(pady=4)
        self.health_lbl = tk.Label(self.ui, text="", bg=BG, fg=DIM,
                                   font=("DejaVu Sans Mono", 9))
        self.health_lbl.pack(side="bottom", pady=(0, 8))

        self.rx = None
        self.state = "idle"
        self.run_mode = "position"
        self.windows = {"left": [], "right": []}   # (t, pos, quat wxyz)
        self.start_snap = {}                       # tracker -> (pos, mean_rot)
        self.held = None
        self.done_axes = []                        # accepted axis unit vectors
        self.pos_waypoints = []                    # position mode waypoints
        self.rot_ref = None                        # rotation mode: last accepted pose
        self.M = None
        self.pkt_times = []
        self.seen_valid = {"left": False, "right": False}
        self.mark_steps()
        self.ui.after(66, self.tick)

    # ---------------- helpers ----------------
    def stages(self):
        return POS_STAGES if self.run_mode == "position" else ROT_STAGES

    def n_done(self):
        base = 1 if self.start_snap else 0
        if self.run_mode == "position":
            return len(self.pos_waypoints) or base
        return base + len(self.done_axes)

    def mark_steps(self):
        st = self.stages()
        done = self.n_done()
        for i, l in enumerate(self.step_lbls):
            title = st[i][0]
            if i < done:
                l.configure(text="✓  " + title, fg=GOOD)
            elif (i == done and self.state == "run") or \
                 (self.state == "done" and i == 4):
                l.configure(text="▶  " + title, fg=FG)
            else:
                l.configure(text="○  " + title, fg=DIM)

    def show_stage(self, i, flash=False):
        title, instr = self.stages()[i]
        held = (" — '%s' tracker" % self.held) if self.held else ""
        self.stage_lbl.configure(text=title.upper(), fg=GOOD if flash else FG)
        self.instr_lbl.configure(text="%s   (%s ARM%s)" % (instr, self.side.get(), held))
        if flash:
            self.ui.bell()
            self.ui.after(800, lambda: self.stage_lbl.configure(fg=FG))

    def reject(self, msg):
        self.stage_lbl.configure(text="✗  REJECTED - REDO", fg=BAD)
        self.instr_lbl.configure(text=msg)
        for _ in range(3):
            self.ui.bell()

    def draw_hold(self, frac, moving):
        self.hold_canvas.coords(self.hold_fill, 0, 0, int(520 * frac), 30)
        self.hold_canvas.itemconfigure(
            self.hold_txt,
            text=("HOLD ... %.0f%%" % (frac * 100)) if 0 < frac < 1 else
                 ("MOVING" if moving else "STILL"))

    # ---------------- flow ----------------
    def start(self):
        self.run_mode = self.mode.get()
        side = self.side.get()
        if self.rx:
            self.rx.close()
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("127.0.0.1", STREAM_PORT[side]))
        self.rx.setblocking(False)
        self.windows = {"left": [], "right": []}
        self.start_snap, self.done_axes, self.pos_waypoints = {}, [], []
        self.held, self.M, self.rot_ref = None, None, None
        self.pkt_times = []
        # rotation mode: the verified POSITION map already defines each arm
        # axis in the stream frame (rows of M) - use it to auto-correct a
        # backwards rotation direction and to reject off-axis rotations
        # (repeated det -1 results from direction mistakes, 2026-08-27)
        self.exp_rows = None
        self.corrected = []
        if self.run_mode == "rotation":
            try:
                self.exp_rows = np.asarray(
                    json.load(open(MAP_FILE))[side], float)
            except Exception:
                self.exp_rows = None
        self.state = "run"
        self.save_btn.configure(state="disabled")
        self.discard_btn.configure(state="disabled")
        self.result_lbl.configure(text="")
        self.show_stage(0)
        self.mark_steps()

    def reset_idle(self):
        self.state = "idle"
        self.M, self.held, self.rot_ref = None, None, None
        self.done_axes, self.pos_waypoints = [], []
        self.save_btn.configure(state="disabled")
        self.discard_btn.configure(state="disabled")
        self.stage_lbl.configure(text="press Start", fg=FG)
        self.instr_lbl.configure(text="pick arm + mode, then Start")
        self.result_lbl.configure(text="")
        self.mark_steps()

    def finished(self, cols, mags, unit):
        U = np.array(cols).T
        W, S, Vt = np.linalg.svd(U)
        R = W @ Vt
        self.M = R.T
        det = float(np.linalg.det(R))
        quality = float(np.min(S) / np.max(S))
        self.state = "done"
        self.mark_steps()
        ok = quality >= 0.7 and det > 0
        self.stage_lbl.configure(
            text="✓  CALIBRATION COMPLETE" if ok else
            ("⚠  MIRRORED RESULT" if det < 0 else "⚠  LOW QUALITY"),
            fg=GOOD if ok else WARN)
        note = ""
        if det < 0:
            note = ("  -  det -1: one motion went the REVERSED direction "
                    "(the arm faces you: forward = toward table center, "
                    "arm-left = YOUR right, yaw CCW from above). Discard and redo.")
        elif not ok:
            note = "  -  axes not perpendicular; consider redoing"
        self.instr_lbl.configure(text="review, then Save or Discard" + note)
        lines = ["mode: %s   tracker: '%s'" % (self.run_mode, self.held),
                 "magnitudes: " + "  ".join("%.0f%s" % (v, unit) for v in mags),
                 "quality %.2f   det %+d%s" % (quality, round(det),
                                               "  [MIRRORED]" if det < 0 else "")]
        if self.corrected:
            lines.append("direction auto-corrected: %s (rotated opposite - OK)"
                         % ", ".join(self.corrected))
        for row in np.round(self.M, 3):
            lines.append("  [% .3f % .3f % .3f]" % tuple(row))
        self.result_lbl.configure(text="\n".join(lines))
        self.save_btn.configure(state="normal")
        self.discard_btn.configure(state="normal")
        self.ui.bell(); self.ui.bell()

    def save(self):
        side = self.side.get()
        key = side if self.run_mode == "position" else side + "_rot"
        try:
            m = json.load(open(MAP_FILE))
        except Exception:
            m = {}
        try:
            json.dump(m, open(MAP_FILE + ".prev", "w"), indent=1)
        except Exception:
            pass
        m[key] = [[float(v) for v in row] for row in self.M]
        json.dump(m, open(MAP_FILE, "w"), indent=1)
        self.stage_lbl.configure(text="✓  SAVED  ['%s']" % key, fg=GOOD)
        self.instr_lbl.configure(
            text="teleop uses it on its next start (Stop + Start Teleop). "
                 "Previous file at .prev. Calibrate the other side/mode, or close.")
        self.save_btn.configure(state="disabled")

    # ---------------- validation ----------------
    AXIS_NAMES = ["FORWARD (+x)", "LEFT (+y)", "UP (+z)"]

    def check_rot_axis(self, axis):
        """Rotation mode: compare against the position map's expected axis.
        Returns (axis, None) possibly sign-corrected, or (None, errmsg)."""
        if self.exp_rows is None:
            return axis, None
        idx = len(self.done_axes)
        exp = self.exp_rows[idx] / np.linalg.norm(self.exp_rows[idx])
        d = float(np.dot(axis, exp))
        ang = float(np.degrees(np.arccos(np.clip(abs(d), -1, 1))))
        if abs(d) < 0.5:
            return None, ("that rotation was %.0f° OFF the arm's %s axis. "
                          "Hold still, then rotate about that axis only."
                          % (ang, self.AXIS_NAMES[idx]))
        if d < 0:
            self.corrected.append(self.AXIS_NAMES[idx])
            return -axis, None       # opposite direction - sign auto-corrected
        return axis, None

    def try_accept_axis(self, axis, mag, unit, min_mag):
        """Perpendicularity gate; True if accepted."""
        if mag < min_mag:
            return False           # not enough motion yet - keep waiting quietly
        bad = [(i + 1, abs(float(np.dot(axis, v)))) for i, v in enumerate(self.done_axes)]
        worst = max(bad, key=lambda x: x[1]) if bad else None
        if worst and worst[1] > PERP_COS:
            ang = float(np.degrees(np.arccos(np.clip(worst[1], -1, 1))))
            self.reject("only %.0f° from axis %d (need ~90°). Hold still, then "
                        "redo this %s PERPENDICULAR to the previous ones."
                        % (ang, worst[0],
                           "stroke" if self.run_mode == "position" else "rotation"))
            return None            # rejected - caller must rebase
        self.done_axes.append(axis)
        return True

    # ---------------- 15 Hz loop ----------------
    def tick(self):
        self.ui.after(66, self.tick)
        if self.rx is None:
            return
        now = time.time()
        while True:
            try:
                data, _ = self.rx.recvfrom(65536)
            except (BlockingIOError, OSError):
                break
            self.pkt_times.append(now)
            try:
                msg = json.loads(data.decode())
            except ValueError:
                continue
            for g in msg.get("gloves", []):
                s = g.get("side")
                if s in self.windows:
                    self.seen_valid[s] = bool(g.get("valid"))
                    if g.get("valid") and "p" in g and "q" in g:
                        self.windows[s].append((now, np.asarray(g["p"], float),
                                                np.asarray(g["q"], float)))

        self.pkt_times = [t for t in self.pkt_times if now - t < 2.0]
        rate = len(self.pkt_times) / 2.0
        health = "trackers:  left %s   right %s   |   stream %3.0f Hz" % (
            "OK " if self.seen_valid["left"] else "-- ",
            "OK " if self.seen_valid["right"] else "-- ", rate)
        if rate > 80:
            self.health_lbl.configure(fg=BAD, text=health +
                "   !! TWO STREAMS - kill the duplicate !!")
        else:
            self.health_lbl.configure(fg=DIM, text=health)
        if self.state != "run":
            return

        stats = {}
        for s in self.windows:
            self.windows[s] = [w for w in self.windows[s] if now - w[0] <= STILL_S]
            w = self.windows[s]
            if len(w) < 8:
                continue
            pts = np.asarray([p for _, p, _ in w])
            c = pts.mean(axis=0)
            mr = mean_rot([q for _, _, q in w])
            stats[s] = dict(c=c, psp=pos_spread(pts, c),
                            rot=mr, osp=ori_spread_deg([q for _, _, q in w], mr),
                            span=w[-1][0] - w[0][0])
        if not stats:
            self.live_lbl.configure(text="waiting for trackers ...")
            return

        pos_lim = POS_SPREAD if self.run_mode == "position" else POS_SPREAD_ROT

        def is_still(st):
            still = st["psp"] < pos_lim and st["span"] >= STILL_S * 0.9
            if self.run_mode == "rotation":
                still = still and st["osp"] < ORI_SPREAD
            return still

        # ---- stage 0: capture start for all trackers ----
        if not self.start_snap:
            worst = max(s["psp"] for s in stats.values())
            ok0 = all(is_still(s) for s in stats.values())
            frac = min(min(s["span"] for s in stats.values()), STILL_S) / STILL_S \
                if all(s["psp"] < pos_lim for s in stats.values()) else 0.0
            self.draw_hold(frac, worst >= pos_lim)
            self.live_lbl.configure(text="stillness %5.1f mm" % (worst * 1000))
            if ok0:
                self.start_snap = {s: (st["c"].copy(), st["rot"])
                                   for s, st in stats.items()}
                if self.run_mode == "position":
                    self.pos_waypoints = []
                self.show_stage(1, flash=True)
                self.mark_steps()
            return

        # ---- detect the held tracker on motion 1 ----
        if self.held is None:
            def deviation(s):
                st = stats[s]
                p0, r0 = self.start_snap[s]
                if self.run_mode == "position":
                    return float(np.linalg.norm(st["c"] - p0))
                return float(np.degrees((st["rot"] * r0.inv()).magnitude()))
            cands = [s for s in stats if s in self.start_snap]
            mover = max(cands, key=deviation)
            st = stats[mover]
            dev = deviation(mover)
            thresh = MIN_STROKE if self.run_mode == "position" else MIN_ROT
            unit = "cm" if self.run_mode == "position" else "°"
            shown = dev * 100 if self.run_mode == "position" else dev
            ok_hold = is_still(st) and dev >= thresh
            self.draw_hold((min(st["span"], STILL_S) / STILL_S) if ok_hold else 0.0,
                           not is_still(st))
            self.live_lbl.configure(
                text="moved %5.1f %s ('%s' tracker)%s"
                     % (shown, unit, mover,
                        "" if dev >= thresh else "   (need %s)"
                        % ("15+ cm" if unit == "cm" else "30°+")))
            if is_still(st) and dev >= thresh:
                p0, r0 = self.start_snap[mover]
                if self.run_mode == "position":
                    axis = (st["c"] - p0) / np.linalg.norm(st["c"] - p0)
                    mag = float(np.linalg.norm(st["c"] - p0)) * 100
                else:
                    rv = (st["rot"] * r0.inv()).as_rotvec()
                    axis = rv / np.linalg.norm(rv)
                    mag = float(np.degrees(np.linalg.norm(rv)))
                    if mag > MAX_ROT:
                        self.start_snap[mover] = (st["c"].copy(), st["rot"])
                        self.reject("rotated %.0f° - too far! Near 180° the "
                                    "axis direction is ambiguous. Hold still, "
                                    "then redo at 45–90°." % mag)
                        return
                    axis2, err = self.check_rot_axis(axis)
                    if err:
                        self.start_snap[mover] = (st["c"].copy(), st["rot"])
                        self.reject(err)
                        return
                    axis = axis2
                self.held = mover
                self.done_axes = [axis]
                self.mags = [mag]
                if self.run_mode == "position":
                    self.pos_waypoints = [p0.copy(), st["c"].copy()]
                else:
                    self.rot_ref = st["rot"]
                    self.rot_start = r0
                self.mark_steps()
                self.show_stage(2, flash=True)
            return

        # ---- motions 2 and 3 ----
        if self.held not in stats:
            self.live_lbl.configure(text="'%s' tracker occluded ..." % self.held)
            return
        st = stats[self.held]
        if self.run_mode == "position":
            ref = self.pos_waypoints[-1]
            dev = float(np.linalg.norm(st["c"] - ref))
            shown, unit, thresh = dev * 100, "cm", MIN_STROKE
        else:
            dev = float(np.degrees((st["rot"] * self.rot_ref.inv()).magnitude()))
            shown, unit, thresh = dev, "°", MIN_ROT
        ok_hold = is_still(st) and dev >= thresh
        self.draw_hold((min(st["span"], STILL_S) / STILL_S) if ok_hold else 0.0,
                       not is_still(st))
        self.live_lbl.configure(
            text="moved %5.1f %s%s" % (shown, unit,
                 "" if dev >= thresh else "   (need %s)" %
                 ("15+ cm" if unit == "cm" else "30°+")))
        if is_still(st) and dev >= thresh:
            if self.run_mode == "position":
                u = st["c"] - self.pos_waypoints[-1]
                axis, mag = u / np.linalg.norm(u), float(np.linalg.norm(u)) * 100
            else:
                rv = (st["rot"] * self.rot_ref.inv()).as_rotvec()
                axis, mag = rv / np.linalg.norm(rv), float(np.degrees(np.linalg.norm(rv)))
                if mag > MAX_ROT:
                    self.rot_ref = st["rot"]      # rebase; redo from here
                    self.reject("rotated %.0f° - too far! Near 180° the axis "
                                "direction is ambiguous. Hold still, then redo "
                                "this rotation at 45–90°." % mag)
                    return
                axis2, err = self.check_rot_axis(axis)
                if err:
                    self.rot_ref = st["rot"]
                    self.reject(err)
                    return
                axis = axis2
            res = self.try_accept_axis(axis, 1.0, unit, 0.0)
            if res is None:                      # rejected: rebase here
                if self.run_mode == "position":
                    self.pos_waypoints[-1] = st["c"].copy()
                else:
                    self.rot_ref = st["rot"]
                return
            self.mags.append(mag)
            if self.run_mode == "position":
                self.pos_waypoints.append(st["c"].copy())
            else:
                self.rot_ref = st["rot"]
            self.mark_steps()
            if len(self.done_axes) >= 3:
                self.finished(self.done_axes, self.mags, unit)
            else:
                self.show_stage(len(self.done_axes) + 1, flash=True)


if __name__ == "__main__":
    App().ui.mainloop()
