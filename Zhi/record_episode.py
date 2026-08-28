#!/home/yam/gck/i2rt/.venv/bin/python
"""One teleop episode: cameras + robot state + hand actions -> h5 + mkv.

Layout follows the T-Rex raw convention (episode_*/ dirs, one HDF5 + one
video file per camera) with LeRobot-v3-style keys, so episodes can later be
packed into a LeRobotDataset (parquet + mp4 shards) without reshaping:

    episodes/episode_000042/
        episode.h5          synchronized low-dim data (see below)
        head.mkv            D435 colour over the table, 640x480@30
        ir_left.mkv         the same D435's infrared PAIR (8-bit grey,
        ir_right.mkv        emitter off; stamps in /observation/ir_t)
        wrist_left.mkv      yambox wrist cams (YHTek MJPG), 640x480@30,
        wrist_right.mkv     recorded on the yambox and copied back
        meta.json           what was live, cams, ports, mapping version

    episode.h5:
        /timestamps                 [N]      unix time of each 30 Hz tick
        /observation/state/arm_left  [N,D]   follower joint pos (rad), NaN if down
        /observation/state/arm_right [N,D]
        /action/hand                [M,22]   true post-clamp hand targets (rad,
        /action/hand_t              [M]      Sharpa order) teed by the driver
        /observation/tactile_f6     [K,5,6]  per-fingertip 6-axis wrench - only
        /observation/tactile_t      [K]      written if a publisher sends it
                                             (reserved; fw currently blocks it)

Arm *action* note: only the follower is queryable (portal get_joint_pos is a
cached read; polling does not disturb teleop). Absolute arm targets a la
T-Rex's action_abs can be derived offline as state[t+1]; the true leader
command would need a minimum_gello change.

Run under the i2rt venv python (shebang). SIGINT/SIGTERM finalizes the take:
stops both ffmpegs with SIGINT (container headers get written), copies the
head footage back from the yambox, writes the h5.
"""
import argparse, glob, json, os, signal, socket, subprocess, sys, threading, time

import h5py
import numpy as np
import portal

YAMBOX = os.environ.get("YAMBOX", "yambox@192.168.1.9")
YAMBOX_IP = os.environ.get("YAMBOX_IP", "192.168.1.9")
SSH = ["ssh", "-n", "-o", "ConnectTimeout=4", "-o", "ControlMaster=auto",
       "-o", "ControlPath=/tmp/zhi_rec_ssh", "-o", "ControlPersist=120"]
ARM_PORTS = {"arm_left": 11334, "arm_right": 11333}
STATUS = "/tmp/zhi_record_status.json"


def sh(cmd, timeout=15):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", "timeout")
    except OSError as e:   # missing binary (v4l2-ctl bit us on 2026-08-27)
        return subprocess.CompletedProcess(cmd, 126, "", str(e))


def ssh(remote, timeout=15):
    return sh(SSH + [YAMBOX, remote], timeout)


# Cameras (roles re-chosen by the user 2026-08-25 evening):
#   head        D435 sn 317622076020 @ usb 71:00.4 port 2, LOCAL  (+ IR pair)
#   wrist_left / wrist_right = the two USB cameras plugged into the YAMBOX
#                (YHTek MJPG UVC cams). Recorded ON the yambox (ffmpeg MJPG
#                stream-copy, near-zero CPU) and copied back at finalize.
# The local D405s are NOT the wrist cams any more.
# Auto-assignment: the yambox cams are sorted by their /dev/v4l/by-path id and
# mapped [wrist_left, wrist_right] in that order. Once both are plugged in,
# pin them here (by-path prefix WITHOUT the trailing ":1.0-video-indexN") if
# the auto order comes out mirrored; leave None to keep auto.
YAMBOX_WRIST_BYPATH = {
    "wrist_left":  None,
    "wrist_right": None,
}
YAMBOX_TAKE_DIR = "/tmp/zhi_take"
YAMBOX_REC_MARKER = "zhi_wrist_rec"


def yambox_wrist_cams():
    """name -> yambox /dev/v4l/by-path capture node for each wrist camera.
    Auto-discovers the cameras plugged into the yambox (index0 = the capture
    node of each UVC device); honors YAMBOX_WRIST_BYPATH pins when set."""
    cp = ssh("ls /dev/v4l/by-path/ 2>/dev/null")
    nodes = sorted(n for n in cp.stdout.split()
                   if n.endswith("video-index0") and "usbv" not in n)
    out = {"wrist_left": None, "wrist_right": None}
    for name, pin in YAMBOX_WRIST_BYPATH.items():
        if pin:
            m = [n for n in nodes if n.startswith(pin)]
            if m:
                out[name] = "/dev/v4l/by-path/" + m[0]
                nodes.remove(m[0])
    for name in ("wrist_left", "wrist_right"):
        if out[name] is None and nodes:
            out[name] = "/dev/v4l/by-path/" + nodes.pop(0)
    return out


def start_yambox_cam(name, dev):
    """Start an MJPG stream-copy recording of `dev` on the yambox. Returns True
    if the recorder came up. SIGINT at stop closes the container cleanly."""
    r = ssh("mkdir -p %s && rm -f %s/%s.mkv && "
            "setsid nohup ffmpeg -y -loglevel error -f v4l2 -input_format mjpeg "
            "-video_size 640x480 -framerate 30 -use_wallclock_as_timestamps 1 "
            "-i %s -c:v copy -metadata comment=%s %s/%s.mkv "
            "> %s/%s.log 2>&1 < /dev/null &"
            % (YAMBOX_TAKE_DIR, YAMBOX_TAKE_DIR, name,
               dev, YAMBOX_REC_MARKER, YAMBOX_TAKE_DIR, name,
               YAMBOX_TAKE_DIR, name))
    time.sleep(1.5)
    return ssh("pgrep -f 'ffmpeg.*%s/%s.mkv' >/dev/null && echo up" %
               (YAMBOX_TAKE_DIR, name)).stdout.strip() == "up"


def stop_yambox_cams(dest, names):
    """SIGINT the yambox recorders, wait for them to close, scp the takes back."""
    ssh("pkill -INT -f 'ffmpeg.*%s' 2>/dev/null; true" % YAMBOX_TAKE_DIR)
    for _ in range(20):
        if ssh("pgrep -f 'ffmpeg.*%s' >/dev/null && echo live" %
               YAMBOX_TAKE_DIR).stdout.strip() != "live":
            break
        time.sleep(0.5)
    for name in names:
        cp = sh(["scp", "-o", "ConnectTimeout=4",
                 "-o", "ControlPath=/tmp/zhi_rec_ssh",
                 "%s:%s/%s.mkv" % (YAMBOX, YAMBOX_TAKE_DIR, name),
                 os.path.join(dest, name + ".mkv")], timeout=120)
        if cp.returncode != 0:
            print("WARNING: could not copy %s.mkv from yambox: %s"
                  % (name, cp.stderr.strip()), flush=True)
    ssh("rm -rf %s" % YAMBOX_TAKE_DIR)


def resolve_side_cam():
    """The local D435's colour (YUYV) capture node. Discovered by probing, not
    by a pinned PCI path: the by-path prefix moved between machines (hermes
    71:00.4 -> jdw-Lambda-Vector 47:00.1), and v4l2-ctl isn't installed here,
    so identify a RealSense usb node that lists yuyv422 raw frames via the
    tools we already depend on (udevadm + ffmpeg)."""
    for n in sorted(glob.glob("/dev/v4l/by-path/*-video-index0")):
        inf = sh(["udevadm", "info", "--query=property", "--name", n])
        if "RealSense" not in inf.stdout:
            continue
        cp = sh(["ffmpeg", "-hide_banner", "-f", "v4l2",
                 "-list_formats", "all", "-i", n], timeout=10)
        if "yuyv422" in (cp.stdout + cp.stderr):
            return n
    return None


# live-preview side channel: while a take runs, each camera also publishes a
# low-rate "latest frame" jpg under PREV_DIR; the viz panel shows them as a
# camera strip. Piggybacks on the SAME ffmpeg that records, so no device is
# opened twice. Between takes viz/cam_preview.py keeps the same jpgs alive
# with grab-only ffmpegs tagged IDLE_MARKER - they must release the devices
# before we open them (see the hand-off in main()).
PREV_DIR = "/tmp/zhi_prev"
IDLE_MARKER = "zhi_idle_preview"


def ffmpeg_args(dev, out, preview=None):
    args = ["ffmpeg", "-y", "-f", "v4l2", "-input_format", "yuyv422",
            "-video_size", "640x480", "-framerate", "30",
            "-use_wallclock_as_timestamps", "1", "-i", dev,
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23",
            "-pix_fmt", "yuv420p", out]
    if preview:
        args += ["-vf", "fps=4,scale=320:240", "-update", "1", preview]
    return args


# the side/head D435 on THIS machine (jdw-Lambda-Vector, since 2026-08-27);
# the hermes-era unit was 317622076020
SIDE_D435_SERIAL = "327343020514"


class IRRecorder(threading.Thread):
    """Side D435 infrared PAIR -> ir_left.mkv / ir_right.mkv (8-bit grey via
    x264) + per-frame wall-clock stamps. librealsense streams infra1+infra2
    while ffmpeg keeps recording the color node - verified concurrent
    2026-08-25 (start AFTER the color ffmpeg). Emitter forced OFF so the IR
    images are clean (no projector dot pattern)."""

    def __init__(self, dest, w=640, h=480, fps=30):
        super().__init__(daemon=True)
        self.dest, self.w, self.h, self.fps = dest, w, h, fps
        self.stop_flag = False
        self.frames = 0
        self.ts = []

    def run(self):
        try:
            import pyrealsense2 as rs
        except ImportError:
            print("WARNING: pyrealsense2 missing - IR pair not recorded", flush=True)
            return
        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(SIDE_D435_SERIAL)
            cfg.enable_stream(rs.stream.infrared, 1, self.w, self.h, rs.format.y8, self.fps)
            cfg.enable_stream(rs.stream.infrared, 2, self.w, self.h, rs.format.y8, self.fps)
            prof = pipe.start(cfg)
            for s in prof.get_device().query_sensors():
                if s.supports(rs.option.emitter_enabled):
                    s.set_option(rs.option.emitter_enabled, 0)
        except Exception as e:
            print("WARNING: IR pair failed to start: %s" % e, flush=True)
            return
        enc = []
        for name in ("ir_left", "ir_right"):
            enc.append(subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                 "-pix_fmt", "gray", "-s", "%dx%d" % (self.w, self.h),
                 "-r", str(self.fps), "-i", "-",
                 "-c:v", "libx264", "-preset", "ultrafast", "-crf", "20",
                 "-pix_fmt", "yuv420p", os.path.join(self.dest, name + ".mkv")],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL))
        while not self.stop_flag:
            try:
                fr = pipe.wait_for_frames(2000)
            except Exception:
                break
            i1 = np.asanyarray(fr.get_infrared_frame(1).get_data())
            i2 = np.asanyarray(fr.get_infrared_frame(2).get_data())
            self.ts.append(time.time())
            try:
                enc[0].stdin.write(i1.tobytes())
                enc[1].stdin.write(i2.tobytes())
            except BrokenPipeError:
                break
            self.frames += 1
        try:
            pipe.stop()
        except Exception:
            pass
        for p in enc:
            try:
                p.stdin.close()
                p.wait(timeout=10)
            except Exception:
                pass


def open_udp(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.setblocking(False)
    return s


def drain(sock):
    out = []
    while True:
        try:
            data, _ = sock.recvfrom(8192)
        except BlockingIOError:
            return out
        except OSError:
            return out
        try:
            out.append(json.loads(data.decode()))
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser("~/Desktop/Zhi/episodes"))
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--hand-udp", type=int, default=59250)
    ap.add_argument("--tactile-udp", type=int, default=59300)
    ap.add_argument("--no-cams", action="store_true", help="skip video (debug)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    n = max([int(os.path.basename(d).split("_")[1])
             for d in glob.glob(os.path.join(args.out, "episode_*")) if
             os.path.basename(d).split("_")[-1].isdigit()] or [0]) + 1
    dest = os.path.join(args.out, "episode_%06d" % n)
    os.makedirs(dest, exist_ok=True)
    remote = "/tmp/zhi_rec_ep%d" % n
    print("episode %d -> %s" % (n, dest), flush=True)

    # ---- arms: portal clients for whichever follower ports answer ----
    # portal.Client blocks forever on a dead port, so gate with a raw TCP probe
    def port_open(port):
        try:
            s = socket.create_connection((YAMBOX_IP, port), timeout=2)
            s.close()
            return True
        except OSError:
            return False

    arms = {}
    for name, port in ARM_PORTS.items():
        try:
            if not port_open(port):
                raise OSError("port closed")
            c = portal.Client("%s:%d" % (YAMBOX_IP, port))
            d = int(c.num_dofs().result(timeout=3))
            arms[name] = (c, d)
            print("%s: follower on :%d, %d dofs" % (name, port, d), flush=True)
        except Exception:
            print("%s: no follower on :%d (not recorded)" % (name, port), flush=True)
    ndof = max([d for _, d in arms.values()] or [7])

    # ---- action / tactile taps ----
    hand_sock = open_udp(args.hand_udp)
    tact_sock = open_udp(args.tactile_udp)

    # ---- cameras ----
    # head        = the local D435 over the table -> head.mkv
    #               + its infrared PAIR -> ir_left/ir_right.mkv
    # wrist_left / wrist_right = the two USB cams ON THE YAMBOX, recorded there
    #               (MJPG stream-copy) and copied back at finalize.
    cam_procs = {}           # name -> local ffmpeg Popen recording that view
    remote_cams = {}         # name -> yambox v4l2 node being recorded there
    head = None
    ir_rec = None            # head-D435 infrared pair recorder thread
    prev_procs = []
    if not args.no_cams:
        os.makedirs(PREV_DIR, exist_ok=True)
        # hand-off: the idle previewer's grabbers hold the v4l2 nodes between
        # takes - kill them and wait for the devices to be released (the
        # cam_preview daemon itself stays up and resumes after this take)
        sh(["pkill", "-f", IDLE_MARKER])
        # the wrist idle grabbers are ssh pipelines whose REMOTE ffmpeg holds
        # the yambox v4l2 node - drop those too before start_yambox_cam opens it
        ssh("pkill -f %s 2>/dev/null; true" % IDLE_MARKER)
        for _ in range(50):
            if sh(["pgrep", "-f", IDLE_MARKER]).returncode != 0:
                break
            time.sleep(0.1)
        head = resolve_side_cam()   # the D435 colour node (now the HEAD view)
        if head:
            cam_procs["head"] = subprocess.Popen(
                ffmpeg_args(head, os.path.join(dest, "head.mkv"),
                            preview=os.path.join(PREV_DIR, "head.jpg")),
                stdout=open(os.path.join(dest, "head.log"), "w"),
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            # IR pair rides the same D435; start AFTER the colour ffmpeg
            ir_rec = IRRecorder(dest)
            ir_rec.start()
        else:
            print("WARNING: head cam (D435) not found", flush=True)
        for name, dev in yambox_wrist_cams().items():
            if not dev:
                print("WARNING: %s cam not found on yambox" % name, flush=True)
                continue
            if start_yambox_cam(name, dev):
                remote_cams[name] = dev
                print("%s: recording on yambox (%s)" % (name, dev), flush=True)
            else:
                print("WARNING: %s recorder failed on yambox (see %s/%s.log there)"
                      % (name, YAMBOX_TAKE_DIR, name), flush=True)

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__("flag", True))
    signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__("flag", True))

    ts, arm_rows, hand_rows, hand_ts, tact_rows, tact_ts = [], {k: [] for k in arms}, [], [], [], []
    period = 1.0 / args.fps
    t0 = time.time()
    next_status = 0.0
    while not stop["flag"]:
        tick = time.time()
        ts.append(tick)
        for name, (c, d) in arms.items():
            try:
                arm_rows[name].append(np.asarray(c.get_joint_pos().result(timeout=1),
                                                 dtype=np.float64))
            except Exception:
                arm_rows[name].append(np.full(d, np.nan))
        for msg in drain(hand_sock):
            if "hand" in msg:
                hand_rows.append(msg["hand"]); hand_ts.append(msg.get("t", tick))
        for msg in drain(tact_sock):
            if "f6" in msg:
                tact_rows.append(msg["f6"]); tact_ts.append(msg.get("t", tick))
        if tick > next_status:
            json.dump({"episode": n, "dir": dest, "t0": t0, "elapsed": tick - t0,
                       "frames": len(ts), "hand_msgs": len(hand_rows),
                       "arms": list(arms), "tactile_msgs": len(tact_rows)},
                      open(STATUS, "w"))
            next_status = tick + 1.0
        dt = period - (time.time() - tick)
        if dt > 0:
            time.sleep(dt)

    # ---- finalize ----
    print("stopping (%.1f s, %d ticks, %d hand actions) ..."
          % (time.time() - t0, len(ts), len(hand_rows)), flush=True)
    for p in cam_procs.values():
        p.send_signal(signal.SIGINT)
    if remote_cams:
        print("stopping yambox wrist cams + copying takes back ...", flush=True)
        stop_yambox_cams(dest, list(remote_cams))
    if ir_rec is not None:
        ir_rec.stop_flag = True
        ir_rec.join(timeout=15)
        print("IR pair: %d frames" % ir_rec.frames, flush=True)
    for p in prev_procs:
        try:
            p.terminate()
        except Exception:
            pass
    for f in ("head.jpg", "wrist_left.jpg", "wrist_right.jpg"):   # stale previews vanish
        try:
            os.remove(os.path.join(PREV_DIR, f))
        except OSError:
            pass
    with h5py.File(os.path.join(dest, "episode.h5"), "w") as f:
        f.attrs.update({"format": "zhi-teleop-v1", "fps": args.fps,
                        "created_unix": t0,
                        "reference": "T-Rex (arXiv:2606.17055) raw layout, "
                                     "LeRobot-v3-style keys",
                        "hand_joint_order": "Sharpa Wave 22-joint SDK order",
                        "hand_action_source": "sharpa_hand_driver --tee "
                                              "(post-clamp commanded targets)"})
        f.create_dataset("timestamps", data=np.asarray(ts))
        if ir_rec is not None and ir_rec.ts:
            # wall-clock stamp of each ir_left/ir_right.mkv frame (both share it)
            f.create_dataset("observation/ir_t", data=np.asarray(ir_rec.ts))
        g = f.create_group("observation/state")
        for name in ARM_PORTS:
            rows = arm_rows.get(name)
            g.create_dataset(name, data=np.vstack(rows) if rows
                             else np.full((len(ts), ndof), np.nan))
        a = f.create_group("action")
        a.create_dataset("hand", data=np.asarray(hand_rows, dtype=np.float64)
                         if hand_rows else np.zeros((0, 22)))
        a.create_dataset("hand_t", data=np.asarray(hand_ts))
        o = f["observation"]
        o.create_dataset("tactile_f6", data=np.asarray(tact_rows, dtype=np.float64)
                         if tact_rows else np.zeros((0, 5, 6)))
        o.create_dataset("tactile_t", data=np.asarray(tact_ts))
    for p in cam_procs.values():
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
    json.dump({"episode": n, "duration_s": time.time() - t0, "fps": args.fps,
               "frames": len(ts), "hand_actions": len(hand_rows),
               "tactile_msgs": len(tact_rows), "arms_recorded": list(arms),
               "cams": {"head": head,
                        "ir_pair": bool(ir_rec and ir_rec.frames),
                        **{k: "yambox:" + v for k, v in remote_cams.items()},
                        **{k: (p.args[p.args.index("-i") + 1] if "-i" in p.args else "?")
                           for k, p in cam_procs.items() if k != "head"}},
               "hand_udp": args.hand_udp, "tactile_udp": args.tactile_udp},
              open(os.path.join(dest, "meta.json"), "w"), indent=1)
    try:
        os.remove(STATUS)
    except OSError:
        pass
    print("episode %d saved -> %s" % (n, dest), flush=True)


if __name__ == "__main__":
    main()
