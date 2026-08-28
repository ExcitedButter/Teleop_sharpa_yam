#!/home/zhicao/Desktop/Zhi/Zhi/.venv/bin/python
"""Idle camera previewer: keeps the viz panel's camera strip alive between takes.

record_episode.py publishes /tmp/zhi_prev/*.jpg only WHILE a take runs (the
previews piggyback on the recording ffmpegs, so no device is opened twice).
This daemon fills the gap: whenever no recorder is running it holds one
low-rate grab-only ffmpeg per camera writing the same jpgs, so the strip in
viz_panel.py is live the whole time the panel is up.

Camera roster (matches record_episode.py since the 2026-08-25 re-choice):
  head        = the local D435 colour node (plain v4l2 grab)
  wrist_left / wrist_right = USB cams ON THE YAMBOX - grabbed remotely
    (ssh ffmpeg -f mjpeg to stdout) and written to the jpg by a local ffmpeg.

Device hand-off (the v4l2 nodes are exclusive-stream):
  - this daemon never (re)starts a grabber while record_episode.py is alive,
    and kills its own grabbers the moment one appears;
  - record_episode.py additionally pkills the argv marker "zhi_idle_preview"
    (locally AND on the yambox - the marker rides in every argv of the
    pipeline, remote ffmpeg included) and waits for the local grabbers to
    vanish before opening the devices, which closes the start-of-take race.

Spawned, healed and terminated by viz_panel.py like its other helpers.
"""
import os, signal, subprocess, sys, time

# realpath, not abspath: the panel spawns us through the ~/Desktop/Zhi/viz
# SYMLINK, whose parent has no record_episode.py (import crash-loop 2026-08-27)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import record_episode as rec   # single source of truth for the camera roster

MARKER = "zhi_idle_preview"    # unique argv token record_episode can pkill
CAMS = ("head", "wrist_left", "wrist_right")

procs = {}   # name -> grabber Popen


def recorder_running():
    return subprocess.run(["pgrep", "-f", r"record_episode\.py"],
                          capture_output=True).returncode == 0


def stop_all():
    # each grabber is a shell pipeline in its own session - kill the GROUP so
    # the local ffmpeg and (for wrists) the ssh die with the shell; the remote
    # ffmpeg then exits on its broken stdout pipe
    for p in procs.values():
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except Exception:
            pass
    for p in procs.values():
        try:
            p.wait(timeout=3)
        except Exception:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except Exception:
                pass
    procs.clear()


def grab_cmd(name, dev, out):
    """Latest-frame jpg pipeline for one camera. head = local YUYV grab;
    the wrist cams are yambox devices (MJPG UVC), so grab there over ssh and
    write the jpg locally. MARKER appears in every argv of the chain (remote
    ffmpeg included) so record_episode's local+remote pkill clears it all."""
    vf = "fps=4,scale=320:240"
    if name == "head":
        return ("ffmpeg -nostdin -y -loglevel error -f v4l2 -input_format yuyv422"
                " -video_size 640x480 -framerate 30 -i %s -vf %s -update 1"
                " -metadata comment=%s %s" % (dev, vf, MARKER, out))
    return ("ssh -o BatchMode=yes -o ConnectTimeout=4 %s"
            " 'ffmpeg -nostdin -loglevel error -f v4l2 -input_format mjpeg"
            " -video_size 640x480 -framerate 30 -i %s -vf %s"
            " -metadata comment=%s -f mjpeg -'"
            " | ffmpeg -nostdin -y -loglevel error -f mjpeg -i -"
            " -update 1 -metadata comment=%s %s"
            % (rec.YAMBOX, dev, vf, MARKER, MARKER, out))


def main():
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *a: (stop_all(), os._exit(0)))
    os.makedirs(rec.PREV_DIR, exist_ok=True)
    last_resolve = 0.0
    while True:
        if recorder_running():
            if procs:      # the recorder owns the cameras during a take
                stop_all()
            time.sleep(1)
            continue
        for name, p in list(procs.items()):   # reap dead grabbers -> respawn
            if p.poll() is not None:
                del procs[name]
        missing = [n for n in CAMS if n not in procs]
        if missing and time.time() - last_resolve > 10:   # unplug backoff
            last_resolve = time.time()
            devs = dict(rec.yambox_wrist_cams())
            devs["head"] = rec.resolve_side_cam()
            for name in missing:
                if devs.get(name):
                    procs[name] = subprocess.Popen(
                        grab_cmd(name, devs[name],
                                 os.path.join(rec.PREV_DIR, name + ".jpg")),
                        shell=True, start_new_session=True,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        stdin=subprocess.DEVNULL)
            if recorder_running():   # lost the start-of-take race - let go
                stop_all()
        time.sleep(1)


if __name__ == "__main__":
    main()
