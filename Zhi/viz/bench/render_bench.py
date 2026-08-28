"""Render the empty bench: two YAM arms, two sharpa hands, nothing on the table.

    conda activate <any env with mujoco>=3.4, numpy, imageio>
    python render_bench.py

Everything is configured by the constants below — there are no command-line
arguments. Outputs land in `reference/`.

WHAT YOU ARE LOOKING AT
    bench_cam        the lens experiments/08-14-2026/bottle's stage-4
                     composite (results/inpainting_bottle.gif) is rendered
                     through: the clip's own focal / height / pitch, aimed so
                     the two mounts sit at the very bottom of frame.
    photo_bench_cam  the same lens standing where the bench's own camera
                     stood when the clip was shot.
    overview         a free 3/4 orbit that shows the whole rig, which neither
                     of the other two do — at 30.8 deg fovy the arms are
                     mostly below the picture.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")       # "glfw" if you have a display
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import numpy as np
import mujoco

# ---- EDIT THESE, not the code --------------------------------------------
HERE = Path(__file__).resolve().parent
SCENE = HERE / "bench_scene.xml"
OUT = HERE / "reference"
WIDTH, HEIGHT = 640, 480          # the clip's own frame size; bench_cam's
#                                   fovy is only correct at this aspect ratio
OVERVIEW_WH = (960, 720)
SETTLE_S = 0.0                    # >0 holds the home pose under gravity for
#                                   this long before rendering. The arm->hand
#                                   weld is POSITION-ONLY and compliant, so
#                                   the hands sag ~19 mm in the first second:
#                                   the home pose is a COMMANDED state, not an
#                                   equilibrium. Leave at 0 for the pose as
#                                   published; the delivered videos replay
#                                   commanded states the same way.
MAKE_VIDEO = False                # a 4 s hold from bench_cam, for a sanity look
VIDEO_FPS = 30


def load():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    return model, data


def hold(model, data, seconds):
    """Drive every position servo to the home pose and let physics run."""
    for a in range(model.nu):
        j = int(model.actuator_trnid[a, 0])
        data.ctrl[a] = data.qpos[int(model.jnt_qposadr[j])]
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def overview_camera(model):
    """A free orbit that actually shows the rig — both mounts, both arms."""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (0.0, 0.42, 0.05)
    cam.distance = 1.50
    cam.azimuth = 120.0                # over the operator's left shoulder
    cam.elevation = -26.0
    model.vis.global_.fovy = 45.0
    return cam


def main():
    OUT.mkdir(exist_ok=True)
    import imageio.v2 as iio
    model, data = load()
    sheet = json.loads((HERE / "rig.json").read_text())
    print(f"{SCENE.name}: {model.nbody} bodies, {model.ngeom} geoms, "
          f"nq {model.nq}, nu {model.nu}, {model.neq} equality constraints")
    print("  rig frame: origin = base midpoint on the tabletop, "
          "+x -> right base, +y -> into the work, +z up, tabletop z = 0")
    for s in ("right", "left"):
        b = sheet["arms"]["bases_rig"][s]
        w = sheet["home_pose"]["wrist_rig"][s]
        print(f"  {s:5s} base {np.round(b['pos_m'], 3)} yaw "
              f"{b['yaw_deg_about_z']:+.1f} deg   home wrist "
              f"{np.round(w['pos_m'], 3)}")
    if SETTLE_S > 0:
        hold(model, data, SETTLE_S)
        mujoco.mj_forward(model, data)

    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as r:
        for name in ("bench_cam", "photo_bench_cam"):
            r.update_scene(data, camera=name)
            iio.imwrite(OUT / f"{name}_home.png", r.render())
            print("  wrote", OUT / f"{name}_home.png")
    with mujoco.Renderer(model, height=OVERVIEW_WH[1],
                         width=OVERVIEW_WH[0]) as r:
        r.update_scene(data, camera=overview_camera(model))
        iio.imwrite(OUT / "overview_home.png", r.render())
        print("  wrote", OUT / "overview_home.png")

    if MAKE_VIDEO:
        model, data = load()
        for a in range(model.nu):
            j = int(model.actuator_trnid[a, 0])
            data.ctrl[a] = data.qpos[int(model.jnt_qposadr[j])]
        frames = []
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as r:
            for _ in range(4 * VIDEO_FPS):
                for _ in range(int(1.0 / VIDEO_FPS / model.opt.timestep)):
                    mujoco.mj_step(model, data)
                r.update_scene(data, camera="bench_cam")
                frames.append(r.render())
        iio.mimwrite(OUT / "bench_hold.mp4", frames, fps=VIDEO_FPS,
                     quality=8)
        print("  wrote", OUT / "bench_hold.mp4")


if __name__ == "__main__":
    main()
