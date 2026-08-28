# Zhi — bimanual teleoperation stack

Teleop for a bimanual rig: two **YAM arms** (i2rt, on the "yambox" follower
PC) with **Sharpa Wave hands**. The operator wears **Manus Quantum
Metagloves** (finger tracking → hand retargeting) and **HTC Vive trackers**
on the wrists (arm tracking → IK). A Tk panel (`Zhi/teleop_panel.py`)
starts/stops every piece and records episodes.

```
 ARMS  (vive tracker retargeting)
 ─────────────────────────────────
 Vive trackers ──SteamVR──▶ vive_wrist_stream.py ──UDP 9873/9874──▶ manus_arm_teleop.py
                            (60 Hz, both wrists      (measured axis map + ssik analytical IK)
                             per packet)                       │ joint targets
                                                               ▼
                                                 yambox followers  192.168.1.9
                                                 right :11333 (can0) · left :11334 (can1)

 HANDS  (sharpa / manus retargeting)
 ───────────────────────────────────
 Manus gloves ──2 USB dongles──▶ SharpaManusClient.out ──▶ retargeting optimizer
                                 (headless build)           (casadi, V4.0, in sharpa-manus-sdk)
                                                               │ ZMQ tcp://127.0.0.1:6668
                                                               ▼
                     sharpa_zmq_to_driver.py ──UDP 59201 (L) / 59202 (R)──▶ sharpa_hand_driver.py ×2
                     (joint_left→59201, straight)   (22 joint radians, JSON)      │ SharpaWaveSDK
                                                                                  ▼
                                                          Sharpa hands  192.168.10.10 (L) / .20 (R)

 RECORDING
 ─────────
 record_episode.py: arm states (30 Hz) + head cam (RealSense D435) + hand
 targets teed from the drivers (UDP 59250) → episodes/episode_NNNNNN/
 (HDF5 + mkv, LeRobot-v3-friendly — see Zhi/RECORD_FORMAT.md)
```

## Repo layout

| path | what |
|---|---|
| `Zhi/teleop_panel.py` | **main entry point** — Tk panel: per-device tiles (arm leaders/followers, Gloves→Retarget chain, wuji, viz), Record / Reset / Loose / Reset-CAN buttons, `--check` health report, `--selftest` |
| `Zhi/vive_wrist_stream.py` | SteamVR → UDP wrist stream (both trackers in every packet; singleton-guarded on port 9879) |
| `Zhi/manus_arm_teleop.py` | wrist stream → arm IK → follower. `--dry-run`, `--reset`, `--loose`, `--mirror-home`; prefers the **measured** axis map in `params/vive_arm_map.json`, IK is ssik analytical (`pip install ssik`) with DLS fallback |
| `Zhi/sharpa_zmq_to_driver.py` | retargeting optimizer output (ZMQ) → per-hand UDP |
| `Zhi/sharpa_hand_driver.py` | UDP 22-joint targets → Sharpa hand via SharpaWaveSDK; `--side`, `--listen`, `--tee` (recorder tap); retries until the hand appears, SIGINT homes + releases |
| `Zhi/sharpa_wrist_to_udp.py` | wrist stream helper (hermes original) |
| `Zhi/record_episode.py` | episode recorder (`--out`, `--fps`, `--hand-udp`, `--tactile-udp`, `--no-cams`) |
| `Zhi/RECORD_FORMAT.md` | episode format spec (`zhi-teleop-v1`) |
| `Zhi/calibrate_*.py` | calibration tools — see below |
| `Zhi/params/` | all calibration + config state (see below) |
| `Zhi/viz/` | live viz: `viz_panel.py` (MuJoCo mirror of real joint states), `cam_preview.py`, `hand_relay.py`, `bench/bench_scene.xml` |
| `Zhi/i2rt/` | copy of the i2rt repo (models + `combine_arm_and_gripper_xml`) so the IK/viz are self-contained |
| `Zhi/start_teleop.sh`, `Zhi/start_record.sh` | hermes-era session scripts (kept for reference; the panel is the current entry point) |
| `viz`, `episodes` (top level) | relative symlinks into `Zhi/` (hermes layout convention) |

**Not in git** (see `.gitignore`): `Zhi/.venv/` and `Zhi/sharpamanus-venv/`
(local virtualenvs), `Zhi/sharpa-manus-sdk/` (clone of
[sharpa-robotics/sharpa-manus-sdk](https://github.com/sharpa-robotics/sharpa-manus-sdk)
with a headless client patch — build with `SHARPA_HEADLESS=1`, it bundles
SharpaWaveSDK 4.6.6 + the retargeting optimizer + Manus calibration files),
and `Zhi/episodes/` (recorded data).

## `params/` — the state that makes it work

| file | meaning |
|---|---|
| `vive_trackers.json` | tracker serial → side (**left=LHR-F4DCB137, right=LHR-124A9764** — verified; do NOT trust x-position auto-assign) and `operator_yaw_deg` **pinned to 0** |
| `vive_arm_map.json` | **measured** 3×3 tracker→arm axis map per side (det +1), written by `calibrate_arm_axes.py`; optional `<side>_rot` wrist-rotation maps from `calibrate_vive_gui.py`. The teleop prefers this over all base-yaw/flip heuristics. (`.orig-backup` = hermes original, `.prev` = previous calibration) |
| `arm_home.json` | reset/home joint pose (written by `--mirror-home`, zeros if absent) |
| `init_state.json` | panel Reset target |
| `head_view.mkv` | reference clip of the correct head-cam framing |

## Running

```bash
cd Zhi
../Zhi/.venv/bin/python teleop_panel.py          # the panel (or just python3 teleop_panel.py)
python3 teleop_panel.py --check                  # health report: dongles, hands, yambox, docker, files
```

Typical arm-teleop session from the panel: start the followers → **Reset**
(arms go to home) → **Start Teleop** (countdown, then live tracking).
Keep a hand on the e-stop for the first runs.

Hands: **Gloves** tile (starts the headless Manus client + optimizer) →
**Retarget** tile (bridge + both hand drivers). The drivers wait until the
hands appear on the network.

Manual / debug equivalents:

```bash
# arms, no motion — sanity-check tracking end to end
python3 manus_arm_teleop.py --side right --dry-run

# hands, piece by piece
python3 sharpa_zmq_to_driver.py --print
python3 sharpa_hand_driver.py --side left  --listen 127.0.0.1:59201 --tee 127.0.0.1:59250
```

Recording: the panel's **Record** tile, or
`python3 record_episode.py` (episodes land in `episodes/`, format in
`RECORD_FORMAT.md`).

## Calibration

Order matters — do them top to bottom when setting up from scratch:

1. **Tracker identity** — put both trackers on, wiggle ONE wrist and watch
   which side moves in `viz` (or the teleop's dry-run prints). If crossed,
   swap the serials in `params/vive_trackers.json`. Never rely on the
   left-of/right-of auto-assignment.
2. **Arm axis map** (the important one) — per side, with the follower up:

   ```bash
   python3 calibrate_arm_axes.py --side left    # then --side right
   ```

   It jogs the arm along its own axes (`--delta` metres per jog), reads how
   the tracker moved, and writes the measured 3×3 map into
   `params/vive_arm_map.json`. A good map prints `det=+1.00` when the
   teleop starts (`using MEASURED arm-axis map`).
3. **Wrist rotation map** (optional refinement) — `calibrate_vive_gui.py`
   wrist-rotation mode adds `<side>_rot` entries so rotations use their own
   measured map instead of conjugating through the position map.
   (`calibrate_vive_yaw.py`, `calibrate_vive_auto.py`,
   `calibrate_vive_by_hand.py`, `calibrate_arm_rot.py` are the older/manual
   variants; `operator_yaw_deg` stays 0.)
4. **Home pose** — pose one arm where you want home, then
   `python3 manus_arm_teleop.py --side <s> --mirror-home` writes
   `params/arm_home.json` (mirrors to the other side,
   `MIRROR_SIGN=[-1,1,1,-1,1,-1]`).
5. **Glove (finger) calibration** — run the Manus `CalibrationGUI` from the
   sharpa-manus-sdk clone and save per-operator `Calibration_*.mcal` files
   next to the client binary. The repo's generic `.mcal` works but personal
   calibration noticeably improves finger retargeting. Restart the client
   after changing them.

## Hardware map (this rig)

- Local PC `192.168.1.3` (`enp68s0`); secondary addr `192.168.10.240/24` on
  the same NIC for the hands' tactile stream; UFW allow `192.168.10.0/24`.
- Yambox follower `yambox@192.168.1.9`, repo at `~/i2rt`, followers on
  `:11333` (right/can0) and `:11334` (left/can1). If the USB-CAN adapters
  come back from a reboot stuck in DFU mode (`0483:df11`): usbreset +
  `dfu-util`, then `~/i2rt/scripts/reset_all_can.sh` (panel: Reset CAN).
- Sharpa hands: left `192.168.10.10`, right `192.168.10.20` (SDK discovery
  on port 54321).
- Manus dongles: USB VID 3325, udev rule `70-manus-dongle.rules` (MODE 0666).
- Head cam: RealSense D435 serial `327343020514`; wrist cams plug into the
  yambox.
