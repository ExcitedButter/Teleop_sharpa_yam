# Zhi teleop episode format (`zhi-teleop-v1`)

Recorded by `record_episode.py` (panel "Record" tile, or run it directly).
Design references:
- **T-Rex** (arXiv:2606.17055) — 100 h tactile teleop dataset collected on the
  SAME Sharpa Wave hands; raw layout = one dir per episode with one HDF5 +
  one video file per camera, later packed to LeRobot v3.
- **LeRobotDataset v3** — parquet + mp4 shards with `observation.*` /
  `action` keys; our h5 keys mirror theirs 1:1 so packing is mechanical.

## Layout

```
~/Desktop/Zhi/episodes/episode_000042/
    episode.h5      synchronized low-dim data
    head.mkv        yambox head cam   640x480@30  (remux to mp4 when packing)
    side.mkv        hermes side cam   640x480@30
    meta.json       duration, frames, which arms/cams were live, ports
```

## episode.h5

| dataset                        | shape    | meaning |
|--------------------------------|----------|---------|
| `/timestamps`                  | [N]      | unix time of each 30 Hz tick |
| `/observation/state/arm_left`  | [N,6]    | follower joint pos (rad), NaN-padded when that side is down |
| `/observation/state/arm_right` | [N,6]    | " |
| `/action/hand` + `/action/hand_t` | [M,22]+[M] | TRUE commanded hand targets (rad, Sharpa 22-joint order), post-clamp, teed by `sharpa_hand_driver --tee` at ~60 Hz with own timestamps |
| `/observation/tactile_f6` + `/observation/tactile_t` | [K,5,6]+[K] | per-fingertip 6-axis wrench (T-Rex's `tactile_f6`); reserved — written when a publisher sends `{"t":..,"f6":[[..6]..]}` JSON to udp 127.0.0.1:59300 |

## Mapping to LeRobot v3 / T-Rex keys

- `observation.images.head` / `observation.images.side`  ← the mkv files
- `observation.state`  ← concat arm state (+ hand state when readback works)
- `action` (abs targets, T-Rex `action_abs` style) ← `/action/hand` resampled
  to the 30 Hz grid (nearest ≤ t) + arm targets
- `observation.tactile_f6` ← as-is (T-Rex uses [10,6] bimanual; we use [5,6])

## Known gaps / next steps

- **Arm action**: only follower *state* is queryable without touching
  minimum_gello; derive `action_abs[t] ≈ state[t+1]` offline, or add a tee to
  the leader for true targets.
- **Hand state & tactile**: blocked by hand firmware < SDK minimum (state
  packets unparseable, "Invalid Pre-Header"). After a firmware upgrade via
  SharpaPilot, publish tactile wrenches to udp 59300 — the recorder and the
  format already accept them.
- Timestamps are wall-clock on both video (`-use_wallclock_as_timestamps 1`)
  and h5, so alignment at pack time is a nearest-timestamp join.
