# params — initial-state snapshot

Captured for the current teleop/recording initial state.

## Contents
- **init_state.json** — captured joint positions:
  - `yam.left` / `yam.right` — 6-DOF follower arm joint positions (radians)
  - `sharpa.left` / `sharpa.right` — 22-joint hand pose (radians, Sharpa order)
- **head_view.mkv** — short clip from the head/ego camera (on the yambox, usb 0:3;
  yuyv422 640x480 @30fps)
- **head_cam_intrinsics.* / head_cam_extrinsics.*** — head-camera calibration
  (TO ADD — provide the file path and I'll copy it in)

## Head camera
Connected to the **yambox** (192.168.1.9), device
`/dev/v4l/by-path/pci-0000:c6:00.3-usb-0:3:1.0-video-index0`.
