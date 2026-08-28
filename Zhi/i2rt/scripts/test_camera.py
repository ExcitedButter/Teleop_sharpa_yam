"""Show a live view from every connected camera. Press 'q' to quit."""

import glob

import cv2


def find_working_cameras() -> list[str]:
    working = []
    for path in sorted(glob.glob("/dev/video*")):
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                working.append(path)
        cap.release()
    return working


def main() -> None:
    devices = find_working_cameras()
    print(f"found {len(devices)} working camera(s): {devices}")
    if not devices:
        print("no working cameras found")
        return

    caps = {path: cv2.VideoCapture(path, cv2.CAP_V4L2) for path in devices}

    try:
        while True:
            for path, cap in caps.items():
                ok, frame = cap.read()
                if ok:
                    cv2.imshow(path, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        for cap in caps.values():
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
