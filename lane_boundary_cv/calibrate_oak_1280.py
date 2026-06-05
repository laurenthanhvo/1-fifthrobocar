import argparse
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np


def make_pipeline(width, height, fps):
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    cam.setPreviewSize(width, height)
    cam.setPreviewKeepAspectRatio(True)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setInterleaved(False)
    cam.setFps(fps)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    return pipeline


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--fps", type=int, default=10)

    # INNER corners, not squares.
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--rows", type=int, required=True)

    # Square size in meters. Example: 0.025 for 25 mm squares.
    ap.add_argument("--square", type=float, required=True)

    ap.add_argument("--samples", type=int, default=35)
    ap.add_argument("--out", default="camera_calibration_1280x800.npz")
    ap.add_argument("--save-dir", default="calib_1280_samples")
    args = ap.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(exist_ok=True)

    pattern_size = (args.cols, args.rows)

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square

    objpoints = []
    imgpoints = []

    pipeline = make_pipeline(args.width, args.height, args.fps)

    print("Starting OAK-D calibration capture.")
    print(f"Resolution: {args.width}x{args.height}")
    print(f"Checkerboard inner corners: {args.cols}x{args.rows}")
    print("Move/tilt the checkerboard around. The script auto-saves good detections.")
    print("Press Ctrl+C when done, or wait until target samples are reached.")

    last_save = 0
    sample_idx = 0

    with dai.Device(pipeline) as device:
        q = device.getOutputQueue("rgb", maxSize=1, blocking=False)

        try:
            while sample_idx < args.samples:
                pkt = q.tryGet()
                if pkt is None:
                    time.sleep(0.02)
                    continue

                frame = pkt.getCvFrame()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                ok, corners = cv2.findChessboardCorners(
                    gray,
                    pattern_size,
                    flags=cv2.CALIB_CB_ADAPTIVE_THRESH
                        + cv2.CALIB_CB_NORMALIZE_IMAGE
                        + cv2.CALIB_CB_FAST_CHECK,
                )

                now = time.time()

                if ok and now - last_save > 0.8:
                    corners_refined = cv2.cornerSubPix(
                        gray,
                        corners,
                        (11, 11),
                        (-1, -1),
                        criteria=(
                            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            30,
                            0.001,
                        ),
                    )

                    objpoints.append(objp.copy())
                    imgpoints.append(corners_refined)

                    vis = frame.copy()
                    cv2.drawChessboardCorners(vis, pattern_size, corners_refined, ok)

                    fname = save_dir / f"calib_{sample_idx:03d}.jpg"
                    cv2.imwrite(str(fname), vis)

                    sample_idx += 1
                    last_save = now

                    print(f"Saved sample {sample_idx}/{args.samples}: {fname}")

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("Stopped by user.")

    if len(objpoints) < 10:
        raise RuntimeError(f"Only got {len(objpoints)} samples. Need at least 10, ideally 25-40.")

    print(f"Calibrating with {len(objpoints)} samples...")

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        (args.width, args.height),
        None,
        None,
    )

    np.savez(
        args.out,
        camera_matrix=K,
        dist_coeffs=D,
        image_width=np.array(args.width),
        image_height=np.array(args.height),
        rms=np.array(rms),
    )

    print("\nSaved:", args.out)
    print("RMS reprojection error:", rms)
    print("Camera matrix K:")
    print(K)
    print("Distortion coefficients:")
    print(D.ravel())
    print("\nUse these in myconfig.py:")
    print(f'OAK_CALIBRATION_FILE = "/home/jetson/projects/mycars/lane_boundary_cv/{args.out}"')
    print(f"CAMERA_FX_PX = {K[0,0]:.6f}")
    print(f"CAMERA_FY_PX = {K[1,1]:.6f}")


if __name__ == "__main__":
    main()
