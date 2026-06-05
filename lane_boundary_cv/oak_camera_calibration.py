#!/usr/bin/env python3

"""
OAK-D camera calibration helper for DonkeyCar lane_boundary_cv.

This script has three modes:

1. capture
   Captures checkerboard images from the OAK-D camera.

2. calibrate
   Uses saved checkerboard images to estimate camera_matrix and dist_coeffs.

3. test
   Loads camera_calibration.npz and shows raw versus undistorted live video.

Your checkerboard:
    9 x 12 total squares
    therefore 8 x 11 inner corners

Typical usage:

    python oak_camera_calibration.py capture
    python oak_camera_calibration.py calibrate
    python oak_camera_calibration.py test

If you are headless and cannot open a preview window:

    python oak_camera_calibration.py capture --headless --auto-save --count 50
"""

import argparse
import glob
import os
import time
from pathlib import Path
from typing import Tuple

import cv2
import depthai as dai
import numpy as np


DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 800
DEFAULT_FPS = 20

DEFAULT_IMAGE_DIR = "calib_images"
DEFAULT_OUTPUT_FILE = "camera_calibration.npz"

# Your board has 9 x 12 total squares, so it has 8 x 11 inner corners.
# OpenCV pattern size is (columns, rows).
CHECKERBOARD_PATTERNS = [
    (8, 11),
    (11, 8),
]


def has_gui_display() -> bool:
    if os.name == "nt":
        return True

    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def set_supported_sensor_resolution(cam) -> None:
    sensor_resolution = getattr(
        dai.ColorCameraProperties.SensorResolution,
        "THE_800_P",
        None,
    )

    if sensor_resolution is None:
        sensor_resolution = getattr(
            dai.ColorCameraProperties.SensorResolution,
            "THE_720_P",
            None,
        )

    if sensor_resolution is not None:
        cam.setResolution(sensor_resolution)


def make_oak_pipeline(width: int, height: int, fps: int) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    set_supported_sensor_resolution(cam)
    cam.setPreviewSize(width, height)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(fps)

    # The sensor mode avoids OV9782 resolution warnings.
    # The preview size above still controls the calibration image size.
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("preview")
    cam.preview.link(xout.input)

    return pipeline


def capture_images(args: argparse.Namespace) -> None:
    if not args.headless and not has_gui_display():
        args.headless = True
        print("No graphical display detected. Running capture in headless mode.")

    if args.headless and not args.auto_save:
        args.auto_save = True
        print("Headless mode has no keyboard controls. Enabling auto-save.")

    output_dir = Path(args.image_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    pipeline = make_oak_pipeline(args.width, args.height, args.fps)

    print()
    print("Starting OAK-D calibration image capture.")
    print(f"Saving images to: {output_dir.resolve()}")
    print(f"Frame size: {args.width} x {args.height}")
    print()
    if args.headless:
        print("Headless capture:")
        print(f"  saving up to {args.count} images automatically")
        print("  press Ctrl+C to stop early")
        if not args.save_even_if_not_found:
            print("  only saving frames where a checkerboard is detected")
    else:
        print("Controls:")
        print("  SPACE: save current frame")
        print("  q: quit")
    print()
    print("Move the checkerboard around:")
    print("  center, left, right, top, bottom, tilted, close, far")
    print()

    last_auto_save = 0.0
    saved_count = 0

    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue(name="preview", maxSize=4, blocking=False)

        while True:
            packet = queue.get()
            frame = packet.getCvFrame()

            display = frame.copy()

            found_any = False
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            for pattern in CHECKERBOARD_PATTERNS:
                found, corners = find_checkerboard(gray, pattern)
                if found:
                    found_any = True
                    cv2.drawChessboardCorners(display, pattern, corners, found)
                    cv2.putText(
                        display,
                        f"checkerboard found: {pattern}",
                        (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                    break

            if not found_any:
                cv2.putText(
                    display,
                    "checkerboard not found",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )

            should_save = False

            if args.auto_save:
                now = time.time()
                if saved_count < args.count and now - last_auto_save >= args.interval:
                    should_save = found_any or args.save_even_if_not_found
                    last_auto_save = now

            key = -1
            if not args.headless:
                cv2.imshow("OAK-D calibration capture", display)
                key = cv2.waitKey(1) & 0xFF

                if key == ord(" "):
                    should_save = True

                if key == ord("q"):
                    break

            if should_save:
                filename = output_dir / f"calib_{saved_count:03d}.png"
                cv2.imwrite(str(filename), frame)
                print(f"Saved {filename}")
                saved_count += 1

            if args.auto_save and saved_count >= args.count:
                print(f"Captured requested {args.count} images.")
                break

    if not args.headless:
        cv2.destroyAllWindows()

    print()
    print(f"Done. Saved {saved_count} images.")


def find_checkerboard(gray: np.ndarray, pattern: Tuple[int, int]):
    # Try the newer OpenCV checkerboard detector first.
    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            found, corners = cv2.findChessboardCornersSB(
                gray,
                pattern,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                return True, corners
        except cv2.error:
            pass

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )

    found, corners = cv2.findChessboardCorners(gray, pattern, flags)

    if not found:
        return False, None

    corners = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        ),
    )

    return True, corners


def make_object_points(pattern: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = pattern

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size

    return objp


def calibrate_camera(args: argparse.Namespace) -> None:
    image_paths = sorted(glob.glob(os.path.join(args.image_dir, "*.png")))

    if not image_paths:
        raise RuntimeError(f"No PNG images found in {args.image_dir}")

    debug_dir = Path(args.debug_dir)
    debug_dir.mkdir(parents=True, exist_ok=True)

    objpoints = []
    imgpoints = []
    used_images = []
    image_size = None

    print()
    print("Starting calibration.")
    print(f"Looking for images in: {Path(args.image_dir).resolve()}")
    print(f"Trying checkerboard inner corner patterns: {CHECKERBOARD_PATTERNS}")
    print()

    for path in image_paths:
        image = cv2.imread(path)

        if image is None:
            print(f"Could not read {path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = gray.shape[::-1]

        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            print(f"Skipped {path}, image size {current_size} does not match {image_size}")
            continue

        found = False
        chosen_pattern = None
        chosen_corners = None

        for pattern in CHECKERBOARD_PATTERNS:
            ok, corners = find_checkerboard(gray, pattern)
            if ok:
                found = True
                chosen_pattern = pattern
                chosen_corners = corners
                break

        if not found:
            print(f"Skipped {path}, checkerboard not found")
            continue

        objp = make_object_points(chosen_pattern, args.square_size)

        objpoints.append(objp)
        imgpoints.append(chosen_corners)
        used_images.append(path)

        annotated = image.copy()
        cv2.drawChessboardCorners(annotated, chosen_pattern, chosen_corners, True)

        debug_name = debug_dir / Path(path).name
        cv2.imwrite(str(debug_name), annotated)

        print(f"Used {path} with pattern {chosen_pattern}")

    print()
    print(f"Usable calibration images: {len(objpoints)}")

    if len(objpoints) < args.min_images:
        raise RuntimeError(
            f"Only found checkerboard in {len(objpoints)} images. "
            f"Need at least {args.min_images}. Capture more images."
        )

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    mean_error = compute_reprojection_error(
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    np.savez(
        args.output,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_width=image_size[0],
        image_height=image_size[1],
        rms_error=rms,
        mean_reprojection_error=mean_error,
        square_size=args.square_size,
        used_images=np.array(used_images),
    )

    print()
    print("Calibration complete.")
    print(f"Saved: {Path(args.output).resolve()}")
    print()
    print("Image size:")
    print(f"  width:  {image_size[0]}")
    print(f"  height: {image_size[1]}")
    print()
    print("RMS calibration error:")
    print(f"  {rms}")
    print()
    print("Mean reprojection error:")
    print(f"  {mean_error}")
    print()
    print("Camera matrix:")
    print(camera_matrix)
    print()
    print("Distortion coefficients:")
    print(dist_coeffs)
    print()
    print(f"Annotated checkerboard detections saved to: {debug_dir.resolve()}")


def compute_reprojection_error(
    objpoints,
    imgpoints,
    rvecs,
    tvecs,
    camera_matrix,
    dist_coeffs,
) -> float:
    total_error = 0.0
    total_points = 0

    for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            objp,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )

        error = cv2.norm(imgp, projected, cv2.NORM_L2)
        total_error += error * error
        total_points += len(objp)

    return float(np.sqrt(total_error / total_points))


def test_undistortion(args: argparse.Namespace) -> None:
    if not has_gui_display():
        raise RuntimeError(
            "test mode needs a graphical display because it uses cv2.imshow. "
            "Run it from the Jetson desktop, enable X forwarding, or use "
            "capture --headless on display-less sessions."
        )

    data = np.load(args.output)

    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    pipeline = make_oak_pipeline(args.width, args.height, args.fps)

    print()
    print("Starting live undistortion test.")
    print("Press q to quit.")
    print()

    map1 = None
    map2 = None
    last_shape = None

    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue(name="preview", maxSize=4, blocking=False)

        while True:
            packet = queue.get()
            frame = packet.getCvFrame()

            h, w = frame.shape[:2]

            if last_shape != (h, w):
                new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                    camera_matrix,
                    dist_coeffs,
                    (w, h),
                    args.alpha,
                    (w, h),
                )

                map1, map2 = cv2.initUndistortRectifyMap(
                    camera_matrix,
                    dist_coeffs,
                    None,
                    new_camera_matrix,
                    (w, h),
                    cv2.CV_16SC2,
                )

                last_shape = (h, w)
                print(f"Initialized undistortion maps for {w} x {h}")

            undistorted = cv2.remap(
                frame,
                map1,
                map2,
                interpolation=cv2.INTER_LINEAR,
            )

            combined = np.hstack([frame, undistorted])

            cv2.putText(
                combined,
                "raw",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                combined,
                "undistorted",
                (w + 15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Raw left, undistorted right", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and calibrate OAK-D camera images for DonkeyCar."
    )

    subparsers = parser.add_subparsers(dest="mode")

    capture = subparsers.add_parser("capture")
    capture.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    capture.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    capture.add_argument("--fps", type=int, default=DEFAULT_FPS)
    capture.add_argument("--image-dir", default=DEFAULT_IMAGE_DIR)
    capture.add_argument("--headless", action="store_true")
    capture.add_argument("--auto-save", action="store_true")
    capture.add_argument("--save-even-if-not-found", action="store_true")
    capture.add_argument("--count", type=int, default=50)
    capture.add_argument("--interval", type=float, default=1.0)

    calibrate = subparsers.add_parser("calibrate")
    calibrate.add_argument("--image-dir", default=DEFAULT_IMAGE_DIR)
    calibrate.add_argument("--output", default=DEFAULT_OUTPUT_FILE)
    calibrate.add_argument("--debug-dir", default="calib_debug")
    calibrate.add_argument("--square-size", type=float, default=1.0)
    calibrate.add_argument("--min-images", type=int, default=12)

    test = subparsers.add_parser("test")
    test.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    test.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    test.add_argument("--fps", type=int, default=DEFAULT_FPS)
    test.add_argument("--output", default=DEFAULT_OUTPUT_FILE)
    test.add_argument("--alpha", type=float, default=0.0)

    args = parser.parse_args()
    if args.mode is None:
        parser.error("mode is required: choose capture, calibrate, or test")
    return args


def main() -> None:
    args = parse_args()

    if args.mode == "capture":
        capture_images(args)
    elif args.mode == "calibrate":
        calibrate_camera(args)
    elif args.mode == "test":
        test_undistortion(args)
    else:
        raise ValueError(f"Unknown mode: {args.mode}")


if __name__ == "__main__":
    main()
