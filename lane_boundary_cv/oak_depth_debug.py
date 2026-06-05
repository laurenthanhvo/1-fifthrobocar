# oak_depth_debug.py
import time
import math
import cv2
import depthai as dai
import numpy as np


IMAGE_W = 640
IMAGE_H = 400

# Detection range in millimeters.
MIN_DEPTH_MM = 150
MAX_DEPTH_MM = 2000

# Stop threshold.
# 1 foot = about 305 mm.
# If camera is behind the front bumper, add that offset.
STOP_DISTANCE_MM = 305

# Ignore tiny blobs.
MIN_OBSTACLE_AREA = 500


def make_depth_heatmap(depth_mm, min_mm=150, max_mm=2000):
    """
    Convert depth map into a color heatmap.
    Closer objects become warmer/brighter.
    """
    depth = depth_mm.copy()

    # Invalid depth is usually 0. Push invalid values far away visually.
    depth[depth == 0] = max_mm

    depth = np.clip(depth, min_mm, max_mm)

    # Normalize so close = high intensity.
    depth_norm = ((max_mm - depth) / float(max_mm - min_mm) * 255.0)
    depth_norm = np.clip(depth_norm, 0, 255).astype(np.uint8)

    heatmap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
    return heatmap


def detect_depth_obstacle(depth_mm):
    """
    Detect nearby obstacle from depth alone.
    This is not using color. It looks for connected regions
    within a certain distance range.
    """
    valid = (
        (depth_mm > MIN_DEPTH_MM)
        & (depth_mm < MAX_DEPTH_MM)
    ).astype(np.uint8) * 255

    kernel = np.ones((5, 5), np.uint8)
    valid = cv2.morphologyEx(valid, cv2.MORPH_OPEN, kernel)
    valid = cv2.morphologyEx(valid, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        valid,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if len(contours) == 0:
        return None

    candidates = []

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_OBSTACLE_AREA:
            continue

        x, y, w, h = cv2.boundingRect(c)

        roi_depth = depth_mm[y:y+h, x:x+w]
        roi_valid = roi_depth[(roi_depth > MIN_DEPTH_MM) & (roi_depth < MAX_DEPTH_MM)]

        if roi_valid.size == 0:
            continue

        # Median is more stable than min.
        dist_mm = float(np.median(roi_valid))

        candidates.append({
            "bbox": (x, y, w, h),
            "area": area,
            "distance_mm": dist_mm,
        })

    if not candidates:
        return None

    # Pick closest obstacle.
    candidates.sort(key=lambda d: d["distance_mm"])
    return candidates[0]


def estimate_height_from_bbox_px(bbox_h_px, distance_mm, fy_px):
    """
    Approximate object height from bbox pixel height and depth.

    height_m ≈ pixel_height * distance_m / fy

    This is approximate. It assumes the bbox covers the physical height
    of the object and the object is roughly vertical.
    """
    if fy_px is None or fy_px <= 0:
        return None

    distance_m = distance_mm / 1000.0
    height_m = (bbox_h_px * distance_m) / fy_px
    return height_m


pipeline = dai.Pipeline()

cam_rgb = pipeline.createColorCamera()
mono_left = pipeline.createMonoCamera()
mono_right = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

xout_rgb = pipeline.createXLinkOut()
xout_depth = pipeline.createXLinkOut()

xout_rgb.setStreamName("rgb")
xout_depth.setStreamName("depth")

# RGB camera.
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
cam_rgb.setPreviewSize(IMAGE_W, IMAGE_H)
cam_rgb.setPreviewKeepAspectRatio(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
cam_rgb.setInterleaved(False)
cam_rgb.setFps(20)

# Stereo mono cameras.
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setFps(20)
mono_right.setFps(20)

# Stereo depth.
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

try:
    # Align depth to the RGB camera frame if supported by your DepthAI version.
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(IMAGE_W, IMAGE_H)
except Exception:
    pass

mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)
cam_rgb.preview.link(xout_rgb.input)
stereo.depth.link(xout_depth.input)

with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
    q_depth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)

    # Try to get camera intrinsics for height estimate.
    fy_px = None
    try:
        calib = device.readCalibration()
        intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, IMAGE_W, IMAGE_H)
        fy_px = float(intr[1][1])
        print("fy_px:", fy_px)
    except Exception as e:
        print("Could not read RGB intrinsics; height estimate disabled:", e)

    last_save = 0.0

    while True:
        rgb_packet = q_rgb.tryGet()
        depth_packet = q_depth.tryGet()

        if rgb_packet is None or depth_packet is None:
            time.sleep(0.01)
            continue

        rgb = rgb_packet.getCvFrame()
        depth_mm = depth_packet.getFrame()

        # Make sure depth size matches RGB size.
        if depth_mm.shape[:2] != rgb.shape[:2]:
            depth_mm = cv2.resize(
                depth_mm,
                (rgb.shape[1], rgb.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        heatmap = make_depth_heatmap(depth_mm, MIN_DEPTH_MM, MAX_DEPTH_MM)
        overlay = cv2.addWeighted(rgb, 0.55, heatmap, 0.45, 0)

        obs = detect_depth_obstacle(depth_mm)

        if obs is not None:
            x, y, w, h = obs["bbox"]
            dist_mm = obs["distance_mm"]
            height_m = estimate_height_from_bbox_px(h, dist_mm, fy_px)

            stop = dist_mm <= STOP_DISTANCE_MM

            cv2.rectangle(overlay, (x, y), (x + w, y + h), (255, 255, 255), 2)

            label = f"OBSTACLE {dist_mm/1000.0:.2f}m"
            if height_m is not None:
                label += f" h={height_m:.2f}m"
            if stop:
                label += " STOP"

            cv2.putText(
                overlay,
                label,
                (x, max(25, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            print(label)

        # Save debug frames once per second so you can view them over VS Code/SSH.
        now = time.time()
        if now - last_save > 1.0:
            cv2.imwrite("depth_debug_heatmap.jpg", cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
            cv2.imwrite("depth_raw_heatmap.jpg", heatmap)
            last_save = now


        # Headless mode:
        # Do not call cv2.imshow() over SSH because Qt/xcb will crash.
        # View depth_debug_heatmap.jpg and depth_raw_heatmap.jpg instead.
