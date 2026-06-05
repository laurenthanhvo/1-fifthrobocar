from pathlib import Path
import time
import cv2
import depthai as dai
import numpy as np


# ============================================================
# Settings
# ============================================================

PREVIEW_W = 300
PREVIEW_H = 300
FPS = 20

# MobileNet SSD model used by DepthAI examples.
# If this file does not exist, you need to copy/download it from depthai-python examples/models.
NN_PATH = str((Path(__file__).parent / "models" / "mobilenet-ssd_openvino_2021.4_5shave.blob").resolve())

# MobileNet SSD labels from DepthAI example.
LABEL_MAP = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
    "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
    "tvmonitor",
]

CONF_THRESHOLD = 0.45

# Depth range in millimeters.
DEPTH_LOWER_MM = 100
DEPTH_UPPER_MM = 5000

# Save output image every second.
SAVE_PATH = "spatial_size_debug.jpg"


def get_socket(name_old, name_new):
    """
    DepthAI renamed sockets in newer versions:
    RGB -> CAM_A, LEFT -> CAM_B, RIGHT -> CAM_C
    """
    if hasattr(dai.CameraBoardSocket, name_new):
        return getattr(dai.CameraBoardSocket, name_new)
    return getattr(dai.CameraBoardSocket, name_old)


def estimate_size_meters(bbox_px, z_mm, fx_px, fy_px):
    """
    Estimate real-world width/height from bbox pixels and depth.

    width_m  ≈ bbox_width_px  * z_m / fx_px
    height_m ≈ bbox_height_px * z_m / fy_px
    """
    x1, y1, x2, y2 = bbox_px
    bbox_w_px = max(1, x2 - x1)
    bbox_h_px = max(1, y2 - y1)

    z_m = z_mm / 1000.0

    width_m = bbox_w_px * z_m / fx_px
    height_m = bbox_h_px * z_m / fy_px

    return width_m, height_m


def frame_norm(frame, bbox):
    """
    Convert normalized detection bbox into pixel bbox.
    bbox = [xmin, ymin, xmax, ymax], each in 0..1
    """
    h, w = frame.shape[:2]

    x1 = int(np.clip(bbox[0], 0, 1) * w)
    y1 = int(np.clip(bbox[1], 0, 1) * h)
    x2 = int(np.clip(bbox[2], 0, 1) * w)
    y2 = int(np.clip(bbox[3], 0, 1) * h)

    return x1, y1, x2, y2


# ============================================================
# Check model
# ============================================================

if not Path(NN_PATH).exists():
    raise FileNotFoundError(
        f"Could not find model blob:\n{NN_PATH}\n\n"
        "Create a models folder and put mobilenet-ssd_openvino_2021.4_5shave.blob inside it."
    )


# ============================================================
# Build DepthAI pipeline
# ============================================================

pipeline = dai.Pipeline()

cam_rgb = pipeline.create(dai.node.ColorCamera)
mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatial_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_nn = pipeline.create(dai.node.XLinkOut)
xout_depth = pipeline.create(dai.node.XLinkOut)

xout_rgb.setStreamName("rgb")
xout_nn.setStreamName("detections")
xout_depth.setStreamName("depth")

rgb_socket = get_socket("RGB", "CAM_A")
left_socket = get_socket("LEFT", "CAM_B")
right_socket = get_socket("RIGHT", "CAM_C")

# RGB camera for neural network.
cam_rgb.setBoardSocket(rgb_socket)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
cam_rgb.setPreviewSize(PREVIEW_W, PREVIEW_H)
cam_rgb.setPreviewKeepAspectRatio(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
cam_rgb.setInterleaved(False)
cam_rgb.setFps(FPS)

# Mono stereo cameras for depth.
mono_left.setBoardSocket(left_socket)
mono_right.setBoardSocket(right_socket)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setFps(FPS)
mono_right.setFps(FPS)

# Stereo depth.
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Align depth to RGB if supported.
try:
    stereo.setDepthAlign(rgb_socket)
    stereo.setOutputSize(PREVIEW_W, PREVIEW_H)
except Exception:
    pass

# Spatial detection network.
spatial_nn.setBlobPath(NN_PATH)
spatial_nn.setConfidenceThreshold(CONF_THRESHOLD)
spatial_nn.input.setBlocking(False)
spatial_nn.setBoundingBoxScaleFactor(0.5)
spatial_nn.setDepthLowerThreshold(DEPTH_LOWER_MM)
spatial_nn.setDepthUpperThreshold(DEPTH_UPPER_MM)

# Links.
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

cam_rgb.preview.link(spatial_nn.input)
stereo.depth.link(spatial_nn.inputDepth)

cam_rgb.preview.link(xout_rgb.input)
spatial_nn.out.link(xout_nn.input)
stereo.depth.link(xout_depth.input)


# ============================================================
# Run
# ============================================================

with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue("rgb", maxSize=1, blocking=False)
    q_det = device.getOutputQueue("detections", maxSize=1, blocking=False)
    q_depth = device.getOutputQueue("depth", maxSize=1, blocking=False)

    # Camera intrinsics for width/height estimate.
    calib = device.readCalibration()
    K = np.array(
        calib.getCameraIntrinsics(rgb_socket, PREVIEW_W, PREVIEW_H),
        dtype=np.float32,
    )
    fx_px = float(K[0, 0])
    fy_px = float(K[1, 1])

    print("fx_px:", fx_px)
    print("fy_px:", fy_px)
    print("Saving debug image to:", SAVE_PATH)

    latest_frame = None
    latest_detections = []
    latest_depth = None
    last_save = 0.0

    while True:
        rgb_packet = q_rgb.tryGet()
        det_packet = q_det.tryGet()
        depth_packet = q_depth.tryGet()

        if rgb_packet is not None:
            latest_frame = rgb_packet.getCvFrame()

        if det_packet is not None:
            latest_detections = det_packet.detections

        if depth_packet is not None:
            latest_depth = depth_packet.getFrame()

        if latest_frame is None:
            time.sleep(0.01)
            continue

        frame = latest_frame.copy()

        for det in latest_detections:
            label_id = int(det.label)
            label = LABEL_MAP[label_id] if label_id < len(LABEL_MAP) else str(label_id)

            x1, y1, x2, y2 = frame_norm(
                frame,
                [det.xmin, det.ymin, det.xmax, det.ymax],
            )

            z_mm = float(det.spatialCoordinates.z)
            x_mm = float(det.spatialCoordinates.x)
            y_mm = float(det.spatialCoordinates.y)

            width_m, height_m = estimate_size_meters(
                (x1, y1, x2, y2),
                z_mm,
                fx_px,
                fy_px,
            )

            distance_m = z_mm / 1000.0

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

            text1 = f"{label} conf={det.confidence:.2f}"
            text2 = f"dist={distance_m:.2f}m x={x_mm/1000:.2f}m y={y_mm/1000:.2f}m"
            text3 = f"w={width_m:.2f}m h={height_m:.2f}m"

            cv2.putText(frame, text1, (x1, max(18, y1 - 36)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame, text2, (x1, max(18, y1 - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame, text3, (x1, max(18, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            print(
                f"{label}: distance={distance_m:.2f}m, "
                f"width={width_m:.2f}m, height={height_m:.2f}m, "
                f"x={x_mm/1000:.2f}m, y={y_mm/1000:.2f}m"
            )

        now = time.time()
        if now - last_save > 1.0:
            cv2.imwrite(SAVE_PATH, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            last_save = now

        time.sleep(0.01)
