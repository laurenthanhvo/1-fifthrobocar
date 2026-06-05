import os


# ============================================================
# File paths
# ============================================================

CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, "data")


# ============================================================
# Vehicle loop
# ============================================================

DRIVE_LOOP_HZ = 20
MAX_LOOPS = None


# ============================================================
# Camera
# ============================================================

CAMERA_TYPE = "OAKD"
CAMERA_INDEX = 0

IMAGE_W = 640
IMAGE_H = 440
IMAGE_DEPTH = 3
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

CAMERA_VFLIP = False
CAMERA_HFLIP = False
BGR2RGB = False

OAK_UNDISTORT = False
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")
OAK_UNDISTORT_ALPHA = 0.1


# ============================================================
# Web UI
# ============================================================

WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))
WEB_INIT_MODE = "user"
OVERLAY_IMAGE = True


# ============================================================
# Custom CV autopilot
# ============================================================

CV_CONTROLLER_MODULE = "lane_center_follower"
CV_CONTROLLER_CLASS = "LaneCenterFollower"
CV_CONTROLLER_INPUTS = ["cam/image_array", "cam/depth_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]
CV_CONTROLLER_CONDITION = "run_pilot"


# ============================================================
# PID placeholders
# ============================================================

PID_P = -0.01
PID_I = 0.0
PID_D = -0.0001

PID_P_DELTA = 0.005
PID_D_DELTA = 0.00005


# ============================================================
# Drivetrain / VESC
# ============================================================

DRIVE_TRAIN_TYPE = "VESC"

VESC_MAX_SPEED_PERCENT = 0.3
VESC_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
VESC_HAS_SENSOR = True
VESC_START_HEARTBEAT = True
VESC_BAUDRATE = 115200
VESC_TIMEOUT = 0.05

VESC_STEERING_SCALE = 0.5
VESC_STEERING_OFFSET = 0.45


# ============================================================
# Joystick / controller
# ============================================================

USE_JOYSTICK_AS_DEFAULT = False
CONTROLLER_TYPE = "custom"

JOYSTICK_MAX_THROTTLE = 0.5
JOYSTICK_STEERING_SCALE = 1.0
JOYSTICK_DEADZONE = 0.1
JOYSTICK_THROTTLE_DIR = -1.0
JOYSTICK_DEVICE_FILE = "/dev/input/js0"

USE_NETWORKED_JS = False
NETWORK_JS_SERVER_IP = None
AUTO_RECORD_ON_THROTTLE = False
USE_FPV = False


# ============================================================
# Recording / tubs
# ============================================================

RECORD_DURING_AI = False
AUTO_CREATE_NEW_TUB = True


# ============================================================
# Disable unrelated optional features
# ============================================================

DONKEY_GYM = False
HAVE_MQTT_TELEMETRY = False
HAVE_PERFMON = False
HAVE_RGB_LED = False
USE_SSD1306_128_32 = False
STOP_SIGN_DETECTOR = False
SHOW_FPS = False


# ============================================================
# Lane-boundary CV behavior
# ============================================================

LANE_USE_BIRDSEYE = True

# IMPORTANT:
# These are the original/default bird's-eye points used by lane_center_follower.py.
# Do not use the later "wider FOV" or "conservative bird" values.
LANE_BIRD_SRC_BOTTOM_LEFT = (0.05, 0.98)
LANE_BIRD_SRC_BOTTOM_RIGHT = (0.95, 0.98)
LANE_BIRD_SRC_TOP_RIGHT = (0.68, 0.52)
LANE_BIRD_SRC_TOP_LEFT = (0.32, 0.52)

LANE_BIRD_DST_BOTTOM_LEFT = (0.15, 1.00)
LANE_BIRD_DST_BOTTOM_RIGHT = (0.85, 1.00)
LANE_BIRD_DST_TOP_RIGHT = (0.85, 0.00)
LANE_BIRD_DST_TOP_LEFT = (0.15, 0.00)

LANE_ROI_Y_START = 0.52
LANE_ROI_Y_END = 1.00
LANE_LOOKAHEAD_Y_FRACTION = 0.62
LANE_NUM_BANDS = 12

LANE_MIN_PIXELS_PER_SIDE = 10


# ============================================================
# Lane color detection
# ============================================================

LANE_MASK_COLOR = (0, 255, 0)

LANE_KERNEL_SIZE = 5
LANE_MIN_COMPONENT_AREA = 35
LANE_MIN_BAND_PIXELS = 6
LANE_MIN_RUN_WIDTH = 3
LANE_MIN_COLUMN_COUNT = 2


# ============================================================
# Lane center / fallback behavior
# ============================================================

LANE_EXPECTED_WIDTH_PX = None
LANE_EXPECTED_WIDTH_FRAC = 0.35

LANE_ALLOW_ONE_SIDE_FALLBACK = True
LANE_KEEP_SIDE = "right_of_left"

LANE_ONE_LINE_SIDE_MARGIN_PX = 25

LANE_MIN_WIDTH_FRAC = 0.15
LANE_MAX_WIDTH_FRAC = 0.98

LANE_MAX_CENTER_JUMP_PX = 80
LANE_TRACK_MAX_GAP_PX = 180


# ============================================================
# Steering / throttle
# ============================================================

LANE_TEST_THROTTLE = 0.35
LANE_THROTTLE_TURN = 0.28

LANE_STEERING_GAIN = 0.006
LANE_STRAIGHT_STEERING_GAIN = 0.003
LANE_CURVE_STEERING_GAIN = 0.010
LANE_HEADING_GAIN = 0.0

LANE_MAX_STEERING = 0.35
LANE_STEERING_BIAS = -0.05

LANE_SMOOTHING_ALPHA = 0.35
LANE_STEERING_SMOOTHING = 0.20
LANE_DEADBAND_PX = 20

LANE_CENTER_RECOVERY_ALPHA = 0.85
LANE_STEERING_RECOVERY_ALPHA = 0.75

LANE_STRAIGHT_PATH_DELTA_PX = 45
LANE_STRAIGHT_ERROR_PX = 35

LANE_BOUNDARY_MARGIN_PX = 45
LANE_LEFT_CURVE_TARGET_BIAS_PX = -20
LANE_TURN_SLOWDOWN_STEER_THRESHOLD = 0.35


# ============================================================
# Original obstacle stop logic
# ============================================================

OBSTACLE_ENABLE = True

OBSTACLE_ROI_Y_START = 0.20
OBSTACLE_ROI_Y_END = 1.00
OBSTACLE_CORRIDOR_HALF_WIDTH_PX = 180

# Red/orange obstacle detection.
OBSTACLE_HSV_LOW1 = (0, 80, 50)
OBSTACLE_HSV_HIGH1 = (20, 255, 255)
OBSTACLE_HSV_LOW2 = (160, 80, 50)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Keep dark-object detection OFF. This prevents huge boxes around monitors/chairs.
OBSTACLE_DARK_V_MAX = None

OBSTACLE_MIN_AREA = 350

OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2

OBSTACLE_STOP_THROTTLE = 0.0
OBSTACLE_STOP_STEERING = 0.0

OBSTACLE_LANE_MARGIN_PX = 10
OBSTACLE_DRAW_IGNORED = False


# ============================================================
# Hard-disable all later experiments
# ============================================================

DEPTH_VIEW_MODE = "off"
DEPTH_OBSTACLE_ENABLE = False
DEPTH_FULLSCREEN_BTN = "web/w2"   # button "2" = enter/exit full depth view
# DEPTH_OVERLAY_BTN    = "web/w1"   # button "1" = toggle thumbnail (from before)
DEPTH_OVERLAY_DEFAULT_ON = False

YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_OBSTACLE_AVOIDANCE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False

# ============================================================
# FINAL: blue lane tape detection
# ============================================================

# OpenCV HSV blue range. This is what the current blue tape needs.
LANE_HSV_LOW = (80, 15, 20)
LANE_HSV_HIGH = (155, 255, 255)

LANE_MASK_COLOR = (0, 255, 0)


# ============================================================
# FIX: restore steering authority
# ============================================================

# Let the lane follower command a real steering angle again.
LANE_MAX_STEERING = 0.75

# Make the VESC actually apply more of the steering command.
VESC_STEERING_SCALE = 0.65

# Normal speed can stay slow.
VESC_MAX_SPEED_PERCENT = 0.25
LANE_TEST_THROTTLE = 0.25
LANE_THROTTLE_TURN = 0.18

# Make steering respond more clearly.
LANE_STEERING_GAIN = 0.008
LANE_STRAIGHT_STEERING_GAIN = 0.004
LANE_CURVE_STEERING_GAIN = 0.012

# Do not ignore as much center error.
LANE_DEADBAND_PX = 8

# Slightly less smoothing so wheel response is not delayed.
LANE_STEERING_SMOOTHING = 0.35
LANE_SMOOTHING_ALPHA = 0.45

# Use blue tape detection.
LANE_HSV_LOW = (80, 15, 20)
LANE_HSV_HIGH = (155, 255, 255)


# ============================================================
# FIX: stronger realignment to lane center
# ============================================================

# Remove steering bias that was canceling small corrections.
LANE_STEERING_BIAS = 0.0

# Let the car steer more than before.
LANE_MAX_STEERING = 0.75
VESC_STEERING_SCALE = 0.70

# Stronger correction even on straight paths.
LANE_STEERING_GAIN = 0.010
LANE_STRAIGHT_STEERING_GAIN = 0.008
LANE_CURVE_STEERING_GAIN = 0.014

# Do not ignore small-but-visible center errors.
LANE_DEADBAND_PX = 5

# React faster to new centerline.
LANE_SMOOTHING_ALPHA = 0.55
LANE_STEERING_SMOOTHING = 0.45
LANE_CENTER_RECOVERY_ALPHA = 0.95
LANE_STEERING_RECOVERY_ALPHA = 0.90

# Keep speed slower while testing steering.
VESC_MAX_SPEED_PERCENT = 0.3
LANE_TEST_THROTTLE = 0.33
LANE_THROTTLE_TURN = 0.33
# ============================================================
# Detect dark/black obstacles too
# ============================================================

# Keep red/orange obstacle detection.
OBSTACLE_HSV_LOW1 = (0, 70, 40)
OBSTACLE_HSV_HIGH1 = (25, 255, 255)
OBSTACLE_HSV_LOW2 = (155, 70, 40)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Also detect dark objects.
# Lower = fewer false positives, may miss dark objects.
# Higher = more sensitive, may detect shadows/chairs/monitors.
OBSTACLE_DARK_V_MAX = 80

# Make obstacle blobs large enough so floor specks/shadows are ignored.
OBSTACLE_MIN_AREA = 500

# Only stop if the obstacle bottom-center is inside the two detected lanes.
OBSTACLE_LANE_MARGIN_PX = 15
OBSTACLE_DRAW_IGNORED = False

# Keep the same stop behavior.
OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2
OBSTACLE_STOP_THROTTLE = 0.0
OBSTACLE_STOP_STEERING = 0.0

# ============================================================
# YOLO + LiDAR obstacle detection
# ============================================================

USE_YOLO_LIDAR_OBSTACLE = True

YOLO_MODEL_PATH = "models/yolo11n.pt"
YOLO_CONFIDENCE = 0.35
YOLO_IMAGE_SIZE = 640
YOLO_DEVICE = 0

# COCO classes to care about.
# Start with common demo objects. Add/remove as needed.
YOLO_TARGET_CLASSES = [
    "person",
    "car",
    "truck",
    "bus",
    "bottle",
    "chair",
    "backpack",
    "suitcase",
]

# Run YOLO every N frames to keep driving fast.
YOLO_RUN_EVERY_N_FRAMES = 2

# Stop logic.
YOLO_STOP_DISTANCE_M = 0.75
YOLO_MIN_HEIGHT_M = 0.05
YOLO_STOP_THROTTLE = 0.0
YOLO_STOP_STEERING = 0.0

# Camera calibration for estimating physical width/height.
# If calibration loading fails, these are used as fallback.
CAMERA_FX_PX = 560.91
CAMERA_FY_PX = 560.94

# LiDAR settings.
LIDAR_ENABLE = True
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200

# Angle calibration:
# 0 deg should mean "straight ahead from the car".
# Tune these if LiDAR distance is being read from the wrong direction.
LIDAR_FRONT_ANGLE_DEG = 0.0
LIDAR_YAW_OFFSET_DEG = 0.0
LIDAR_CAMERA_ANGLE_SIGN = 1.0
LIDAR_ANGLE_WINDOW_DEG = 5.0

# Only accept LiDAR distances in this range.
LIDAR_MIN_DISTANCE_M = 0.05
LIDAR_MAX_DISTANCE_M = 5.0

# Draw boxes and labels on the DonkeyCar web image.
YOLO_DRAW_DEBUG = True

# Disable old color-only obstacle stopping while testing YOLO.
# Turn this back on later if you want both systems.
OBSTACLE_ENABLE = False

# ============================================================
# Fix OAK calibration size mismatch
# ============================================================

IMAGE_W = 1280
IMAGE_H = 800
IMAGE_DEPTH = 3

# ============================================================
# YOLO + Livox MID-360 Ethernet LiDAR fusion
# ============================================================

USE_YOLO_LIDAR_OBSTACLE = True

# Restore calibrated OAK-D image size
IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3

# YOLO model
YOLO_MODEL_PATH = "models/yolo11n.pt"
YOLO_CONFIDENCE = 0.35
YOLO_IMAGE_SIZE = 640
YOLO_DEVICE = 0
YOLO_RUN_EVERY_N_FRAMES = 2
YOLO_DRAW_DEBUG = True

YOLO_TARGET_CLASSES = [
    "person",
    "car",
    "truck",
    "bus",
    "bottle",
    "chair",
    "backpack",
    "suitcase",
]

# Stop thresholds
YOLO_STOP_DISTANCE_M = 0.75
YOLO_MIN_HEIGHT_M = 0.05
YOLO_STOP_THROTTLE = 0.0
YOLO_STOP_STEERING = 0.0

# Camera fallback intrinsics; calibration file will override these if available.
CAMERA_FX_PX = 560.91
CAMERA_FY_PX = 560.94

# Disable old serial LiDAR reader.
LIDAR_ENABLE = False

# Livox MID-360 Ethernet settings from tcpdump
ETH_LIDAR_ENABLE = True
ETH_LIDAR_IP = "192.168.1.199"
ETH_LIDAR_INTERFACE = "eth0"
ETH_LIDAR_DATA_PORT = 56301
ETH_LIDAR_SOURCE_DATA_PORT = 56300

# Angle matching between camera bbox center and LiDAR point cloud.
# If left/right matching is reversed, flip this to 1.0.
LIVOX_CAMERA_ANGLE_SIGN = -1.0
LIVOX_CAMERA_ANGLE_OFFSET_DEG = 0.0
LIVOX_ANGLE_WINDOW_DEG = 6.0

# Livox coordinate assumptions:
# x = forward, y = lateral, z = vertical, units mm.
LIVOX_MIN_FORWARD_M = 0.05
LIVOX_MAX_FORWARD_M = 5.0

# Use only points near object angle.
LIVOX_MIN_POINTS_FOR_MATCH = 5

# Keep old color-based obstacle detector off while testing YOLO.
OBSTACLE_ENABLE = False

# ============================================================
# View-button mode setup
# ============================================================

# Keep OAK-D depth camera enabled for button 5 heatmap.
IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3

CV_CONTROLLER_INPUTS = ["cam/image_array", "cam/depth_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]

# Turn off bad YOLO/LiDAR overlay for now.
USE_YOLO_LIDAR_OBSTACLE = False
YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False
LIDAR_ENABLE = False
ETH_LIDAR_ENABLE = False

# Restore original non-YOLO obstacle detector inside lane_center_follower.py.
OBSTACLE_ENABLE = True

# Button mapping:
# web/w1 = button 1, web/w2 = button 2, etc.
VIEW_RAW_BTN = "web/w1"
VIEW_LANE_BTN = "web/w2"
VIEW_OBSTACLE_BTN = "web/w3"
VIEW_TRACKER_BTN = "web/w4"
VIEW_DEPTH_BTN = "web/w5"

DEFAULT_VIEW_MODE = "lane"

# Disable old fullscreen-depth toggle so button 2 is not stolen.
DEPTH_FULLSCREEN_BTN = None
DEPTH_OVERLAY_BTN = None
DEPTH_OVERLAY_DEFAULT_ON = False

# ============================================================
# View 3: YOLO-only raw RGB object detection view
# ============================================================

YOLO_VIEW_ENABLE = True
YOLO_VIEW_MODEL_PATH = "models/yolo11n.pt"
YOLO_VIEW_CONFIDENCE = 0.20
YOLO_VIEW_IMAGE_SIZE = 1280
YOLO_VIEW_DEVICE = 0
YOLO_VIEW_RUN_EVERY_N_FRAMES = 2

# Classes to show in View 3.
# YOLO_VIEW_TARGET_CLASSES = [
#     "person",
#     "car",
#     "truck",
#     "bus",
#     "bottle",
#     "chair",
#     "backpack",
#     "suitcase",
#     "wheel",
#     "box"
# ]
YOLO_VIEW_TARGET_CLASSES = []

# Keep the bad YOLO/LiDAR stopping overlay disabled.
# View 3 is display-only YOLO.
USE_YOLO_LIDAR_OBSTACLE = False
YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False
LIDAR_ENABLE = False
ETH_LIDAR_ENABLE = False

# Keep original non-YOLO obstacle stop active for View 2.
OBSTACLE_ENABLE = True

# ============================================================
# View 3: show YOLO confidence + OAK-D depth distance/height
# ============================================================

YOLO_VIEW_SHOW_DISTANCE_HEIGHT = True

# Depth image is usually in millimeters.
YOLO_VIEW_DEPTH_MIN_MM = 100
# YOLO_VIEW_DEPTH_MAX_MM = 5000
YOLO_VIEW_DEPTH_MAX_MM = 8000

# Use the center of the box for distance so background edges matter less.
# YOLO_VIEW_DEPTH_CENTER_FRAC = 0.350
YOLO_VIEW_DEPTH_CENTER_FRAC = 0.8
# (add this parameter — currently hardcoded to 20 in the method)
YOLO_VIEW_DEPTH_MIN_VALID_PX = 5

# Camera intrinsics fallback. Calibration may override this elsewhere.
CAMERA_FX_PX = 560.91
CAMERA_FY_PX = 560.94

# ============================================================
# View 4: bird's-eye YOLO obstacle view + height/distance stop
# ============================================================

# Keep the separate YOLO/LiDAR DonkeyCar part disabled so it does not overwrite views.
USE_YOLO_LIDAR_OBSTACLE = False
YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False
LIDAR_ENABLE = False
ETH_LIDAR_ENABLE = False

# Keep original red/orange obstacle stop for View 2.
OBSTACLE_ENABLE = True

# Button 4 behavior.
YOLO_BIRD_ENABLE = True
YOLO_BIRD_MODEL_PATH = "models/yolo11n.pt"
YOLO_BIRD_CONFIDENCE = 0.30
YOLO_BIRD_IMAGE_SIZE = 640
YOLO_BIRD_DEVICE = 0
YOLO_BIRD_RUN_EVERY_N_FRAMES = 2

# Empty list means: use all YOLO detections internally,
# but display every accepted detection as "OBSTACLE".
YOLO_BIRD_TARGET_CLASSES = []

# Height threshold:
# 0.07 m = 7 cm. If obstacle height <= 7 cm, do not stop.
YOLO_BIRD_MIN_STOP_HEIGHT_M = 0.07

# Stop distance:
# 1.5 ft = 0.4572 m.
YOLO_BIRD_STOP_DISTANCE_M = 0.4572

YOLO_BIRD_STOP_THROTTLE = 0.0
YOLO_BIRD_STOP_STEERING = 0.0

# Only stop for YOLO obstacles whose bottom-center lands inside the lane.
YOLO_BIRD_REQUIRE_INSIDE_LANE = True

# Depth filtering in millimeters.
YOLO_BIRD_DEPTH_MIN_MM = 100
YOLO_BIRD_DEPTH_MAX_MM = 5000
YOLO_BIRD_DEPTH_CENTER_FRAC = 0.35

# Camera intrinsics fallback.
CAMERA_FX_PX = 560.91
CAMERA_FY_PX = 560.94

# ============================================================
# View 2: stricter original obstacle stop logic
# ============================================================

# Keep original non-YOLO obstacle detector active for View 2.
OBSTACLE_ENABLE = True

# Disable the external YOLO/LiDAR part so it does not overwrite cv/image_array.
USE_YOLO_LIDAR_OBSTACLE = False
YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False
LIDAR_ENABLE = False
ETH_LIDAR_ENABLE = False

# The old wide search box is no longer the real gate.
# Detection will be masked to the actual lane region.
OBSTACLE_CORRIDOR_HALF_WIDTH_PX = 260

# Require obstacle pixels to be inside BOTH detected green lane boundaries.
OBSTACLE_REQUIRE_ACTUAL_TWO_LANES = True

# Shrink the allowed lane region slightly so lane-edge/background objects are ignored.
OBSTACLE_LANE_MASK_MARGIN_PX = 18

# Do not draw outside-lane ignored detections.
OBSTACLE_DRAW_IGNORED = False

# Keep red/orange obstacle detection.
OBSTACLE_HSV_LOW1 = (0, 70, 40)
OBSTACLE_HSV_HIGH1 = (25, 255, 255)
OBSTACLE_HSV_LOW2 = (155, 70, 40)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Recommended for demo stability:
# turn this off if monitors/chairs/shadows keep triggering false obstacles.
OBSTACLE_DARK_V_MAX = None

# Ignore small blobs.
OBSTACLE_MIN_AREA = 500

# Stop behavior.
OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2
OBSTACLE_STOP_THROTTLE = 0.0
OBSTACLE_STOP_STEERING = 0.0

# ============================================================
# FIX: View 2 false obstacle detections
# ============================================================

OBSTACLE_ENABLE = True

# Disable dark obstacle detection for View 2.
# This prevents shadows, black floor regions, monitors, and chair legs from triggering.
OBSTACLE_DARK_V_MAX = None

# Make red/orange detection stricter so tan floor tiles do not count.
# Higher S means only strongly saturated red/orange objects are accepted.
OBSTACLE_HSV_LOW1 = (0, 120, 70)
OBSTACLE_HSV_HIGH1 = (18, 255, 255)
OBSTACLE_HSV_LOW2 = (165, 120, 70)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Require larger object blobs.
OBSTACLE_MIN_AREA = 900

# Require the obstacle to have real height in the bird's-eye image.
OBSTACLE_MIN_BBOX_HEIGHT_PX = 35
OBSTACLE_MIN_BBOX_WIDTH_PX = 20

# Keep using actual lane interior gate.
OBSTACLE_LANE_MASK_MARGIN_PX = 25
OBSTACLE_DRAW_IGNORED = False

# ============================================================
# FINAL MERGE: View 4 = YOLO + View 2 lane-mask stop logic
# ============================================================

# Keep external YOLO/LiDAR DonkeyCar part disabled.
USE_YOLO_LIDAR_OBSTACLE = False
YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False
LIDAR_ENABLE = False
ETH_LIDAR_ENABLE = False

# Keep original View 2 detector enabled because it will be used as cone fallback.
OBSTACLE_ENABLE = True

# View 4 YOLO settings.
YOLO_BIRD_ENABLE = True
YOLO_BIRD_MODEL_PATH = "models/yolo11n.pt"
YOLO_BIRD_CONFIDENCE = 0.25
YOLO_BIRD_IMAGE_SIZE = 640
YOLO_BIRD_DEVICE = 0
YOLO_BIRD_RUN_EVERY_N_FRAMES = 2

# Empty list = accept all YOLO classes, then display them as OBSTACLE.
YOLO_BIRD_TARGET_CLASSES = []

# Instead of checking one point, require projected object footprint to overlap lane mask.
YOLO_BIRD_LANE_OVERLAP_MIN = 0.20

# Use only bottom part of YOLO box as ground contact footprint.
# This fixes the huge weird yellow trapezoids from projecting full vertical boxes.
YOLO_BIRD_BASE_FRAC = 0.25

# Use original red/orange View 2 detector as fallback.
# This is what will catch orange cones when YOLO misses them.
YOLO_BIRD_COLOR_FALLBACK = True

# Height rule for YOLO objects.
YOLO_BIRD_MIN_STOP_HEIGHT_M = 0.07

# Use View 2 stop-line logic, not depth-distance logic.
OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2
YOLO_BIRD_STOP_THROTTLE = 0.0
YOLO_BIRD_STOP_STEERING = 0.0

# Cone/color fallback thresholds.
OBSTACLE_HSV_LOW1 = (0, 120, 70)
OBSTACLE_HSV_HIGH1 = (25, 255, 255)
OBSTACLE_HSV_LOW2 = (155, 120, 70)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)
OBSTACLE_DARK_V_MAX = None
OBSTACLE_MIN_AREA = 500
OBSTACLE_MIN_BBOX_HEIGHT_PX = 25
OBSTACLE_MIN_BBOX_WIDTH_PX = 15
OBSTACLE_LANE_MASK_MARGIN_PX = 20

# ============================================================
# View 4: generic dark/depth obstacle fallback for black wheel
# ============================================================

# Use OAK-D depth inside View 4 for distance/height.
# The separate YOLO/LiDAR part stays disabled.
USE_YOLO_LIDAR_OBSTACLE = False
LIDAR_ENABLE = False
ETH_LIDAR_ENABLE = False

# Car clearance rule.
# If object height is <= this, View 4 labels DRIVE OVER and does not stop.
# Set this to the actual underbelly clearance of your car.
CAR_UNDERBELLY_CLEARANCE_M = 0.07

# Stop line behavior from View 2.
OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2

# Enable generic black/dark obstacle fallback in View 4 only.
YOLO_BIRD_DARK_FALLBACK = True

# Dark-object threshold.
# Lower = stricter, fewer false positives.
# Higher = detects more dark objects but more shadows.
YOLO_BIRD_DARK_V_MAX = 65

# Reject tiny dark specks.
YOLO_BIRD_DARK_MIN_AREA = 450
YOLO_BIRD_DARK_MIN_WIDTH_PX = 18
YOLO_BIRD_DARK_MIN_HEIGHT_PX = 18

# Depth validity.
YOLO_BIRD_DEPTH_MIN_MM = 100
YOLO_BIRD_DEPTH_MAX_MM = 5000
YOLO_BIRD_DEPTH_CENTER_FRAC = 0.45

# Make View 4 use the same lane mask logic.
YOLO_BIRD_LANE_OVERLAP_MIN = 0.15
YOLO_BIRD_BASE_FRAC = 0.25
YOLO_BIRD_COLOR_FALLBACK = True

# Keep View 2 stable: do not turn on global dark detection there.
OBSTACLE_DARK_V_MAX = None

# Keep actual camera/CV/depth processing resolution small.
IMAGE_W = 1280
IMAGE_H = 800
IMAGE_DEPTH = 3

# Do NOT upscale the actual ui/image_array in Python.
# We only want browser display scaling.
WEB_UPSCALE_IMAGE = False

# ============================================================
# View 4: compute obstacle height in raw camera view, display in bird view
# ============================================================

# Height/distance should be computed from raw RGB/depth coordinates,
# not from stretched bird's-eye coordinates.
YOLO_BIRD_MEASURE_IN_RAW = True

# Use only the bottom part of a raw object box for bird's-eye lane overlap.
YOLO_BIRD_BASE_FRAC = 0.20

# Raw fallback detectors for unknown objects / cones.
YOLO_BIRD_RAW_COLOR_FALLBACK = True
YOLO_BIRD_RAW_DARK_FALLBACK = True

# Dark object threshold for raw RGB, useful for black wheel.
YOLO_BIRD_DARK_V_MAX = 70
YOLO_BIRD_DARK_MIN_AREA = 350
YOLO_BIRD_DARK_MIN_WIDTH_PX = 15
YOLO_BIRD_DARK_MIN_HEIGHT_PX = 15

# Car clearance.
CAR_UNDERBELLY_CLEARANCE_M = 0.07

# Stop line still uses bird's-eye view.
OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2

# ============================================================
# View 4: tire / black wheel detection fix
# ============================================================

YOLO_BIRD_DARK_FALLBACK = True
YOLO_BIRD_RAW_DARK_FALLBACK = True

# Make dark detection more sensitive for black rubber.
YOLO_BIRD_DARK_V_MAX = 95

# Remove overly strict tiny-object rejection.
YOLO_BIRD_DARK_MIN_AREA = 120
YOLO_BIRD_DARK_MIN_WIDTH_PX = 8
YOLO_BIRD_DARK_MIN_HEIGHT_PX = 8

# Process more dark candidates so outside objects do not hide the tire.
YOLO_BIRD_MAX_DARK_CANDIDATES = 12

# Keep clearance rule.
CAR_UNDERBELLY_CLEARANCE_M = 0.07

# Keep View 2 dark detection off.
OBSTACLE_DARK_V_MAX = None

# ============================================================
# View 4 clean lane-gated obstacle logic
# ============================================================

# View 4 should ignore all detections outside the two green lanes.
VIEW4_DRAW_OUTSIDE_LANE = False
VIEW4_ONLY_INSIDE_LANE = True

# Require enough overlap between object footprint and lane mask.
YOLO_BIRD_LANE_OVERLAP_MIN = 0.35

# Use only the bottom/base of objects for lane overlap.
YOLO_BIRD_BASE_FRAC = 0.20

# Clearance rule.
CAR_UNDERBELLY_CLEARANCE_M = 0.07

# Depth filtering.
YOLO_BIRD_DEPTH_MIN_MM = 100
YOLO_BIRD_DEPTH_MAX_MM = 5000
YOLO_BIRD_DEPTH_CENTER_FRAC = 0.65

# Keep fallback detectors, but lane-gate them.
YOLO_BIRD_RAW_COLOR_FALLBACK = True
YOLO_BIRD_RAW_DARK_FALLBACK = True
YOLO_BIRD_DARK_FALLBACK = True
YOLO_BIRD_COLOR_FALLBACK = True

# Dark tire sensitivity.
YOLO_BIRD_DARK_V_MAX = 95
YOLO_BIRD_DARK_MIN_AREA = 120
YOLO_BIRD_DARK_MIN_WIDTH_PX = 8
YOLO_BIRD_DARK_MIN_HEIGHT_PX = 8
YOLO_BIRD_MAX_DARK_CANDIDATES = 8

# Do not trust View 3 for final height/distance; View 3 is raw YOLO debug only.
# View 4 is the lane-gated final obstacle view.
YOLO_VIEW_SHOW_DISTANCE_HEIGHT = True

# Keep View 2 dark detection off so shadows do not trigger original stop.
OBSTACLE_DARK_V_MAX = None

# ============================================================
# View 4: better low-object height estimate
# ============================================================

# Estimate height from depth points relative to floor, not bbox height.
VIEW4_USE_GROUND_PLANE_HEIGHT = True

# Ground plane sampling around the object.
VIEW4_GROUND_SAMPLE_PAD_PX = 55
VIEW4_OBJECT_DEPTH_TOL_M = 0.30
VIEW4_MIN_GROUND_POINTS = 80
VIEW4_MIN_OBJECT_POINTS = 20

# Tire/dark object detection should not be clipped by the lane mask.
# The lane mask is used only to decide whether the object matters.
YOLO_BIRD_DARK_V_MAX = 105
YOLO_BIRD_DARK_MIN_AREA = 80
YOLO_BIRD_DARK_MIN_WIDTH_PX = 6
YOLO_BIRD_DARK_MIN_HEIGHT_PX = 6
YOLO_BIRD_MAX_DARK_CANDIDATES = 12

# Clearance rule.
CAR_UNDERBELLY_CLEARANCE_M = 0.07

# Require object base to overlap lane.
YOLO_BIRD_LANE_OVERLAP_MIN = 0.25
YOLO_BIRD_BASE_FRAC = 0.20

# View 4 should ignore all outside-lane objects.
VIEW4_ONLY_INSIDE_LANE = True
VIEW4_DRAW_OUTSIDE_LANE = False

# ============================================================
# FINAL 1280x800 RECALIBRATED SETUP
# ============================================================

IMAGE_W = 1280
IMAGE_H = 800
IMAGE_DEPTH = 3

# New calibration file that you will generate below.
OAK_UNDISTORT = False
OAK_CALIBRATION_FILE = "/home/jetson/projects/mycars/lane_boundary_cv/camera_calibration.npz"
OAK_UNDISTORT_ALPHA = 0.1

# YOLO still accepts imgsz=640 internally, but boxes are returned
# in the original 1280x800 image coordinates.
YOLO_VIEW_IMAGE_SIZE = 640
YOLO_BIRD_IMAGE_SIZE = 640

# These will be overwritten from the calibration file after calibration.
CAMERA_FX_PX = 560.91
CAMERA_FY_PX = 560.94


