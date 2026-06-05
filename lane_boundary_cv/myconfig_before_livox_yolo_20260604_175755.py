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

OAK_UNDISTORT = True
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
CAMERA_FX_PX = 287.82
CAMERA_FY_PX = 287.82

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

IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3
