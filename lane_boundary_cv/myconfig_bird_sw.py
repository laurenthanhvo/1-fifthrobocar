"""
Focused DonkeyCar config for lane_boundary_cv.

Active pilot:
    lane_center_follower_bird_sw.LaneCenterFollower

That pilot runs a bird's-eye transform, tracks left/right lane boundaries with
sliding windows, derives the center between those boundary fits, then smooths
and rate-limits the steering target.
"""

import os


# ============================================================
# Paths
# ============================================================

CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, "data")
METADATA = []


# ============================================================
# Vehicle Loop
# ============================================================

DRIVE_LOOP_HZ = 20
MAX_LOOPS = None


# ============================================================
# Camera
# ============================================================

CAMERA_TYPE = "OAKD"
CAMERA_INDEX = 0

IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

CAMERA_VFLIP = False
CAMERA_HFLIP = False
BGR2RGB = False

OAK_UNDISTORT = True
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")
OAK_UNDISTORT_ALPHA = 0.1


# ============================================================
# Web / User Control
# ============================================================

WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))
WEB_INIT_MODE = "user"
OVERLAY_IMAGE = True

USE_JOYSTICK_AS_DEFAULT = False
CONTROLLER_TYPE = "custom"
JOYSTICK_DEVICE_FILE = "/dev/input/js0"
JOYSTICK_MAX_THROTTLE = 0.5
JOYSTICK_STEERING_SCALE = 1.0
JOYSTICK_DEADZONE = 0.1
JOYSTICK_THROTTLE_DIR = -1.0
USE_NETWORKED_JS = False
NETWORK_JS_SERVER_IP = None
USE_FPV = False


# ============================================================
# Recording / Optional Parts
# ============================================================

AUTO_RECORD_ON_THROTTLE = False
RECORD_DURING_AI = False
AUTO_CREATE_NEW_TUB = True

DONKEY_GYM = False
HAVE_MQTT_TELEMETRY = False
HAVE_PERFMON = False
HAVE_RGB_LED = False
USE_SSD1306_128_32 = False
SSD1306_128_32_I2C_ROTATION = 0
SSD1306_RESOLUTION = 1
STOP_SIGN_DETECTOR = False
SHOW_FPS = False


# ============================================================
# Buttons / Template Compatibility
# ============================================================

TOGGLE_RECORDING_BTN = "option"
INC_PID_P_BTN = None
DEC_PID_P_BTN = None
INC_PID_D_BTN = None
DEC_PID_D_BTN = None

PID_P = -0.01
PID_I = 0.0
PID_D = -0.0001
PID_P_DELTA = 0.005
PID_D_DELTA = 0.00005

AI_THROTTLE_MULT = 1.0


# ============================================================
# Drivetrain / VESC
# ============================================================

DRIVE_TRAIN_TYPE = "VESC"

VESC_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
VESC_HAS_SENSOR = True
VESC_START_HEARTBEAT = True
VESC_BAUDRATE = 115200
VESC_TIMEOUT = 0.05

VESC_MAX_SPEED_PERCENT = 0.35
VESC_STEERING_SCALE = 0.55
VESC_STEERING_OFFSET = 0.45


# ============================================================
# Active Computer Vision Pilot
# ============================================================

CV_CONTROLLER_MODULE = "lane_center_follower_bird_sw"
CV_CONTROLLER_CLASS = "LaneCenterFollower"
CV_CONTROLLER_INPUTS = ["cam/image_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]
CV_CONTROLLER_CONDITION = "run_pilot"


# ============================================================
# Bird's-Eye Transform
# ============================================================

LANE_USE_BIRDSEYE = True

# Source trapezoid in the undistorted OAK frame.
LANE_BIRD_SRC_BOTTOM_LEFT = (0.08, 0.98)
LANE_BIRD_SRC_BOTTOM_RIGHT = (0.92, 0.98)
LANE_BIRD_SRC_TOP_LEFT = (0.38, 0.62)
LANE_BIRD_SRC_TOP_RIGHT = (0.62, 0.62)

# Destination rectangle in the transformed frame.
LANE_BIRD_DST_BOTTOM_LEFT = (0.20, 1.00)
LANE_BIRD_DST_BOTTOM_RIGHT = (0.80, 1.00)
LANE_BIRD_DST_TOP_LEFT = (0.20, 0.00)
LANE_BIRD_DST_TOP_RIGHT = (0.80, 0.00)


# ============================================================
# Lane Mask
# ============================================================

# Blue tape HSV threshold.
LANE_HSV_LOW = (90, 50, 50)
LANE_HSV_HIGH = (130, 255, 255)
LANE_MASK_COLOR = (0, 255, 0)

LANE_KERNEL_SIZE = 5
LANE_MIN_COMPONENT_AREA = 45


# ============================================================
# Region Of Interest / Search Area
# ============================================================

LANE_ROI_Y_START = 0.35
LANE_ROI_Y_END = 1.00

# Smaller values look farther ahead; larger values react closer to the car.
LANE_LOOKAHEAD_Y_FRACTION = 0.62

# Ignore warped-image edge noise.
LANE_SEARCH_X_START_FRAC = 0.08
LANE_SEARCH_X_END_FRAC = 0.92

# Use only if the camera is physically mounted off the car centerline.
LANE_IMAGE_CENTER_OFFSET_PX = 0


# ============================================================
# Sliding-Window Boundary Tracking
# ============================================================

LANE_SW_WINDOWS = 8
LANE_SW_MARGIN = 35
LANE_SW_MIN_WINDOW_PIXELS = 3
LANE_SW_MIN_LANE_PIXELS = 18
LANE_ONE_SIDE_MIN_PIXELS = 18

# Once a lane has been found, prefer pixels near the last left/right fits.
LANE_PREVIOUS_FIT_MARGIN = 50

# Reject sudden fitted-center jumps before they become steering targets.
LANE_MAX_CENTER_JUMP_PX = 120
LANE_MAX_MISSED_FRAMES = 2
LANE_DROPOUT_THROTTLE = 0.0


# ============================================================
# Lane Width / One-Side Fallback
# ============================================================

LANE_ALLOW_ONE_SIDE_FALLBACK = True

# If this is None, the tracker learns width from validated two-boundary frames.
# Set to 280 to force the older bird's-eye width estimate.
LANE_EXPECTED_WIDTH_PX = None
LANE_EXPECTED_WIDTH_FRAC = 0.55
LANE_WIDTH_SMOOTHING_ALPHA = 0.15

LANE_MIN_WIDTH_FRAC = 0.20
LANE_MAX_WIDTH_FRAC = 0.75
LANE_MIN_WIDTH_PX = None
LANE_MAX_WIDTH_PX = None


# ============================================================
# Target Stabilization
# ============================================================

LANE_TARGET_MEDIAN_WINDOW = 5
LANE_MAX_TARGET_STEP_PX = 10
LANE_FALLBACK_MAX_TARGET_STEP_PX = 5
LANE_SMOOTHING_ALPHA = 0.28
LANE_FALLBACK_SMOOTHING_ALPHA = 0.10


# ============================================================
# Steering / Speed
# ============================================================

LANE_STEERING_GAIN = 0.06
LANE_MAX_STEERING = 0.80
LANE_STEERING_BIAS = -0.05
LANE_INVERT_STEERING = False

LANE_TEST_THROTTLE = 0.30
LANE_THROTTLE_MIN = 0.20
LANE_THROTTLE_TURN = 0.25
LANE_ONE_SIDE_THROTTLE = 0.20
LANE_THROTTLE_SLOWDOWN = 0.0
