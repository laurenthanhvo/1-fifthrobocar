"""
Custom DonkeyCar config for lane_boundary_cv.

This file overrides the default config.py values.
The goal is to run a custom computer-vision autopilot that drives between
two lane boundaries using lane_center_follower.py.
"""
# myconfig.py
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

# IMAGE_W = 320
# IMAGE_H = 240
IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

CAMERA_VFLIP = False
CAMERA_HFLIP = False
BGR2RGB = False

# Use the checkerboard calibration made by oak_camera_calibration.py.
OAK_UNDISTORT = True
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")
OAK_UNDISTORT_ALPHA = 0.1 # 1.0 makes the camera spherical, 0.0 is a bit zoomed in, 0.1 is sweet spot


# ============================================================
# Web UI
# ============================================================

WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))
WEB_INIT_MODE = "user"

# Show the CV debug overlay in the DonkeyCar web UI.
OVERLAY_IMAGE = True


# ============================================================
# Custom computer vision autopilot
# ============================================================

# This must match the filename:
# lane_center_follower.py
CV_CONTROLLER_MODULE = "lane_center_follower"
CV_CONTROLLER_CLASS = "LaneCenterFollower"

CV_CONTROLLER_INPUTS = ["cam/image_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]
CV_CONTROLLER_CONDITION = "run_pilot"


# ============================================================
# PID values
# ============================================================
# The current lane_center_follower.py uses direct steering by default:
# steering = LANE_STEERING_GAIN * error
#
# These PID values are still included because the DonkeyCar CV template
# expects PID config values to exist.

PID_P = -0.01
PID_I = 0.0
PID_D = -0.0001

PID_P_DELTA = 0.005
PID_D_DELTA = 0.00005


# ============================================================
# Drivetrain / VESC
# ============================================================

DRIVE_TRAIN_TYPE = "VESC"

VESC_MAX_SPEED_PERCENT = 0.2
VESC_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
VESC_HAS_SENSOR = True
VESC_START_HEARTBEAT = True
VESC_BAUDRATE = 115200
VESC_TIMEOUT = 0.05

# Existing steering calibration from the working setup.
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
# Simulator
# ============================================================

DONKEY_GYM = False


# ============================================================
# Lane-boundary CV behavior
# ============================================================
# Goal:
#   Detect left lane boundary + right lane boundary
#   Compute the center between them
#   Steer to stay centered between the two boundaries
#
# This is different from DonkeyCar's default line follower, which follows
# one single center line.


# ----------------------------
# Region of interest
# ----------------------------

# Look at lower part of the image where the track is visible.
# Lower value sees farther ahead; higher value ignores more background.
# LANE_ROI_Y_START = 0.45
# ---
LANE_ROI_Y_START = 0.52  
LANE_ROI_Y_END = 1.00

# Smaller = look farther ahead into curves.
# Larger = react closer to the car.
# Try 0.60-0.70 for curves.
LANE_LOOKAHEAD_Y_FRACTION = 0.62

# Number of horizontal bands to scan for curve-aware lane detection.
# LANE_NUM_BANDS = 10
# ---
LANE_NUM_BANDS = 12



# ----------------------------
# Two-lane / one-line fallback
# ----------------------------

# Learn lane width when both boundaries are visible.
# If this is None, the code learns lane width automatically.
LANE_EXPECTED_WIDTH_PX = None

# Allow the car to keep estimating the lane center if only one boundary is visible.
LANE_ALLOW_ONE_SIDE_FALLBACK = True

# This is the driving rule for one-line fallback.
# Use this if the car should stay to the RIGHT of a visible left boundary.
LANE_KEEP_SIDE = "right_of_left"

# Allow wide lane boundaries.
LANE_MIN_WIDTH_FRAC = 0.15
LANE_MAX_WIDTH_FRAC = 0.98


# ----------------------------
# Lane color detection
# ----------------------------
# Handles:
#   white tape
#   blue tape
#   washed-out blue tape that looks pale under sunlight

# Broad white / washed-out tape detection.
# LANE_HSV_LOW = (0, 0, 135)
# LANE_HSV_HIGH = (180, 120, 255)
# ---
LANE_HSV_LOW             = (18,  100, 100)   # tight yellow only
LANE_HSV_HIGH            = (32,  255, 255)

LANE_WHITE_S_MAX = 125
LANE_WHITE_V_MIN = 105

# Blue tape detection.
LANE_BLUE_H_LOW = 80
LANE_BLUE_H_HIGH = 155
LANE_BLUE_S_MIN = 15
LANE_BLUE_V_MIN = 20

# RGB blue fallback for sunlight / washed-out tape.
LANE_BLUE_RGB_MARGIN = 5
LANE_BLUE_RGB_MIN = 25

# Keep contrast enhancement off first.
# Turn this True only if dim lighting becomes a major issue.
LANE_USE_CLAHE = False
LANE_CLAHE_CLIP_LIMIT = 2.0
LANE_CLAHE_TILE_GRID_SIZE = 8


# ----------------------------
# Noise filtering
# ----------------------------

LANE_KERNEL_SIZE = 5

# Lower = more sensitive but noisier.
# Higher = cleaner but may miss thin/washed-out tape.
LANE_MIN_COMPONENT_AREA = 35

LANE_MIN_BAND_PIXELS = 6
LANE_MIN_RUN_WIDTH = 3
LANE_MIN_COLUMN_COUNT = 2


# ----------------------------
# Steering / throttle tuning
# ----------------------------

# Increase this if the car is not turning hard enough into curves.
LANE_STEERING_GAIN = 0.006

# Allow stronger steering than before.
LANE_MAX_STEERING = 0.70

# Optional drift correction.
# This only works if lane_center_follower.py applies LANE_STEERING_BIAS.
# If your code does not use this yet, it is harmless.
# LANE_STEERING_BIAS = 0.10
LANE_STEERING_BIAS = -0.05

# Higher = reacts faster but jitters more.
# Lower = smoother but slower to turn.
LANE_SMOOTHING_ALPHA = 0.35

LANE_STEERING_SMOOTHING  = 0.20  # new: steering output smoothing
LANE_DEADBAND_PX         = 20

# Keep PID off because the custom lane follower uses direct pixel-error steering.
LANE_USE_PID = False

# Throttle values.
# The car should slow down on curves.
LANE_TEST_THROTTLE = 0.38
LANE_THROTTLE_MIN = 0.25
LANE_THROTTLE_MAX = 0.42
LANE_THROTTLE_TURN = 0.28

# LANE_HEADING_GAIN = 0.10         # start here; tune up if understeering, down if oscillating
# LANE_FAR_LOOKAHEAD_Y_FRACTION = 0.35   # far point higher in the ROI = looks further ahead


# ============================================================
# Safety / startup
# ============================================================

# Start in user mode so the car does not immediately drive itself.
WEB_INIT_MODE = "user"

# Disable unrelated optional features.
HAVE_MQTT_TELEMETRY = False
HAVE_PERFMON = False
HAVE_RGB_LED = False
USE_SSD1306_128_32 = False
STOP_SIGN_DETECTOR = False
SHOW_FPS = False




# # """ 
# # My CAR CONFIG 

# # This file is read by your car application's manage.py script to change the car
# # performance

# # If desired, all config overrides can be specified here. 
# # The update operation will not touch this file.
# # """

# # import os
# # 
# # 
# # import os
# # 
# # #
# # # FILE PATHS
# # #
# # CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
# # DATA_PATH = os.path.join(CAR_PATH, 'data')
# # 
# # 
# # #
# # # VEHICLE loop
# # #
# # DRIVE_LOOP_HZ = 20      # the vehicle loop will pause if faster than this speed.
# # MAX_LOOPS = None        # the vehicle loop can abort after this many iterations, when given a positive integer.
# # 
# # 
# # #
# # # CAMERA configuration
# # #
# # CAMERA_TYPE = "MOCK"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|D435|MOCK|IMAGE_LIST)
# # IMAGE_W = 320
# # IMAGE_H = 240
# # IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
# # CAMERA_FRAMERATE = DRIVE_LOOP_HZ
# # CAMERA_VFLIP = False
# # CAMERA_HFLIP = False
# # CAMERA_INDEX = 0  # used for 'WEBCAM' and 'CVCAM' when there is more than one camera connected
# # # For CSIC camera - If the camera is mounted in a rotated position, changing the below parameter will correct the output frame orientation
# # CSIC_CAM_GSTREAMER_FLIP_PARM = 0 # (0 => none , 4 => Flip horizontally, 6 => Flip vertically)
# # BGR2RGB = False  # true to convert from BRG format to RGB format; requires opencv
# # 
# # # For IMAGE_LIST camera
# # PATH_MASK = "~/mycar/data/tub_1_20-03-12/*.jpg"
# # 
# # 
# # #
# # # PCA9685, over rides only if needed, ie. TX2..
# # #
# # PCA9685_I2C_ADDR = 0x40     #I2C address, use i2cdetect to validate this number
# # PCA9685_I2C_BUSNUM = None   #None will auto detect, which is fine on the pi. But other platforms should specify the bus num.
# # 
# # 
# # #
# # # SSD1306_128_32
# # #
# # USE_SSD1306_128_32 = False    # Enable the SSD_1306 OLED Display
# # SSD1306_128_32_I2C_ROTATION = 0 # 0 = text is right-side up, 1 = rotated 90 degrees clockwise, 2 = 180 degrees (flipped), 3 = 270 degrees
# # SSD1306_RESOLUTION = 1 # 1 = 128x32; 2 = 128x64
# # 
# # 
# # #
# # # MEASURED ROBOT PROPERTIES
# # #
# # AXLE_LENGTH = 0.03     # length of axle; distance between left and right wheels in meters
# # WHEEL_BASE = 0.1       # distance between front and back wheels in meters
# # WHEEL_RADIUS = 0.0315  # radius of wheel in meters
# # MIN_SPEED = 0.1        # minimum speed in meters per second; speed below which car stalls
# # MAX_SPEED = 3.0        # maximum speed in meters per second; speed at maximum throttle (1.0)
# # MIN_THROTTLE = 0.1     # throttle (0 to 1.0) that corresponds to MIN_SPEED, throttle below which car stalls
# # MAX_STEERING_ANGLE = 3.141592653589793 / 4  # for car-like robot; maximum steering angle in radians (corresponding to tire angle at steering == -1)
# # 
# # 
# # #
# # # DRIVE_TRAIN_TYPE
# # # These options specify which chasis and motor setup you are using.
# # # See Actuators documentation https://docs.donkeycar.com/parts/actuators/
# # # for a detailed explanation of each drive train type and it's configuration.
# # # Choose one of the following and then update the related configuration section:
# # #
# # # "PWM_STEERING_THROTTLE" uses two PWM output pins to control a steering servo and an ESC, as in a standard RC car.
# # # "MM1" Robo HAT MM1 board
# # # "SERVO_HBRIDGE_2PIN" Servo for steering and HBridge motor driver in 2pin mode for motor
# # # "SERVO_HBRIDGE_3PIN" Servo for steering and HBridge motor driver in 3pin mode for motor
# # # "DC_STEER_THROTTLE" uses HBridge pwm to control one steering dc motor, and one drive wheel motor
# # # "DC_TWO_WHEEL" uses HBridge in 2-pin mode to control two drive motors, one on the left, and one on the right.
# # # "DC_TWO_WHEEL_L298N" using HBridge in 3-pin mode to control two drive motors, one of the left and one on the right.
# # # "MOCK" no drive train.  This can be used to test other features in a test rig.
# # # (deprecated) "SERVO_HBRIDGE_PWM" use ServoBlaster to output pwm control from the PiZero directly to control steering,
# # #                                  and HBridge for a drive motor.
# # # (deprecated) "PIGPIO_PWM" uses Raspberrys internal PWM
# # # (deprecated) "I2C_SERVO" uses PCA9685 servo controller to control a steering servo and an ESC, as in a standard RC car
# # #
# # DRIVE_TRAIN_TYPE = "PWM_STEERING_THROTTLE"
# # 
# # #
# # # PWM_STEERING_THROTTLE drivetrain configuration
# # #
# # # Drive train for RC car with a steering servo and ESC.
# # # Uses a PwmPin for steering (servo) and a second PwmPin for throttle (ESC)
# # # Base PWM Frequence is presumed to be 60hz; use PWM_xxxx_SCALE to adjust pulse with for non-standard PWM frequencies
# # #
# # PWM_STEERING_THROTTLE = {
# #     "PWM_STEERING_PIN": "PCA9685.1:40.1",   # PWM output pin for steering servo
# #     "PWM_STEERING_SCALE": 1.0,              # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
# #     "PWM_STEERING_INVERTED": False,         # True if hardware requires an inverted PWM pulse
# #     "PWM_THROTTLE_PIN": "PCA9685.1:40.0",   # PWM output pin for ESC
# #     "PWM_THROTTLE_SCALE": 1.0,              # used to compensate for PWM frequence differences from 60hz; NOT for increasing/limiting speed
# #     "PWM_THROTTLE_INVERTED": False,         # True if hardware requires an inverted PWM pulse
# #     "STEERING_LEFT_PWM": 460,               #pwm value for full left steering
# #     "STEERING_RIGHT_PWM": 290,              #pwm value for full right steering
# #     "THROTTLE_FORWARD_PWM": 500,            #pwm value for max forward throttle
# #     "THROTTLE_STOPPED_PWM": 370,            #pwm value for no movement
# #     "THROTTLE_REVERSE_PWM": 220,            #pwm value for max reverse throttle
# # }
# # 
# # #
# # # I2C_SERVO (deprecated in favor of PWM_STEERING_THROTTLE)
# # #
# # STEERING_CHANNEL = 1            #(deprecated) channel on the 9685 pwm board 0-15
# # STEERING_LEFT_PWM = 460         #pwm value for full left steering
# # STEERING_RIGHT_PWM = 290        #pwm value for full right steering
# # THROTTLE_CHANNEL = 0            #(deprecated) channel on the 9685 pwm board 0-15
# # THROTTLE_FORWARD_PWM = 500      #pwm value for max forward throttle
# # THROTTLE_STOPPED_PWM = 370      #pwm value for no movement
# # THROTTLE_REVERSE_PWM = 220      #pwm value for max reverse throttle
# # 
# # #
# # # PIGPIO_PWM (deprecated in favor of PWM_STEERING_THROTTLE)
# # #
# # STEERING_PWM_PIN = 13           #(deprecated) Pin numbering according to Broadcom numbers
# # STEERING_PWM_FREQ = 50          #Frequency for PWM
# # STEERING_PWM_INVERTED = False   #If PWM needs to be inverted
# # THROTTLE_PWM_PIN = 18           #(deprecated) Pin numbering according to Broadcom numbers
# # THROTTLE_PWM_FREQ = 50          #Frequency for PWM
# # THROTTLE_PWM_INVERTED = False   #If PWM needs to be inverted
# # 
# # #
# # # SERVO_HBRIDGE_2PIN drivetrain configuration
# # # - configures a steering servo and an HBridge in 2pin mode (2 pwm pins)
# # # - Servo takes a standard servo PWM pulse between 1 millisecond (fully reverse)
# # #   and 2 milliseconds (full forward) with 1.5ms being neutral.
# # # - the motor is controlled by two pwm pins,
# # #   one for forward and one for backward (reverse).
# # # - the pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the reverse pwm is 0 duty_cycle,
# # #   in backward mode, the forward pwm is 0 duty cycle.
# # # - both pwms are 0 duty cycle (LOW) to 'detach' motor and
# # #   and glide to a stop.
# # # - both pwms are full duty cycle (100% HIGH) to brake
# # #
# # # Pin specifier string format:
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # SERVO_HBRIDGE_2PIN = {
# #     "FWD_DUTY_PIN": "RPI_GPIO.BOARD.18",  # provides forward duty cycle to motor
# #     "BWD_DUTY_PIN": "RPI_GPIO.BOARD.16",  # provides reverse duty cycle to motor
# #     "PWM_STEERING_PIN": "RPI_GPIO.BOARD.33",       # provides servo pulse to steering servo
# #     "PWM_STEERING_SCALE": 1.0,        # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
# #     "PWM_STEERING_INVERTED": False,   # True if hardware requires an inverted PWM pulse
# #     "STEERING_LEFT_PWM": 460,         # pwm value for full left steering (use `donkey calibrate` to measure value for your car)
# #     "STEERING_RIGHT_PWM": 290,        # pwm value for full right steering (use `donkey calibrate` to measure value for your car)
# # }
# # 
# # #
# # # SERVO_HBRIDGE_3PIN drivetrain configuration
# # # - configures a steering servo and an HBridge in 3pin mode (2 ttl pins, 1 pwm pin)
# # # - Servo takes a standard servo PWM pulse between 1 millisecond (fully reverse)
# # #   and 2 milliseconds (full forward) with 1.5ms being neutral.
# # # - the motor is controlled by three pins,
# # #   one ttl output for forward, one ttl output
# # #   for backward (reverse) enable and one pwm pin
# # #   for motor power.
# # # - the pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the forward pin  is HIGH and the
# # #   backward pin is LOW,
# # # - in backward mode, the forward pin is LOW and the
# # #   backward pin is HIGH.
# # # - both forward and backward pins are LOW to 'detach' motor
# # #   and glide to a stop.
# # # - both forward and backward pins are HIGH to brake
# # #
# # # Pin specifier string format:
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # SERVO_HBRIDGE_3PIN = {
# #     "FWD_PIN": "RPI_GPIO.BOARD.18",   # ttl pin, high enables motor forward
# #     "BWD_PIN": "RPI_GPIO.BOARD.16",   # ttl pin, high enables motor reverse
# #     "DUTY_PIN": "RPI_GPIO.BOARD.35",  # provides duty cycle to motor
# #     "PWM_STEERING_PIN": "RPI_GPIO.BOARD.33",   # provides servo pulse to steering servo
# #     "PWM_STEERING_SCALE": 1.0,        # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
# #     "PWM_STEERING_INVERTED": False,   # True if hardware requires an inverted PWM pulse
# #     "STEERING_LEFT_PWM": 460,         # pwm value for full left steering (use `donkey calibrate` to measure value for your car)
# #     "STEERING_RIGHT_PWM": 290,        # pwm value for full right steering (use `donkey calibrate` to measure value for your car)
# # }
# # 
# # #
# # # DRIVETRAIN_TYPE == "SERVO_HBRIDGE_PWM" (deprecated in favor of SERVO_HBRIDGE_2PIN)
# # # - configures a steering servo and an HBridge in 2pin mode (2 pwm pins)
# # # - Uses ServoBlaster library, which is NOT installed by default, so
# # #   you will need to install it to make this work.
# # # - Servo takes a standard servo PWM pulse between 1 millisecond (fully reverse)
# # #   and 2 milliseconds (full forward) with 1.5ms being neutral.
# # # - the motor is controlled by two pwm pins,
# # #   one for forward and one for backward (reverse).
# # # - the pwm pins produce a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the reverse pwm is 0 duty_cycle,
# # #   in backward mode, the forward pwm is 0 duty cycle.
# # # - both pwms are 0 duty cycle (LOW) to 'detach' motor and
# # #   and glide to a stop.
# # # - both pwms are full duty cycle (100% HIGH) to brake
# # #
# # HBRIDGE_PIN_FWD = 18       # provides forward duty cycle to motor
# # HBRIDGE_PIN_BWD = 16       # provides reverse duty cycle to motor
# # STEERING_CHANNEL = 0       # PCA 9685 channel for steering control
# # STEERING_LEFT_PWM = 460    # pwm value for full left steering (use `donkey calibrate` to measure value for your car)
# # STEERING_RIGHT_PWM = 290   # pwm value for full right steering (use `donkey calibrate` to measure value for your car)
# # 
# # #
# # # DC_STEER_THROTTLE drivetrain with one motor as steering, one as drive
# # # - uses L298N type motor controller in two pin wiring
# # #   scheme utilizing two pwm pins per motor; one for
# # #   forward(or right) and one for reverse (or left)
# # #
# # # GPIO pin configuration for the DRIVE_TRAIN_TYPE=DC_STEER_THROTTLE
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # DC_STEER_THROTTLE = {
# #     "LEFT_DUTY_PIN": "RPI_GPIO.BOARD.18",   # pwm pin produces duty cycle for steering left
# #     "RIGHT_DUTY_PIN": "RPI_GPIO.BOARD.16",  # pwm pin produces duty cycle for steering right
# #     "FWD_DUTY_PIN": "RPI_GPIO.BOARD.15",    # pwm pin produces duty cycle for forward drive
# #     "BWD_DUTY_PIN": "RPI_GPIO.BOARD.13",    # pwm pin produces duty cycle for reverse drive
# # }
# # 
# # #
# # # DC_TWO_WHEEL drivetrain pin configuration
# # # - configures L298N_HBridge_2pin driver
# # # - two wheels as differential drive, left and right.
# # # - each wheel is controlled by two pwm pins,
# # #   one for forward and one for backward (reverse).
# # # - each pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the reverse pwm is 0 duty_cycle,
# # #   in backward mode, the forward pwm is 0 duty cycle.
# # # - both pwms are 0 duty cycle (LOW) to 'detach' motor and
# # #   and glide to a stop.
# # # - both pwms are full duty cycle (100% HIGH) to brake
# # #
# # # Pin specifier string format:
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # DC_TWO_WHEEL = {
# #     "LEFT_FWD_DUTY_PIN": "RPI_GPIO.BOARD.18",  # pwm pin produces duty cycle for left wheel forward
# #     "LEFT_BWD_DUTY_PIN": "RPI_GPIO.BOARD.16",  # pwm pin produces duty cycle for left wheel reverse
# #     "RIGHT_FWD_DUTY_PIN": "RPI_GPIO.BOARD.15", # pwm pin produces duty cycle for right wheel forward
# #     "RIGHT_BWD_DUTY_PIN": "RPI_GPIO.BOARD.13", # pwm pin produces duty cycle for right wheel reverse
# # }
# # 
# # #
# # # DC_TWO_WHEEL_L298N drivetrain pin configuration
# # # - configures L298N_HBridge_3pin driver
# # # - two wheels as differential drive, left and right.
# # # - each wheel is controlled by three pins,
# # #   one ttl output for forward, one ttl output
# # #   for backward (reverse) enable and one pwm pin
# # #   for motor power.
# # # - the pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the forward pin  is HIGH and the
# # #   backward pin is LOW,
# # # - in backward mode, the forward pin is LOW and the
# # #   backward pin is HIGH.
# # # - both forward and backward pins are LOW to 'detach' motor
# # #   and glide to a stop.
# # # - both forward and backward pins are HIGH to brake
# # #
# # # GPIO pin configuration for the DRIVE_TRAIN_TYPE=DC_TWO_WHEEL_L298N
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # DC_TWO_WHEEL_L298N = {
# #     "LEFT_FWD_PIN": "RPI_GPIO.BOARD.16",        # TTL output pin enables left wheel forward
# #     "LEFT_BWD_PIN": "RPI_GPIO.BOARD.18",        # TTL output pin enables left wheel reverse
# #     "LEFT_EN_DUTY_PIN": "RPI_GPIO.BOARD.22",    # PWM pin generates duty cycle for left motor speed
# # 
# #     "RIGHT_FWD_PIN": "RPI_GPIO.BOARD.15",       # TTL output pin enables right wheel forward
# #     "RIGHT_BWD_PIN": "RPI_GPIO.BOARD.13",       # TTL output pin enables right wheel reverse
# #     "RIGHT_EN_DUTY_PIN": "RPI_GPIO.BOARD.11",   # PWM pin generates duty cycle for right wheel speed
# # }
# # 
# # 
# # 
# # #
# # # Input controllers
# # #
# # #WEB CONTROL
# # WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))  # which port to listen on when making a web controller
# # WEB_INIT_MODE = "user"              # which control mode to start in. one of user|local_angle|local. Setting local will start in ai mode.
# # 
# # #JOYSTICK
# # USE_JOYSTICK_AS_DEFAULT = False      #when starting the manage.py, when True, will not require a --js option to use the joystick
# # JOYSTICK_MAX_THROTTLE = 0.5         #this scalar is multiplied with the -1 to 1 throttle value to limit the maximum throttle. This can help if you drop the controller or just don't need the full speed available.
# # JOYSTICK_STEERING_SCALE = 1.0       #some people want a steering that is less sensitve. This scalar is multiplied with the steering -1 to 1. It can be negative to reverse dir.
# # AUTO_RECORD_ON_THROTTLE = False     #if true, we will record whenever throttle is not zero. if false, you must manually toggle recording with some other trigger. Usually circle button on joystick.
# # CONTROLLER_TYPE = 'xbox'            #(ps3|ps4|xbox|pigpio_rc|nimbus|wiiu|F710|rc3|MM1|custom) custom will run the my_joystick.py controller written by the `donkey createjs` command
# # USE_NETWORKED_JS = False            #should we listen for remote joystick control over the network?
# # NETWORK_JS_SERVER_IP = None         #when listening for network joystick control, which ip is serving this information
# # JOYSTICK_DEADZONE = 0.01            # when non zero, this is the smallest throttle before recording triggered.
# # JOYSTICK_THROTTLE_DIR = -1.0         # use -1.0 to flip forward/backward, use 1.0 to use joystick's natural forward/backward
# # USE_FPV = False                     # send camera data to FPV webserver
# # JOYSTICK_DEVICE_FILE = "/dev/input/js0" # this is the unix file use to access the joystick.
# # 
# # 
# # #SOMBRERO
# # HAVE_SOMBRERO = False           #set to true when using the sombrero hat from the Donkeycar store. This will enable pwm on the hat.
# # 
# # #PIGPIO RC control
# # STEERING_RC_GPIO = 26
# # THROTTLE_RC_GPIO = 20
# # DATA_WIPER_RC_GPIO = 19
# # PIGPIO_STEERING_MID = 1500         # Adjust this value if your car cannot run in a straight line
# # PIGPIO_MAX_FORWARD = 2000          # Max throttle to go fowrward. The bigger the faster
# # PIGPIO_STOPPED_PWM = 1500
# # PIGPIO_MAX_REVERSE = 1000          # Max throttle to go reverse. The smaller the faster
# # PIGPIO_SHOW_STEERING_VALUE = False
# # PIGPIO_INVERT = False
# # PIGPIO_JITTER = 0.025   # threshold below which no signal is reported
# # 
# # 
# # # ROBOHAT MM1 controller
# # MM1_STEERING_MID = 1500         # Adjust this value if your car cannot run in a straight line
# # MM1_MAX_FORWARD = 2000          # Max throttle to go fowrward. The bigger the faster
# # MM1_STOPPED_PWM = 1500
# # MM1_MAX_REVERSE = 1000          # Max throttle to go reverse. The smaller the faster
# # MM1_SHOW_STEERING_VALUE = False
# # # Serial port
# # # -- Default Pi: '/dev/ttyS0'
# # # -- Jetson Nano: '/dev/ttyTHS1'
# # # -- Google coral: '/dev/ttymxc0'
# # # -- Windows: 'COM3', Arduino: '/dev/ttyACM0'
# # # -- MacOS/Linux:please use 'ls /dev/tty.*' to find the correct serial port for mm1
# # #  eg.'/dev/tty.usbmodemXXXXXX' and replace the port accordingly
# # MM1_SERIAL_PORT = '/dev/ttyS0'  # Serial Port for reading and sending MM1 data.
# # 
# # 
# # #
# # # LOGGING
# # #
# # HAVE_CONSOLE_LOGGING = True
# # LOGGING_LEVEL = 'INFO'          # (Python logging level) 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
# # LOGGING_FORMAT = '%(message)s'  # (Python logging format - https://docs.python.org/3/library/logging.html#formatter-objects
# # 
# # 
# # #
# # # MQTT TELEMETRY
# # #
# # HAVE_MQTT_TELEMETRY = False
# # TELEMETRY_DONKEY_NAME = 'my_robot1234'
# # TELEMETRY_MQTT_TOPIC_TEMPLATE = 'donkey/%s/telemetry'
# # TELEMETRY_MQTT_JSON_ENABLE = False
# # TELEMETRY_MQTT_BROKER_HOST = 'broker.hivemq.com'
# # TELEMETRY_MQTT_BROKER_PORT = 1883
# # TELEMETRY_PUBLISH_PERIOD = 1
# # TELEMETRY_LOGGING_ENABLE = True
# # TELEMETRY_LOGGING_LEVEL = 'INFO' # (Python logging level) 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
# # TELEMETRY_LOGGING_FORMAT = '%(message)s'  # (Python logging format - https://docs.python.org/3/library/logging.html#formatter-objects
# # TELEMETRY_DEFAULT_INPUTS = 'pilot/angle,pilot/throttle,recording'
# # TELEMETRY_DEFAULT_TYPES = 'float,float'
# # 
# # 
# # #
# # # PERFORMANCE MONITOR
# # #
# # HAVE_PERFMON = False
# # 
# # 
# # #
# # # RECORD OPTIONS
# # #
# # RECORD_DURING_AI = False        #normally we do not record during ai mode. Set this to true to get image and steering records for your Ai. Be careful not to use them to train.
# # AUTO_CREATE_NEW_TUB = False     #create a new tub (tub_YY_MM_DD) directory when recording or append records to data directory directly
# # 
# # 
# # #
# # # LED
# # #
# # HAVE_RGB_LED = False            #do you have an RGB LED like https://www.amazon.com/dp/B07BNRZWNF
# # LED_INVERT = False              #COMMON ANODE? Some RGB LED use common anode. like https://www.amazon.com/Xia-Fly-Tri-Color-Emitting-Diffused/dp/B07MYJQP8B
# # 
# # #LED board pin number for pwm outputs
# # #These are physical pinouts. See: https://www.raspberrypi-spy.co.uk/2012/06/simple-guide-to-the-rpi-gpio-header-and-pins/
# # LED_PIN_R = 12
# # LED_PIN_G = 10
# # LED_PIN_B = 16
# # 
# # #LED status color, 0-100
# # LED_R = 0
# # LED_G = 0
# # LED_B = 1
# # 
# # #LED Color for record count indicator
# # REC_COUNT_ALERT = 1000          #how many records before blinking alert
# # REC_COUNT_ALERT_CYC = 15        #how many cycles of 1/20 of a second to blink per REC_COUNT_ALERT records
# # REC_COUNT_ALERT_BLINK_RATE = 0.4 #how fast to blink the led in seconds on/off
# # 
# # #first number is record count, second tuple is color ( r, g, b) (0-100)
# # #when record count exceeds that number, the color will be used
# # RECORD_ALERT_COLOR_ARR = [ (0, (1, 1, 1)),
# #             (3000, (5, 5, 5)),
# #             (5000, (5, 2, 0)),
# #             (10000, (0, 5, 0)),
# #             (15000, (0, 5, 5)),
# #             (20000, (0, 0, 5)), ]
# # 
# # #LED status color, 0-100, for model reloaded alert
# # MODEL_RELOADED_LED_R = 100
# # MODEL_RELOADED_LED_G = 0
# # MODEL_RELOADED_LED_B = 0
# # 
# # 
# # #
# # # DonkeyGym
# # #
# # # Only on Ubuntu linux, you can use the simulator as a virtual donkey and
# # # issue the same python manage.py drive command as usual, but have them control a virtual car.
# # # This enables that, and sets the path to the simualator and the environment.
# # # You will want to download the simulator binary from: https://github.com/tawnkramer/donkey_gym/releases/download/v18.9/DonkeySimLinux.zip
# # # then extract that and modify DONKEY_SIM_PATH.
# # DONKEY_GYM = False
# # DONKEY_SIM_PATH = "path to sim" #"/home/tkramer/projects/sdsandbox/sdsim/build/DonkeySimLinux/donkey_sim.x86_64" when racing on virtual-race-league use "remote", or user "remote" when you want to start the sim manually first.
# # DONKEY_GYM_ENV_NAME = "donkey-generated-track-v0" # ("donkey-generated-track-v0"|"donkey-generated-roads-v0"|"donkey-warehouse-v0"|"donkey-avc-sparkfun-v0")
# # GYM_CONF = { "body_style" : "donkey", "body_rgb" : (128, 128, 128), "car_name" : "car", "font_size" : 100} # body style(donkey|bare|car01) body rgb 0-255
# # GYM_CONF["racer_name"] = "Your Name"
# # GYM_CONF["country"] = "Place"
# # GYM_CONF["bio"] = "I race robots."
# # 
# # SIM_HOST = "127.0.0.1"              # when racing on virtual-race-league use host "trainmydonkey.com"
# # SIM_ARTIFICIAL_LATENCY = 0          # this is the millisecond latency in controls. Can use useful in emulating the delay when useing a remote server. values of 100 to 400 probably reasonable.
# # 
# # # Save info from Simulator (pln)
# # SIM_RECORD_LOCATION = False
# # SIM_RECORD_GYROACCEL= False
# # SIM_RECORD_VELOCITY = False
# # SIM_RECORD_LIDAR = False
# # 
# # # publish camera over network on TCP socket
# # # This is used to create a tcp service to publish the camera feed
# # PUB_CAMERA_IMAGES = False
# # 
# # 
# # #
# # # AI Overrides
# # #
# # # Launch mode: override AI at launch time (transition from user to Auto pilot).
# # AI_LAUNCH_DURATION = 0.0            # the ai will output throttle for this many seconds
# # AI_LAUNCH_THROTTLE = 0.0            # the ai will output this throttle value
# # AI_LAUNCH_ENABLE_BUTTON = 'R2'      # this keypress will enable this boost. It must be enabled before each use to prevent accidental trigger.
# # AI_LAUNCH_KEEP_ENABLED = False      # when False ( default) you will need to hit the AI_LAUNCH_ENABLE_BUTTON for each use. This is safest. When this True, is active on each trip into "local" ai mode.
# # 
# # # throttle scaling: scale the output of the throttle of the ai pilot for all model types.
# # AI_THROTTLE_MULT = 1.0              # this multiplier will scale every throttle value for all output from NN models
# # 
# # 
# # #
# # # Intel Realsense D435 and D435i depth sensing camera
# # #
# # REALSENSE_D435_RGB = True       # True to capture RGB image
# # REALSENSE_D435_DEPTH = False    # True to capture depth as image array
# # REALSENSE_D435_IMU = False      # True to capture IMU data (D435i only)
# # REALSENSE_D435_ID = None        # serial number of camera or None if you only have one camera (it will autodetect)
# # 
# # 
# # #
# # # Stop Sign Detector
# # #
# # STOP_SIGN_DETECTOR = False
# # STOP_SIGN_MIN_SCORE = 0.2
# # STOP_SIGN_SHOW_BOUNDING_BOX = True
# # STOP_SIGN_MAX_REVERSE_COUNT = 10    # How many times should the car reverse when detected a stop sign, set to 0 to disable reversing
# # STOP_SIGN_REVERSE_THROTTLE = -0.5     # Throttle during reversing when detected a stop sign
# # 
# # #
# # # Frames/Second counter
# # #
# # SHOW_FPS = False
# # FPS_DEBUG_INTERVAL = 10    # the interval in seconds for printing the frequency info into the shell
# # 
# # #
# # # computer vision template
# # #
# # configure which part is used as the autopilot - change to use your own autopilot
# # Custom lane center follower
# CV_CONTROLLER_MODULE = "lane_center_follower"
# CV_CONTROLLER_CLASS = "LaneCenterFollower"
# CV_CONTROLLER_INPUTS = ["cam/image_array"]
# CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]
# CV_CONTROLLER_CONDITION = "run_pilot"

# OVERLAY_IMAGE = True
# # 
# # # LineFollower - line color and detection area
# # SCAN_Y = 100          # num pixels from the top to start horiz scan
# # SCAN_HEIGHT = 20      # num pixels high to grab from horiz scan
# # COLOR_THRESHOLD_LOW  = (0, 50, 50)    # HSV dark yellow (opencv HSV hue value is 0..179, saturation and value are both 0..255)
# # COLOR_THRESHOLD_HIGH = (50, 255, 255) # HSV light yellow (opencv HSV hue value is 0..179, saturation and value are both 0..255)
# # 
# # # LineFollower - target (expected) line position and detection thresholds
# # TARGET_PIXEL = None   # In not None, then this is the expected horizontal position in pixels of the yellow line.
# #                       # If None, then detect the position yellow line at startup;
# #                       # so this assumes you have positioned the car prior to starting.
# #                       # Alternatively set this to IMAGE_W / 2 to follow middle line
# # TARGET_THRESHOLD = 10 # number of pixels from TARGET_PIXEL that vehicle must be pointing
# #                       # before a steering change will be made; this prevents algorithm
# #                       # from being too twitchy when it is on or near the line.
# # CONFIDENCE_THRESHOLD = 0.0015   # The fraction of total sampled pixels that must be yellow in the sample slice.
# #                                 # The sample slice will have SCAN_HEIGHT pixels and the total number
# #                                 # of sampled pixels is IMAGE_W x SCAN_HEIGHT, so if you want to make sure
# #                                 # that all the pixels in the sample slice are yellow, then the confidence
# #                                 # threshold should be SCAN_HEIGHT / (IMAGE_W x SCAN_HEIGHT) or (1 / IMAGE_W).
# #                                 # if you want half of the pixels in the slice to match hten (1 / IMAGE_W) / 2.
# #                                 # If you keep getting `No line detected` logs in the console then you
# #                                 # may want to lower the threshold.
# # 
# # # LineFollower - throttle step controller; increase throttle on straights, descrease on turns
# # THROTTLE_MAX = 0.3    # maximum throttle value the controller will produce
# # THROTTLE_MIN = 0.15   # minimum throttle value the controller will produce
# # THROTTLE_INITIAL = THROTTLE_MIN  # initial throttle value
# # THROTTLE_STEP = 0.05  # how much to change throttle when off the line
# # 
# # # These three PID constants are crucial to the way the car drives. If you are tuning them
# # # start by setting the others zero and focus on first Kp, then Kd, and then Ki.
# # PID_P = -0.01         # proportional mult for PID path follower
# # PID_I = 0.000         # integral mult for PID path follower
# # PID_D = -0.0001       # differential mult for PID path follower
# # 
# # PID_P_DELTA = 0.005   # amount the inc/dec function will change the P value
# # PID_D_DELTA = 0.00005 # amount the inc/dec function will change the D value
# # 
# # OVERLAY_IMAGE = True  # True to draw computer vision overlay on camera image in web ui
# #                       # NOTE: this does not affect what is saved to the data
# # 
# # 
# # #
# # # Assign path follow functions to buttons.
# # # You can use game pad buttons OR web ui buttons ('web/w1' to 'web/w5')
# # # Use None use the game controller default
# # # NOTE: the cross button is already reserved for the emergency stop
# # #
# # TOGGLE_RECORDING_BTN = "option" # button to toggle recording mode
# # INC_PID_D_BTN = None            # button to change PID 'D' constant by PID_D_DELTA
# # DEC_PID_D_BTN = None            # button to change PID 'D' constant by -PID_D_DELTA
# # INC_PID_P_BTN = "R2"            # button to change PID 'P' constant by PID_P_DELTA
# # DEC_PID_P_BTN = "L2"            # button to change PID 'P' constant by -PID_P_DELTA
# # 

# # # ============================================================
# # # Lane boundary CV experiment settings
# # # Copied from the working path_follower car where needed
# # # ============================================================

# # # Start with MOCK camera so the app boots before we touch OAK-D.
# # # Later we will change this to OAKD.
# # CAMERA_TYPE = "OAKD"
# # CAMERA_INDEX = 0

# # # Use the real VESC drivetrain, not the default PCA9685 PWM drivetrain.
# # DRIVE_TRAIN_TYPE = "VESC"

# # VESC_MAX_SPEED_PERCENT = 0.2
# # VESC_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
# # VESC_HAS_SENSOR = True
# # VESC_START_HEARTBEAT = True
# # VESC_BAUDRATE = 115200
# # VESC_TIMEOUT = 0.05
# # VESC_STEERING_SCALE = 0.5
# # VESC_STEERING_OFFSET = 0.45

# # # Joystick settings from working path_follower setup.
# # CONTROLLER_TYPE = "custom"
# # JOYSTICK_DEADZONE = 0.1
# # JOYSTICK_THROTTLE_DIR = -1.0
# # JOYSTICK_DEVICE_FILE = "/dev/input/js0"

# # # Keep simulator off.
# # DONKEY_GYM = False

# # # Tub behavior.
# # AUTO_CREATE_NEW_TUB = True

# # # Start safe.
# # WEB_INIT_MODE = "user"







############################################################
############################################################
## myconfig_russel.py
############################################################
############################################################

# # """ 
# # My CAR CONFIG 

# # This file is read by your car application's manage.py script to change the car
# # performance

# # If desired, all config overrides can be specified here. 
# # The update operation will not touch this file.
# # """

# # import os
# # 
# # 
# # import os
# # 
# # #
# # # FILE PATHS
# # #
# # CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
# # DATA_PATH = os.path.join(CAR_PATH, 'data')
# # 
# # 
# # #
# # # VEHICLE loop
# # #
# # DRIVE_LOOP_HZ = 20      # the vehicle loop will pause if faster than this speed.
# # MAX_LOOPS = None        # the vehicle loop can abort after this many iterations, when given a positive integer.
# # 
# # 
# # #
# # # CAMERA configuration
# # #
# # CAMERA_TYPE = "MOCK"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|D435|MOCK|IMAGE_LIST)
# # IMAGE_W = 320
# # IMAGE_H = 240
# # IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
# # CAMERA_FRAMERATE = DRIVE_LOOP_HZ
# # CAMERA_VFLIP = False
# # CAMERA_HFLIP = False
# # CAMERA_INDEX = 0  # used for 'WEBCAM' and 'CVCAM' when there is more than one camera connected
# # # For CSIC camera - If the camera is mounted in a rotated position, changing the below parameter will correct the output frame orientation
# # CSIC_CAM_GSTREAMER_FLIP_PARM = 0 # (0 => none , 4 => Flip horizontally, 6 => Flip vertically)
# # BGR2RGB = False  # true to convert from BRG format to RGB format; requires opencv
# # 
# # # For IMAGE_LIST camera
# # PATH_MASK = "~/mycar/data/tub_1_20-03-12/*.jpg"
# # 
# # 
# # #
# # # PCA9685, over rides only if needed, ie. TX2..
# # #
# # PCA9685_I2C_ADDR = 0x40     #I2C address, use i2cdetect to validate this number
# # PCA9685_I2C_BUSNUM = None   #None will auto detect, which is fine on the pi. But other platforms should specify the bus num.
# # 
# # 
# # #
# # # SSD1306_128_32
# # #
# # USE_SSD1306_128_32 = False    # Enable the SSD_1306 OLED Display
# # SSD1306_128_32_I2C_ROTATION = 0 # 0 = text is right-side up, 1 = rotated 90 degrees clockwise, 2 = 180 degrees (flipped), 3 = 270 degrees
# # SSD1306_RESOLUTION = 1 # 1 = 128x32; 2 = 128x64
# # 
# # 
# # #
# # # MEASURED ROBOT PROPERTIES
# # #
# # AXLE_LENGTH = 0.03     # length of axle; distance between left and right wheels in meters
# # WHEEL_BASE = 0.1       # distance between front and back wheels in meters
# # WHEEL_RADIUS = 0.0315  # radius of wheel in meters
# # MIN_SPEED = 0.1        # minimum speed in meters per second; speed below which car stalls
# # MAX_SPEED = 3.0        # maximum speed in meters per second; speed at maximum throttle (1.0)
# # MIN_THROTTLE = 0.1     # throttle (0 to 1.0) that corresponds to MIN_SPEED, throttle below which car stalls
# # MAX_STEERING_ANGLE = 3.141592653589793 / 4  # for car-like robot; maximum steering angle in radians (corresponding to tire angle at steering == -1)
# # 
# # 
# # #
# # # DRIVE_TRAIN_TYPE
# # # These options specify which chasis and motor setup you are using.
# # # See Actuators documentation https://docs.donkeycar.com/parts/actuators/
# # # for a detailed explanation of each drive train type and it's configuration.
# # # Choose one of the following and then update the related configuration section:
# # #
# # # "PWM_STEERING_THROTTLE" uses two PWM output pins to control a steering servo and an ESC, as in a standard RC car.
# # # "MM1" Robo HAT MM1 board
# # # "SERVO_HBRIDGE_2PIN" Servo for steering and HBridge motor driver in 2pin mode for motor
# # # "SERVO_HBRIDGE_3PIN" Servo for steering and HBridge motor driver in 3pin mode for motor
# # # "DC_STEER_THROTTLE" uses HBridge pwm to control one steering dc motor, and one drive wheel motor
# # # "DC_TWO_WHEEL" uses HBridge in 2-pin mode to control two drive motors, one on the left, and one on the right.
# # # "DC_TWO_WHEEL_L298N" using HBridge in 3-pin mode to control two drive motors, one of the left and one on the right.
# # # "MOCK" no drive train.  This can be used to test other features in a test rig.
# # # (deprecated) "SERVO_HBRIDGE_PWM" use ServoBlaster to output pwm control from the PiZero directly to control steering,
# # #                                  and HBridge for a drive motor.
# # # (deprecated) "PIGPIO_PWM" uses Raspberrys internal PWM
# # # (deprecated) "I2C_SERVO" uses PCA9685 servo controller to control a steering servo and an ESC, as in a standard RC car
# # #
# # DRIVE_TRAIN_TYPE = "PWM_STEERING_THROTTLE"
# # 
# # #
# # # PWM_STEERING_THROTTLE drivetrain configuration
# # #
# # # Drive train for RC car with a steering servo and ESC.
# # # Uses a PwmPin for steering (servo) and a second PwmPin for throttle (ESC)
# # # Base PWM Frequence is presumed to be 60hz; use PWM_xxxx_SCALE to adjust pulse with for non-standard PWM frequencies
# # #
# # PWM_STEERING_THROTTLE = {
# #     "PWM_STEERING_PIN": "PCA9685.1:40.1",   # PWM output pin for steering servo
# #     "PWM_STEERING_SCALE": 1.0,              # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
# #     "PWM_STEERING_INVERTED": False,         # True if hardware requires an inverted PWM pulse
# #     "PWM_THROTTLE_PIN": "PCA9685.1:40.0",   # PWM output pin for ESC
# #     "PWM_THROTTLE_SCALE": 1.0,              # used to compensate for PWM frequence differences from 60hz; NOT for increasing/limiting speed
# #     "PWM_THROTTLE_INVERTED": False,         # True if hardware requires an inverted PWM pulse
# #     "STEERING_LEFT_PWM": 460,               #pwm value for full left steering
# #     "STEERING_RIGHT_PWM": 290,              #pwm value for full right steering
# #     "THROTTLE_FORWARD_PWM": 500,            #pwm value for max forward throttle
# #     "THROTTLE_STOPPED_PWM": 370,            #pwm value for no movement
# #     "THROTTLE_REVERSE_PWM": 220,            #pwm value for max reverse throttle
# # }
# # 
# # #
# # # I2C_SERVO (deprecated in favor of PWM_STEERING_THROTTLE)
# # #
# # STEERING_CHANNEL = 1            #(deprecated) channel on the 9685 pwm board 0-15
# # STEERING_LEFT_PWM = 460         #pwm value for full left steering
# # STEERING_RIGHT_PWM = 290        #pwm value for full right steering
# # THROTTLE_CHANNEL = 0            #(deprecated) channel on the 9685 pwm board 0-15
# # THROTTLE_FORWARD_PWM = 500      #pwm value for max forward throttle
# # THROTTLE_STOPPED_PWM = 370      #pwm value for no movement
# # THROTTLE_REVERSE_PWM = 220      #pwm value for max reverse throttle
# # 
# # #
# # # PIGPIO_PWM (deprecated in favor of PWM_STEERING_THROTTLE)
# # #
# # STEERING_PWM_PIN = 13           #(deprecated) Pin numbering according to Broadcom numbers
# # STEERING_PWM_FREQ = 50          #Frequency for PWM
# # STEERING_PWM_INVERTED = False   #If PWM needs to be inverted
# # THROTTLE_PWM_PIN = 18           #(deprecated) Pin numbering according to Broadcom numbers
# # THROTTLE_PWM_FREQ = 50          #Frequency for PWM
# # THROTTLE_PWM_INVERTED = False   #If PWM needs to be inverted
# # 
# # #
# # # SERVO_HBRIDGE_2PIN drivetrain configuration
# # # - configures a steering servo and an HBridge in 2pin mode (2 pwm pins)
# # # - Servo takes a standard servo PWM pulse between 1 millisecond (fully reverse)
# # #   and 2 milliseconds (full forward) with 1.5ms being neutral.
# # # - the motor is controlled by two pwm pins,
# # #   one for forward and one for backward (reverse).
# # # - the pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the reverse pwm is 0 duty_cycle,
# # #   in backward mode, the forward pwm is 0 duty cycle.
# # # - both pwms are 0 duty cycle (LOW) to 'detach' motor and
# # #   and glide to a stop.
# # # - both pwms are full duty cycle (100% HIGH) to brake
# # #
# # # Pin specifier string format:
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # SERVO_HBRIDGE_2PIN = {
# #     "FWD_DUTY_PIN": "RPI_GPIO.BOARD.18",  # provides forward duty cycle to motor
# #     "BWD_DUTY_PIN": "RPI_GPIO.BOARD.16",  # provides reverse duty cycle to motor
# #     "PWM_STEERING_PIN": "RPI_GPIO.BOARD.33",       # provides servo pulse to steering servo
# #     "PWM_STEERING_SCALE": 1.0,        # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
# #     "PWM_STEERING_INVERTED": False,   # True if hardware requires an inverted PWM pulse
# #     "STEERING_LEFT_PWM": 460,         # pwm value for full left steering (use `donkey calibrate` to measure value for your car)
# #     "STEERING_RIGHT_PWM": 290,        # pwm value for full right steering (use `donkey calibrate` to measure value for your car)
# # }
# # 
# # #
# # # SERVO_HBRIDGE_3PIN drivetrain configuration
# # # - configures a steering servo and an HBridge in 3pin mode (2 ttl pins, 1 pwm pin)
# # # - Servo takes a standard servo PWM pulse between 1 millisecond (fully reverse)
# # #   and 2 milliseconds (full forward) with 1.5ms being neutral.
# # # - the motor is controlled by three pins,
# # #   one ttl output for forward, one ttl output
# # #   for backward (reverse) enable and one pwm pin
# # #   for motor power.
# # # - the pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the forward pin  is HIGH and the
# # #   backward pin is LOW,
# # # - in backward mode, the forward pin is LOW and the
# # #   backward pin is HIGH.
# # # - both forward and backward pins are LOW to 'detach' motor
# # #   and glide to a stop.
# # # - both forward and backward pins are HIGH to brake
# # #
# # # Pin specifier string format:
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # SERVO_HBRIDGE_3PIN = {
# #     "FWD_PIN": "RPI_GPIO.BOARD.18",   # ttl pin, high enables motor forward
# #     "BWD_PIN": "RPI_GPIO.BOARD.16",   # ttl pin, high enables motor reverse
# #     "DUTY_PIN": "RPI_GPIO.BOARD.35",  # provides duty cycle to motor
# #     "PWM_STEERING_PIN": "RPI_GPIO.BOARD.33",   # provides servo pulse to steering servo
# #     "PWM_STEERING_SCALE": 1.0,        # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
# #     "PWM_STEERING_INVERTED": False,   # True if hardware requires an inverted PWM pulse
# #     "STEERING_LEFT_PWM": 460,         # pwm value for full left steering (use `donkey calibrate` to measure value for your car)
# #     "STEERING_RIGHT_PWM": 290,        # pwm value for full right steering (use `donkey calibrate` to measure value for your car)
# # }
# # 
# # #
# # # DRIVETRAIN_TYPE == "SERVO_HBRIDGE_PWM" (deprecated in favor of SERVO_HBRIDGE_2PIN)
# # # - configures a steering servo and an HBridge in 2pin mode (2 pwm pins)
# # # - Uses ServoBlaster library, which is NOT installed by default, so
# # #   you will need to install it to make this work.
# # # - Servo takes a standard servo PWM pulse between 1 millisecond (fully reverse)
# # #   and 2 milliseconds (full forward) with 1.5ms being neutral.
# # # - the motor is controlled by two pwm pins,
# # #   one for forward and one for backward (reverse).
# # # - the pwm pins produce a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the reverse pwm is 0 duty_cycle,
# # #   in backward mode, the forward pwm is 0 duty cycle.
# # # - both pwms are 0 duty cycle (LOW) to 'detach' motor and
# # #   and glide to a stop.
# # # - both pwms are full duty cycle (100% HIGH) to brake
# # #
# # HBRIDGE_PIN_FWD = 18       # provides forward duty cycle to motor
# # HBRIDGE_PIN_BWD = 16       # provides reverse duty cycle to motor
# # STEERING_CHANNEL = 0       # PCA 9685 channel for steering control
# # STEERING_LEFT_PWM = 460    # pwm value for full left steering (use `donkey calibrate` to measure value for your car)
# # STEERING_RIGHT_PWM = 290   # pwm value for full right steering (use `donkey calibrate` to measure value for your car)
# # 
# # #
# # # DC_STEER_THROTTLE drivetrain with one motor as steering, one as drive
# # # - uses L298N type motor controller in two pin wiring
# # #   scheme utilizing two pwm pins per motor; one for
# # #   forward(or right) and one for reverse (or left)
# # #
# # # GPIO pin configuration for the DRIVE_TRAIN_TYPE=DC_STEER_THROTTLE
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # DC_STEER_THROTTLE = {
# #     "LEFT_DUTY_PIN": "RPI_GPIO.BOARD.18",   # pwm pin produces duty cycle for steering left
# #     "RIGHT_DUTY_PIN": "RPI_GPIO.BOARD.16",  # pwm pin produces duty cycle for steering right
# #     "FWD_DUTY_PIN": "RPI_GPIO.BOARD.15",    # pwm pin produces duty cycle for forward drive
# #     "BWD_DUTY_PIN": "RPI_GPIO.BOARD.13",    # pwm pin produces duty cycle for reverse drive
# # }
# # 
# # #
# # # DC_TWO_WHEEL drivetrain pin configuration
# # # - configures L298N_HBridge_2pin driver
# # # - two wheels as differential drive, left and right.
# # # - each wheel is controlled by two pwm pins,
# # #   one for forward and one for backward (reverse).
# # # - each pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the reverse pwm is 0 duty_cycle,
# # #   in backward mode, the forward pwm is 0 duty cycle.
# # # - both pwms are 0 duty cycle (LOW) to 'detach' motor and
# # #   and glide to a stop.
# # # - both pwms are full duty cycle (100% HIGH) to brake
# # #
# # # Pin specifier string format:
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # DC_TWO_WHEEL = {
# #     "LEFT_FWD_DUTY_PIN": "RPI_GPIO.BOARD.18",  # pwm pin produces duty cycle for left wheel forward
# #     "LEFT_BWD_DUTY_PIN": "RPI_GPIO.BOARD.16",  # pwm pin produces duty cycle for left wheel reverse
# #     "RIGHT_FWD_DUTY_PIN": "RPI_GPIO.BOARD.15", # pwm pin produces duty cycle for right wheel forward
# #     "RIGHT_BWD_DUTY_PIN": "RPI_GPIO.BOARD.13", # pwm pin produces duty cycle for right wheel reverse
# # }
# # 
# # #
# # # DC_TWO_WHEEL_L298N drivetrain pin configuration
# # # - configures L298N_HBridge_3pin driver
# # # - two wheels as differential drive, left and right.
# # # - each wheel is controlled by three pins,
# # #   one ttl output for forward, one ttl output
# # #   for backward (reverse) enable and one pwm pin
# # #   for motor power.
# # # - the pwm pin produces a duty cycle from 0 (completely LOW)
# # #   to 1 (100% completely high), which is proportional to the
# # #   amount of power delivered to the motor.
# # # - in forward mode, the forward pin  is HIGH and the
# # #   backward pin is LOW,
# # # - in backward mode, the forward pin is LOW and the
# # #   backward pin is HIGH.
# # # - both forward and backward pins are LOW to 'detach' motor
# # #   and glide to a stop.
# # # - both forward and backward pins are HIGH to brake
# # #
# # # GPIO pin configuration for the DRIVE_TRAIN_TYPE=DC_TWO_WHEEL_L298N
# # # - use RPI_GPIO for RPi/Nano header pin output
# # #   - use BOARD for board pin numbering
# # #   - use BCM for Broadcom GPIO numbering
# # #   - for example "RPI_GPIO.BOARD.18"
# # # - use PIPGIO for RPi header pin output using pigpio server
# # #   - must use BCM (broadcom) pin numbering scheme
# # #   - for example, "PIGPIO.BCM.13"
# # # - use PCA9685 for PCA9685 pin output
# # #   - include colon separated I2C channel and address
# # #   - for example "PCA9685.1:40.13"
# # # - RPI_GPIO, PIGPIO and PCA9685 can be mixed arbitrarily,
# # #   although it is discouraged to mix RPI_GPIO and PIGPIO.
# # #
# # DC_TWO_WHEEL_L298N = {
# #     "LEFT_FWD_PIN": "RPI_GPIO.BOARD.16",        # TTL output pin enables left wheel forward
# #     "LEFT_BWD_PIN": "RPI_GPIO.BOARD.18",        # TTL output pin enables left wheel reverse
# #     "LEFT_EN_DUTY_PIN": "RPI_GPIO.BOARD.22",    # PWM pin generates duty cycle for left motor speed
# # 
# #     "RIGHT_FWD_PIN": "RPI_GPIO.BOARD.15",       # TTL output pin enables right wheel forward
# #     "RIGHT_BWD_PIN": "RPI_GPIO.BOARD.13",       # TTL output pin enables right wheel reverse
# #     "RIGHT_EN_DUTY_PIN": "RPI_GPIO.BOARD.11",   # PWM pin generates duty cycle for right wheel speed
# # }
# # 
# # 
# # 
# # #
# # # Input controllers
# # #
# # #WEB CONTROL
# # WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))  # which port to listen on when making a web controller
# # WEB_INIT_MODE = "user"              # which control mode to start in. one of user|local_angle|local. Setting local will start in ai mode.
# # 
# # #JOYSTICK
# # USE_JOYSTICK_AS_DEFAULT = False      #when starting the manage.py, when True, will not require a --js option to use the joystick
# # JOYSTICK_MAX_THROTTLE = 0.5         #this scalar is multiplied with the -1 to 1 throttle value to limit the maximum throttle. This can help if you drop the controller or just don't need the full speed available.
# # JOYSTICK_STEERING_SCALE = 1.0       #some people want a steering that is less sensitve. This scalar is multiplied with the steering -1 to 1. It can be negative to reverse dir.
# # AUTO_RECORD_ON_THROTTLE = False     #if true, we will record whenever throttle is not zero. if false, you must manually toggle recording with some other trigger. Usually circle button on joystick.
# # CONTROLLER_TYPE = 'xbox'            #(ps3|ps4|xbox|pigpio_rc|nimbus|wiiu|F710|rc3|MM1|custom) custom will run the my_joystick.py controller written by the `donkey createjs` command
# # USE_NETWORKED_JS = False            #should we listen for remote joystick control over the network?
# # NETWORK_JS_SERVER_IP = None         #when listening for network joystick control, which ip is serving this information
# # JOYSTICK_DEADZONE = 0.01            # when non zero, this is the smallest throttle before recording triggered.
# # JOYSTICK_THROTTLE_DIR = -1.0         # use -1.0 to flip forward/backward, use 1.0 to use joystick's natural forward/backward
# # USE_FPV = False                     # send camera data to FPV webserver
# # JOYSTICK_DEVICE_FILE = "/dev/input/js0" # this is the unix file use to access the joystick.
# # 
# # 
# # #SOMBRERO
# # HAVE_SOMBRERO = False           #set to true when using the sombrero hat from the Donkeycar store. This will enable pwm on the hat.
# # 
# # #PIGPIO RC control
# # STEERING_RC_GPIO = 26
# # THROTTLE_RC_GPIO = 20
# # DATA_WIPER_RC_GPIO = 19
# # PIGPIO_STEERING_MID = 1500         # Adjust this value if your car cannot run in a straight line
# # PIGPIO_MAX_FORWARD = 2000          # Max throttle to go fowrward. The bigger the faster
# # PIGPIO_STOPPED_PWM = 1500
# # PIGPIO_MAX_REVERSE = 1000          # Max throttle to go reverse. The smaller the faster
# # PIGPIO_SHOW_STEERING_VALUE = False
# # PIGPIO_INVERT = False
# # PIGPIO_JITTER = 0.025   # threshold below which no signal is reported
# # 
# # 
# # # ROBOHAT MM1 controller
# # MM1_STEERING_MID = 1500         # Adjust this value if your car cannot run in a straight line
# # MM1_MAX_FORWARD = 2000          # Max throttle to go fowrward. The bigger the faster
# # MM1_STOPPED_PWM = 1500
# # MM1_MAX_REVERSE = 1000          # Max throttle to go reverse. The smaller the faster
# # MM1_SHOW_STEERING_VALUE = False
# # # Serial port
# # # -- Default Pi: '/dev/ttyS0'
# # # -- Jetson Nano: '/dev/ttyTHS1'
# # # -- Google coral: '/dev/ttymxc0'
# # # -- Windows: 'COM3', Arduino: '/dev/ttyACM0'
# # # -- MacOS/Linux:please use 'ls /dev/tty.*' to find the correct serial port for mm1
# # #  eg.'/dev/tty.usbmodemXXXXXX' and replace the port accordingly
# # MM1_SERIAL_PORT = '/dev/ttyS0'  # Serial Port for reading and sending MM1 data.
# # 
# # 
# # #
# # # LOGGING
# # #
# # HAVE_CONSOLE_LOGGING = True
# # LOGGING_LEVEL = 'INFO'          # (Python logging level) 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
# # LOGGING_FORMAT = '%(message)s'  # (Python logging format - https://docs.python.org/3/library/logging.html#formatter-objects
# # 
# # 
# # #
# # # MQTT TELEMETRY
# # #
# # HAVE_MQTT_TELEMETRY = False
# # TELEMETRY_DONKEY_NAME = 'my_robot1234'
# # TELEMETRY_MQTT_TOPIC_TEMPLATE = 'donkey/%s/telemetry'
# # TELEMETRY_MQTT_JSON_ENABLE = False
# # TELEMETRY_MQTT_BROKER_HOST = 'broker.hivemq.com'
# # TELEMETRY_MQTT_BROKER_PORT = 1883
# # TELEMETRY_PUBLISH_PERIOD = 1
# # TELEMETRY_LOGGING_ENABLE = True
# # TELEMETRY_LOGGING_LEVEL = 'INFO' # (Python logging level) 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
# # TELEMETRY_LOGGING_FORMAT = '%(message)s'  # (Python logging format - https://docs.python.org/3/library/logging.html#formatter-objects
# # TELEMETRY_DEFAULT_INPUTS = 'pilot/angle,pilot/throttle,recording'
# # TELEMETRY_DEFAULT_TYPES = 'float,float'
# # 
# # 
# # #
# # # PERFORMANCE MONITOR
# # #
# # HAVE_PERFMON = False
# # 
# # 
# # #
# # # RECORD OPTIONS
# # #
# # RECORD_DURING_AI = False        #normally we do not record during ai mode. Set this to true to get image and steering records for your Ai. Be careful not to use them to train.
# # AUTO_CREATE_NEW_TUB = False     #create a new tub (tub_YY_MM_DD) directory when recording or append records to data directory directly
# # 
# # 
# # #
# # # LED
# # #
# # HAVE_RGB_LED = False            #do you have an RGB LED like https://www.amazon.com/dp/B07BNRZWNF
# # LED_INVERT = False              #COMMON ANODE? Some RGB LED use common anode. like https://www.amazon.com/Xia-Fly-Tri-Color-Emitting-Diffused/dp/B07MYJQP8B
# # 
# # #LED board pin number for pwm outputs
# # #These are physical pinouts. See: https://www.raspberrypi-spy.co.uk/2012/06/simple-guide-to-the-rpi-gpio-header-and-pins/
# # LED_PIN_R = 12
# # LED_PIN_G = 10
# # LED_PIN_B = 16
# # 
# # #LED status color, 0-100
# # LED_R = 0
# # LED_G = 0
# # LED_B = 1
# # 
# # #LED Color for record count indicator
# # REC_COUNT_ALERT = 1000          #how many records before blinking alert
# # REC_COUNT_ALERT_CYC = 15        #how many cycles of 1/20 of a second to blink per REC_COUNT_ALERT records
# # REC_COUNT_ALERT_BLINK_RATE = 0.4 #how fast to blink the led in seconds on/off
# # 
# # #first number is record count, second tuple is color ( r, g, b) (0-100)
# # #when record count exceeds that number, the color will be used
# # RECORD_ALERT_COLOR_ARR = [ (0, (1, 1, 1)),
# #             (3000, (5, 5, 5)),
# #             (5000, (5, 2, 0)),
# #             (10000, (0, 5, 0)),
# #             (15000, (0, 5, 5)),
# #             (20000, (0, 0, 5)), ]
# # 
# # #LED status color, 0-100, for model reloaded alert
# # MODEL_RELOADED_LED_R = 100
# # MODEL_RELOADED_LED_G = 0
# # MODEL_RELOADED_LED_B = 0
# # 
# # 
# # #
# # # DonkeyGym
# # #
# # # Only on Ubuntu linux, you can use the simulator as a virtual donkey and
# # # issue the same python manage.py drive command as usual, but have them control a virtual car.
# # # This enables that, and sets the path to the simualator and the environment.
# # # You will want to download the simulator binary from: https://github.com/tawnkramer/donkey_gym/releases/download/v18.9/DonkeySimLinux.zip
# # # then extract that and modify DONKEY_SIM_PATH.
# # DONKEY_GYM = False
# # DONKEY_SIM_PATH = "path to sim" #"/home/tkramer/projects/sdsandbox/sdsim/build/DonkeySimLinux/donkey_sim.x86_64" when racing on virtual-race-league use "remote", or user "remote" when you want to start the sim manually first.
# # DONKEY_GYM_ENV_NAME = "donkey-generated-track-v0" # ("donkey-generated-track-v0"|"donkey-generated-roads-v0"|"donkey-warehouse-v0"|"donkey-avc-sparkfun-v0")
# # GYM_CONF = { "body_style" : "donkey", "body_rgb" : (128, 128, 128), "car_name" : "car", "font_size" : 100} # body style(donkey|bare|car01) body rgb 0-255
# # GYM_CONF["racer_name"] = "Your Name"
# # GYM_CONF["country"] = "Place"
# # GYM_CONF["bio"] = "I race robots."
# # 
# # SIM_HOST = "127.0.0.1"              # when racing on virtual-race-league use host "trainmydonkey.com"
# # SIM_ARTIFICIAL_LATENCY = 0          # this is the millisecond latency in controls. Can use useful in emulating the delay when useing a remote server. values of 100 to 400 probably reasonable.
# # 
# # # Save info from Simulator (pln)
# # SIM_RECORD_LOCATION = False
# # SIM_RECORD_GYROACCEL= False
# # SIM_RECORD_VELOCITY = False
# # SIM_RECORD_LIDAR = False
# # 
# # # publish camera over network on TCP socket
# # # This is used to create a tcp service to publish the camera feed
# # PUB_CAMERA_IMAGES = False
# # 
# # 
# # #
# # # AI Overrides
# # #
# # # Launch mode: override AI at launch time (transition from user to Auto pilot).
# # AI_LAUNCH_DURATION = 0.0            # the ai will output throttle for this many seconds
# # AI_LAUNCH_THROTTLE = 0.0            # the ai will output this throttle value
# # AI_LAUNCH_ENABLE_BUTTON = 'R2'      # this keypress will enable this boost. It must be enabled before each use to prevent accidental trigger.
# # AI_LAUNCH_KEEP_ENABLED = False      # when False ( default) you will need to hit the AI_LAUNCH_ENABLE_BUTTON for each use. This is safest. When this True, is active on each trip into "local" ai mode.
# # 
# # # throttle scaling: scale the output of the throttle of the ai pilot for all model types.
# # AI_THROTTLE_MULT = 1.0              # this multiplier will scale every throttle value for all output from NN models
# # 
# # 
# # #
# # # Intel Realsense D435 and D435i depth sensing camera
# # #
# # REALSENSE_D435_RGB = True       # True to capture RGB image
# # REALSENSE_D435_DEPTH = False    # True to capture depth as image array
# # REALSENSE_D435_IMU = False      # True to capture IMU data (D435i only)
# # REALSENSE_D435_ID = None        # serial number of camera or None if you only have one camera (it will autodetect)
# # 
# # 
# # #
# # # Stop Sign Detector
# # #
# # STOP_SIGN_DETECTOR = False
# # STOP_SIGN_MIN_SCORE = 0.2
# # STOP_SIGN_SHOW_BOUNDING_BOX = True
# # STOP_SIGN_MAX_REVERSE_COUNT = 10    # How many times should the car reverse when detected a stop sign, set to 0 to disable reversing
# # STOP_SIGN_REVERSE_THROTTLE = -0.5     # Throttle during reversing when detected a stop sign
# # 
# # #
# # # Frames/Second counter
# # #
# # SHOW_FPS = False
# # FPS_DEBUG_INTERVAL = 10    # the interval in seconds for printing the frequency info into the shell
# # 
# # #
# # # computer vision template
# # #
# # configure which part is used as the autopilot - change to use your own autopilot
# # Custom lane center follower
# CV_CONTROLLER_MODULE = "lane_center_follower"
# CV_CONTROLLER_CLASS = "LaneCenterFollower"
# CV_CONTROLLER_INPUTS = ["cam/image_array"]
# CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]
# CV_CONTROLLER_CONDITION = "run_pilot"

# OVERLAY_IMAGE = True

# # Lane-boundary color detection in OpenCV HSV:
# # hue 0..179, saturation 0..255, value 0..255.
# #
# # Current default: white boundary tape/lines.
# # LANE_HSV_LOW = (0, 0, 170)
# # LANE_HSV_HIGH = (180, 70, 255)

# # Starting point for blue painter's tape:
# LANE_HSV_LOW = (90, 50, 50)
# LANE_HSV_HIGH = (130, 255, 255)

# # RGB color used in the Donkey Monitor overlay for detected boundary pixels.
# LANE_MASK_COLOR = (0, 255, 0)
# # 
# # # LineFollower - line color and detection area
# # SCAN_Y = 100          # num pixels from the top to start horiz scan
# # SCAN_HEIGHT = 20      # num pixels high to grab from horiz scan
# # COLOR_THRESHOLD_LOW  = (0, 50, 50)    # HSV dark yellow (opencv HSV hue value is 0..179, saturation and value are both 0..255)
# # COLOR_THRESHOLD_HIGH = (50, 255, 255) # HSV light yellow (opencv HSV hue value is 0..179, saturation and value are both 0..255)
# # 
# # # LineFollower - target (expected) line position and detection thresholds
# # TARGET_PIXEL = None   # In not None, then this is the expected horizontal position in pixels of the yellow line.
# #                       # If None, then detect the position yellow line at startup;
# #                       # so this assumes you have positioned the car prior to starting.
# #                       # Alternatively set this to IMAGE_W / 2 to follow middle line
# # TARGET_THRESHOLD = 10 # number of pixels from TARGET_PIXEL that vehicle must be pointing
# #                       # before a steering change will be made; this prevents algorithm
# #                       # from being too twitchy when it is on or near the line.
# # CONFIDENCE_THRESHOLD = 0.0015   # The fraction of total sampled pixels that must be yellow in the sample slice.
# #                                 # The sample slice will have SCAN_HEIGHT pixels and the total number
# #                                 # of sampled pixels is IMAGE_W x SCAN_HEIGHT, so if you want to make sure
# #                                 # that all the pixels in the sample slice are yellow, then the confidence
# #                                 # threshold should be SCAN_HEIGHT / (IMAGE_W x SCAN_HEIGHT) or (1 / IMAGE_W).
# #                                 # if you want half of the pixels in the slice to match hten (1 / IMAGE_W) / 2.
# #                                 # If you keep getting `No line detected` logs in the console then you
# #                                 # may want to lower the threshold.
# # 
# # # LineFollower - throttle step controller; increase throttle on straights, descrease on turns
# # THROTTLE_MAX = 0.3    # maximum throttle value the controller will produce
# # THROTTLE_MIN = 0.15   # minimum throttle value the controller will produce
# # THROTTLE_INITIAL = THROTTLE_MIN  # initial throttle value
# # THROTTLE_STEP = 0.05  # how much to change throttle when off the line
# # 
# # # These three PID constants are crucial to the way the car drives. If you are tuning them
# # # start by setting the others zero and focus on first Kp, then Kd, and then Ki.
# # PID_P = -0.01         # proportional mult for PID path follower
# # PID_I = 0.000         # integral mult for PID path follower
# # PID_D = -0.0001       # differential mult for PID path follower
# # 
# # PID_P_DELTA = 0.005   # amount the inc/dec function will change the P value
# # PID_D_DELTA = 0.00005 # amount the inc/dec function will change the D value
# # 
# # OVERLAY_IMAGE = True  # True to draw computer vision overlay on camera image in web ui
# #                       # NOTE: this does not affect what is saved to the data
# # 
# # 
# # #
# # # Assign path follow functions to buttons.
# # # You can use game pad buttons OR web ui buttons ('web/w1' to 'web/w5')
# # # Use None use the game controller default
# # # NOTE: the cross button is already reserved for the emergency stop
# # #
# # TOGGLE_RECORDING_BTN = "option" # button to toggle recording mode
# # INC_PID_D_BTN = None            # button to change PID 'D' constant by PID_D_DELTA
# # DEC_PID_D_BTN = None            # button to change PID 'D' constant by -PID_D_DELTA
# # INC_PID_P_BTN = "R2"            # button to change PID 'P' constant by PID_P_DELTA
# # DEC_PID_P_BTN = "L2"            # button to change PID 'P' constant by -PID_P_DELTA
# # 

# # ============================================================
# # Lane boundary CV experiment settings
# # Copied from the working path_follower car where needed
# # ============================================================

# # Sliding Window lane detection parameters
# # LANE_ROI_Y_START_FRAC = 0.50
# # LANE_SW_WINDOWS = 6
# # LANE_SW_MARGIN = 25
# # LANE_SW_MIN_WINDOW_PIXELS = 5
# # LANE_SW_MIN_LANE_PIXELS = 30
# # LANE_LOOKAHEAD_Y_FRAC = 0.70
# # LANE_MIN_WIDTH_PX = 25
# # LANE_MAX_WIDTH_PX = 260
# # LANE_MAX_CENTER_JUMP_PX = 55

# LANE_ROI_Y_START_FRAC = 0.45
# LANE_LOOKAHEAD_Y_FRAC = 0.65
# LANE_SW_MARGIN = 35
# LANE_SW_MIN_WINDOW_PIXELS = 3
# LANE_SW_MIN_LANE_PIXELS = 20
# LANE_MIN_BLOB_AREA = 0
# LANE_MAX_WIDTH_PX = 0
# LANE_MAX_CENTER_JUMP_PX = 90


# # Use the previous good fitted lane as a fallback search area.
# LANE_PREVIOUS_FIT_MARGIN = 35
# LANE_MAX_MISSED_FRAMES = 2
# LANE_DROPOUT_THROTTLE = 0.0

# # Filter tiny HSV blobs before the sliding-window tracker sees them.
# LANE_MIN_BLOB_AREA = 0
# LANE_MAX_BLOB_AREA = 0
# LANE_MIN_BLOB_WIDTH = 0
# LANE_MIN_BLOB_HEIGHT = 0

# # Edge-based fallback. This runs only if HSV sliding windows and previous-fit
# # recovery fail to produce a valid lane.
# LANE_USE_HOUGH_FALLBACK = True
# LANE_CANNY_LOW = 50
# LANE_CANNY_HIGH = 150
# LANE_HOUGH_THRESHOLD = 15
# LANE_HOUGH_MIN_LINE_LENGTH = 25
# LANE_HOUGH_MAX_LINE_GAP = 20
# LANE_HOUGH_MIN_ABS_DY = 12
# LANE_HOUGH_PREVIOUS_FIT_MARGIN = 50
# LANE_HOUGH_MIN_SEGMENTS_PER_SIDE = 1

# # Slow down when steering demand is high.
# LANE_TEST_THROTTLE = 0.45
# LANE_THROTTLE_MIN = 0.45
# LANE_THROTTLE_SLOWDOWN = 0.05


# # Start with MOCK camera so the app boots before we touch OAK-D.
# # Later we will change this to OAKD.
# CAMERA_TYPE = "OAKD"
# CAMERA_INDEX = 0

# # Correct the OAK-D Pro Wide lens distortion before CV lane detection.
# # Lower alpha/balance crops more edge distortion; higher keeps more FOV.
# OAK_UNDISTORT = True
# OAK_UNDISTORT_MODEL = "auto"
# OAK_UNDISTORT_ALPHA = 0.0

# # Use the real VESC drivetrain, not the default PCA9685 PWM drivetrain.
# DRIVE_TRAIN_TYPE = "VESC"

# VESC_MAX_SPEED_PERCENT = 0.30
# VESC_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
# VESC_HAS_SENSOR = True
# VESC_START_HEARTBEAT = True
# VESC_BAUDRATE = 115200
# VESC_TIMEOUT = 0.05
# VESC_STEERING_SCALE = 0.5
# VESC_STEERING_OFFSET = 0.45

# # Joystick settings from working path_follower setup.
# CONTROLLER_TYPE = "custom"
# JOYSTICK_DEADZONE = 0.1
# JOYSTICK_THROTTLE_DIR = -1.0
# JOYSTICK_DEVICE_FILE = "/dev/input/js0"

# # Keep simulator off.
# DONKEY_GYM = False

# # Tub behavior.
# AUTO_CREATE_NEW_TUB = True

# # Start safe.
# WEB_INIT_MODE = "user"

# ============================================================
# FINAL OVERRIDES: curve-aware dual-lane follower
# Paste this at the VERY BOTTOM of myconfig.py
# ============================================================

# Make sure this matches your actual file name:
# lane_center_follower.py
CV_CONTROLLER_MODULE = "lane_center_follower"
CV_CONTROLLER_CLASS = "LaneCenterFollower"
CV_CONTROLLER_INPUTS = ["cam/image_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]
CV_CONTROLLER_CONDITION = "run_pilot"
OVERLAY_IMAGE = True


# ============================================================
# Camera
# ============================================================

CAMERA_TYPE = "OAKD"
CAMERA_INDEX = 0

# Bigger image helps the car see both lanes and curves better.
IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# Keep your checkerboard/OAK undistortion settings.
OAK_UNDISTORT = True
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")

# If the image gets too zoomed in, increase this.
# If black edges/distortion are too much, lower it.
OAK_UNDISTORT_ALPHA = 0.1


# ============================================================
# Drivetrain / VESC
# ============================================================

DRIVE_TRAIN_TYPE = "VESC"

# 0.2 is safe but slow. Use 0.25 or 0.3 only once steering is stable.
VESC_MAX_SPEED_PERCENT = 0.25

VESC_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
VESC_HAS_SENSOR = True
VESC_START_HEARTBEAT = True
VESC_BAUDRATE = 115200
VESC_TIMEOUT = 0.05

VESC_STEERING_SCALE = 0.5
VESC_STEERING_OFFSET = 0.45


# ============================================================
# Lane color mode
# ============================================================
# Use:
#   "yellow" for yellow tape
#   "blue" for blue tape
#   "white" for white tape
#   "all" if you want all masks combined

LANE_COLOR_MODE = "blue"


# ============================================================
# Curve-aware lane detection region
# ============================================================

# Your old value was 0.52, which looks too close to the car.
# Lower value lets the car see farther ahead into curves.
LANE_ROI_Y_START = 0.40
LANE_ROI_Y_END = 1.00

# Far lookahead point for curve steering.
# Smaller = farther ahead into the curve.
LANE_LOOKAHEAD_Y_FRACTION = 0.52

# Near point for estimating path direction.
LANE_NEAR_Y_FRACTION = 0.88

# More bands = better curved center path.
LANE_NUM_BANDS = 14

# Ignore extreme fisheye edges but keep enough width to see both lanes.
LANE_SEARCH_X_START_FRAC = 0.08
LANE_SEARCH_X_END_FRAC = 0.92


# ============================================================
# Yellow lane detection
# ============================================================

# Yellow tape HSV range.
LANE_YELLOW_H_LOW = 15
LANE_YELLOW_H_HIGH = 38
LANE_YELLOW_S_MIN = 70
LANE_YELLOW_V_MIN = 70

# Keep these for compatibility / if using white/blue mode later.
LANE_WHITE_S_MAX = 75
LANE_WHITE_V_MIN = 150

LANE_BLUE_H_LOW = 80
LANE_BLUE_H_HIGH = 155
LANE_BLUE_S_MIN = 20
LANE_BLUE_V_MIN = 25

LANE_USE_CLAHE = False
LANE_CLAHE_CLIP_LIMIT = 2.0
LANE_CLAHE_TILE_GRID_SIZE = 8


# ============================================================
# Noise filtering
# ============================================================

# Do NOT use 500 here. That can delete real lane segments on curves.
LANE_MIN_COMPONENT_AREA = 45

LANE_KERNEL_SIZE = 5
LANE_MIN_BAND_PIXELS = 5
LANE_MIN_RUN_WIDTH = 2
LANE_MIN_COLUMN_COUNT = 2

# Allows curves without rejecting center points too aggressively.
LANE_MAX_JUMP_PX = 120


# ============================================================
# Lane width / one-line fallback
# ============================================================

# If None, it learns width when both boundaries are visible.
LANE_EXPECTED_WIDTH_PX = None

# Starter estimate before learned width exists.
LANE_EXPECTED_WIDTH_FRAC = 0.55

# Allow one-line fallback when only one lane boundary is visible on curves.
LANE_ALLOW_ONE_SIDE_FALLBACK = True

# If the curve only shows the left lane boundary,
# stay to the RIGHT of it by half a lane width.
LANE_KEEP_SIDE = "right_of_left"

# Wide lane allowed.
LANE_MIN_WIDTH_FRAC = 0.12
LANE_MAX_WIDTH_FRAC = 0.98


# ============================================================
# Steering / curve control
# ============================================================

# Lateral correction: target center vs image center.
LANE_STEERING_GAIN = 0.035

# Heading correction: far path point vs near path point.
# This is what helps it actually turn into curves.
# LANE_HEADING_GAIN = 0.060
LANE_HEADING_GAIN = 0.040

# Allow stronger steering for sharp curves.
LANE_MAX_STEERING = 0.75

# Drift correction.
# If car naturally drifts left, positive usually biases it right.
# If it gets worse, change to -0.10.
# LANE_STEERING_BIAS = 0.10
LANE_STEERING_BIAS = -0.05

LANE_INVERT_STEERING = False

# Higher = reacts faster but more jitter.
# Lower = smoother but slower to turn.
LANE_SMOOTHING_ALPHA = 0.45


# ============================================================
# Speed / curve slowdown
# ============================================================

# Straight speed.
LANE_THROTTLE_STRAIGHT = 0.45
LANE_TEST_THROTTLE = 0.45

# Curves.
LANE_THROTTLE_TURN = 0.30

# Sharp curves.
LANE_THROTTLE_SHARP_TURN = 0.20

LANE_THROTTLE_MIN = 0.20
LANE_THROTTLE_MAX = 0.45


# ============================================================
# Safety / startup
# ============================================================

WEB_INIT_MODE = "user"
DONKEY_GYM = False
AUTO_CREATE_NEW_TUB = True

HAVE_MQTT_TELEMETRY = False
HAVE_PERFMON = False
HAVE_RGB_LED = False
USE_SSD1306_128_32 = False
STOP_SIGN_DETECTOR = False
SHOW_FPS = False

# Steering correction: reduce right-side bias
LANE_STEERING_BIAS = -0.05

# Slightly less aggressive curve/right snapping
LANE_STEERING_GAIN = 0.030
LANE_HEADING_GAIN = 0.045
LANE_MAX_STEERING = 0.70

# Smoother target tracking
LANE_SMOOTHING_ALPHA = 0.35

LANE_INVERT_STEERING = False

# ============================================================
# SIMPLE BASELINE + CURVE IMPROVEMENT
# ============================================================

CV_CONTROLLER_MODULE = "lane_center_follower"
CV_CONTROLLER_CLASS = "LaneCenterFollower"

# Blue tape threshold. Change only these if lane color changes.
LANE_HSV_LOW = (90, 50, 50)
LANE_HSV_HIGH = (130, 255, 255)
LANE_MASK_COLOR = (0, 255, 0)

# Look at bottom half, but enough forward to see curves.
LANE_ROI_Y_START = 0.50
LANE_ROI_Y_END = 1.00

# Smaller = look farther ahead into the curve.
LANE_LOOKAHEAD_Y_FRACTION = 0.55

# More bands = better curve shape, but more jitter.
LANE_NUM_BANDS = 8

# Steering.
LANE_STEERING_GAIN = 0.05
LANE_MAX_STEERING = 0.40

# Since it was veering right, start slightly left-biased.
LANE_STEERING_BIAS = -0.03

LANE_SMOOTHING_ALPHA = 0.25

# Speed.
LANE_TEST_THROTTLE = 0.30

# One-line fallback if one boundary disappears briefly.
LANE_ALLOW_ONE_SIDE_FALLBACK = True
LANE_EXPECTED_WIDTH_PX = None

# ============================================================
# One-line fallback behavior for curves
# ============================================================

LANE_ALLOW_ONE_SIDE_FALLBACK = True

# For your case:
# If the car enters a right curve and only sees the left lane boundary,
# it should stay to the RIGHT of that visible line.
LANE_KEEP_SIDE = "right_of_left"

# If automatic learned width is wrong, set this manually.
# Start with None so it learns from both lanes when both are visible.
LANE_EXPECTED_WIDTH_PX = None

# ============================================================
# One-line curve fallback
# ============================================================

LANE_ALLOW_ONE_SIDE_FALLBACK = True

# In your screenshot, the visible curve line should be treated as the LEFT lane.
# So the car should aim to the RIGHT of it.
LANE_KEEP_SIDE = "right_of_left"

# Reject fake two-lane detections when the same curve line appears in both halves.
LANE_MIN_WIDTH_FRAC = 0.25
LANE_MAX_WIDTH_FRAC = 0.95

# If the center is too far right, lower this.
# If the center is still too close/on top of the line, raise this.
LANE_EXPECTED_WIDTH_PX = 300

# ============================================================
# Bird's-eye curve follower settings
# ============================================================

CV_CONTROLLER_MODULE = "lane_center_follower"
CV_CONTROLLER_CLASS = "LaneCenterFollower"

# Turn bird's-eye mode on.
LANE_USE_BIRDSEYE = True

# Blue tape threshold.
LANE_HSV_LOW = (90, 50, 50)
LANE_HSV_HIGH = (130, 255, 255)
LANE_MASK_COLOR = (0, 255, 0)

# Bird's-eye source trapezoid.
# These are fractions of image width/height.
# Tune these if the warped view looks wrong.
LANE_BIRD_SRC_BOTTOM_LEFT = (0.05, 0.98)
LANE_BIRD_SRC_BOTTOM_RIGHT = (0.95, 0.98)
LANE_BIRD_SRC_TOP_RIGHT = (0.68, 0.52)
LANE_BIRD_SRC_TOP_LEFT = (0.32, 0.52)

# Bird's-eye destination rectangle.
LANE_BIRD_DST_BOTTOM_LEFT = (0.20, 1.00)
LANE_BIRD_DST_BOTTOM_RIGHT = (0.80, 1.00)
LANE_BIRD_DST_TOP_RIGHT = (0.80, 0.00)
LANE_BIRD_DST_TOP_LEFT = (0.20, 0.00)

# In bird's-eye mode, use more of the image.
LANE_ROI_Y_START = 0.15
LANE_ROI_Y_END = 1.00

# Smaller = look farther ahead into the curve.
LANE_LOOKAHEAD_Y_FRACTION = 0.38

# More bands = better curve shape.
LANE_NUM_BANDS = 10
LANE_MIN_PIXELS_PER_SIDE = 12

# One-line fallback when only one boundary is visible.
LANE_ALLOW_ONE_SIDE_FALLBACK = True
LANE_KEEP_SIDE = "right_of_left"

# If magenta center is too close to the visible line, increase this.
# If magenta center is too far away, decrease this.
LANE_EXPECTED_WIDTH_PX = 280

# Reject fake two-lane detections.
LANE_MIN_WIDTH_FRAC = 0.25
LANE_MAX_WIDTH_FRAC = 0.95

# Steering.
LANE_STEERING_GAIN = 0.065
LANE_MAX_STEERING = 0.75
LANE_STEERING_BIAS = -0.03
LANE_SMOOTHING_ALPHA = 0.30

# Speed.
LANE_TEST_THROTTLE = 0.35
VESC_MAX_SPEED_PERCENT = 0.25

# ============================================================
# STABLE BIRD'S-EYE TEST SETTINGS
# ============================================================

LANE_USE_BIRDSEYE = True

# Do not warp too much background/table/walls.
# Move top points LOWER in the image.
LANE_BIRD_SRC_BOTTOM_LEFT = (0.08, 0.98)
LANE_BIRD_SRC_BOTTOM_RIGHT = (0.92, 0.98)
LANE_BIRD_SRC_TOP_LEFT = (0.38, 0.62)
LANE_BIRD_SRC_TOP_RIGHT = (0.62, 0.62)

LANE_BIRD_DST_BOTTOM_LEFT = (0.20, 1.00)
LANE_BIRD_DST_BOTTOM_RIGHT = (0.80, 1.00)
LANE_BIRD_DST_TOP_LEFT = (0.20, 0.00)
LANE_BIRD_DST_TOP_RIGHT = (0.80, 0.00)

# Ignore the noisy far/top part of the warped image.
LANE_ROI_Y_START = 0.35
LANE_ROI_Y_END = 1.00

# Look a little closer until the center path is stable.
# Smaller = farther ahead. Larger = closer.
LANE_LOOKAHEAD_Y_FRACTION = 0.62

# Fewer bands = less zig-zag while tuning.
LANE_NUM_BANDS = 8
LANE_MIN_PIXELS_PER_SIDE = 18

# Lane width limits.
LANE_MIN_WIDTH_FRAC = 0.20
LANE_MAX_WIDTH_FRAC = 0.75

# Make steering less violent while debugging.
LANE_STEERING_GAIN = 0.035
LANE_MAX_STEERING = 0.45
LANE_SMOOTHING_ALPHA = 0.18

LANE_STEERING_BIAS = -0.03

# Slow test speed.
LANE_TEST_THROTTLE = 0.25
VESC_MAX_SPEED_PERCENT = 0.35


# ============================================================
# ACTIVE SETTINGS — only this block matters, everything above is overridden
# ============================================================

LANE_STEERING_GAIN       = 0.01   # MUST stay low — 0.06 causes the snake
LANE_MAX_STEERING        = 0.75
LANE_SMOOTHING_ALPHA     = 0.4
LANE_STEERING_BIAS       = -0.05
LANE_STEERING_SMOOTHING  = 0.40
LANE_DEADBAND_PX         = 20

LANE_LOOKAHEAD_Y_FRACTION = 0.55   # farther ahead = more stable on straights
LANE_NUM_BANDS           = 10      # more bands = better curve tracking
LANE_ROI_Y_START         = 0.35
LANE_MIN_PIXELS_PER_SIDE = 12

VESC_STEERING_SCALE      = 0.55
LANE_TEST_THROTTLE       = 0.3
VESC_MAX_SPEED_PERCENT   = 0.35

LANE_MAX_CENTER_JUMP_PX  = 60   # reject band if center jumps more than 60px
LANE_MIN_PIXELS_PER_SIDE = 20   # require more pixels before trusting a band
LANE_NUM_BANDS           = 7    # fewer bands = fewer bad detections in the mix


# # ==============================
# # OBSTACLE DETECTION / OBSTACLE AVOIDANCE
# # ==============================
# OBSTACLE_SCAN_Y        = 220    # pixels from top where ROI starts
# OBSTACLE_SCAN_HEIGHT   = 80    # ROI height in pixels
# OBSTACLE_DEPTH_THRESH  = 500   # hard stop distance (mm)
# OBSTACLE_WARN_THRESH   = 900  # start slowing distance (mm)
# OBSTACLE_MIN_AREA = 3000
# OBSTACLE_STOP_THROTTLE = 0.0
# OBSTACLE_SLOW_FACTOR   = 0.5
# OBSTACLE_STEER_BIAS    = 0.35
# OBSTACLE_OVERLAY       = True
# OBSTACLE_CONFIRM_FRAMES = 3   # frames before declaring obstacle
# OBSTACLE_CLEAR_FRAMES   = 5   # frames before clearing obstacle

# ============================================================
# Curve centerline tracking fix
# ============================================================

# Use the same bird's-eye setup.
LANE_USE_BIRDSEYE = True

# Do not force one-line curves to always be left/right.
# The new code uses boundary tracking instead.
LANE_KEEP_SIDE = "temporal"

# Important: use a fixed fallback lane width while testing.
# Increase if the magenta center is too close to one visible lane.
# Decrease if it is too far away.
LANE_EXPECTED_WIDTH_PX = 300

# Allow normal lane spacing but reject bad fake pairs.
LANE_MIN_WIDTH_FRAC = 0.20
LANE_MAX_WIDTH_FRAC = 0.80

# Allow real curve motion across bands.
LANE_MAX_CENTER_JUMP_PX = 180
LANE_TRACK_MAX_GAP_PX = 180

# Use fewer, more stable bands.
LANE_NUM_BANDS = 8
LANE_MIN_PIXELS_PER_SIDE = 14

# Look moderately ahead, not all the way into noisy far-field.
LANE_LOOKAHEAD_Y_FRACTION = 0.55

# Smooth the target more.
LANE_SMOOTHING_ALPHA = 0.22

# Keep steering calm until the magenta path is correct.
LANE_STEERING_GAIN = 0.006
LANE_MAX_STEERING = 0.80
LANE_STEERING_SMOOTHING = 0.25
LANE_DEADBAND_PX = 18

# Keep speed low while debugging.
LANE_TEST_THROTTLE = 0.25
VESC_MAX_SPEED_PERCENT = 0.35
# ============================================================
# Faster speed + quicker steering response
# Added from terminal
# ============================================================

# More drivetrain power.
# If too fast, reduce to 0.40.
VESC_MAX_SPEED_PERCENT = 0.50

# Main autonomous throttle used by lane_center_follower.py.
# If it is too fast, reduce to 0.40.
LANE_TEST_THROTTLE = 0.45

# Look farther ahead into turns.
# Smaller = farther ahead, reacts earlier to curves.
LANE_LOOKAHEAD_Y_FRACTION = 0.50

# See slightly farther up the track.
# Lower = more forward view, but potentially more noise.
LANE_ROI_Y_START = 0.45

# Stronger steering response.
LANE_STEERING_GAIN = 0.010
LANE_MAX_STEERING = 0.85

# Faster centerline response.
# Higher = quicker response, lower = smoother.
LANE_SMOOTHING_ALPHA = 0.50

# Faster steering actuator response.
# Higher = quicker steering, lower = smoother.
LANE_STEERING_SMOOTHING = 0.40

# Smaller deadband = reacts sooner to small errors.
LANE_DEADBAND_PX = 10

# Let physical wheels turn a bit more.
# If it becomes twitchy, reduce to 0.55.
VESC_STEERING_SCALE = 0.55

# ============================================================
# Speed override: reduce autonomous speed to 0.40
# Added from terminal
# ============================================================

LANE_TEST_THROTTLE = 0.35

# ============================================================
# Straight recovery after curves
# Added from terminal
# ============================================================

# Detect when the center path is basically straight again.
LANE_STRAIGHT_PATH_DELTA_PX = 45
LANE_STRAIGHT_ERROR_PX = 35

# Recover quickly after a curve.
LANE_CENTER_RECOVERY_ALPHA = 0.85
LANE_STEERING_RECOVERY_ALPHA = 0.75

# Lower steering gain on straights, stronger gain on curves.
LANE_STRAIGHT_STEERING_GAIN = 0.0045
LANE_CURVE_STEERING_GAIN = 0.010

# Keep some smoothing for curves, but not too much lag.
LANE_SMOOTHING_ALPHA = 0.35
LANE_STEERING_SMOOTHING = 0.25

# Deadband prevents straight-line wiggle.
LANE_DEADBAND_PX = 14

# Keep current speed.
LANE_TEST_THROTTLE = 0.40

# ============================================================
# Speed override: set actual VESC speed cap to 0.35
# Added from terminal
# ============================================================

VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# ============================================================
# Fix left-turn one-line fallback + reduce straight swerving
# Added from terminal
# ============================================================

# When only one green line is visible, force it to be treated as
# the LEFT boundary, so the virtual centerline is placed to the RIGHT.
LANE_KEEP_SIDE = "right_of_left"

# Make the inferred center farther to the right of the single visible line.
# Increase if magenta is still too close to the green line.
# Decrease if magenta goes too far right.
LANE_EXPECTED_WIDTH_PX = 420

# Keep speed fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# Reduce straight-path swerving.
LANE_STRAIGHT_STEERING_GAIN = 0.0035
LANE_DEADBAND_PX = 22
LANE_STEERING_SMOOTHING = 0.18
LANE_SMOOTHING_ALPHA = 0.28

# Keep curve steering strong enough.
LANE_CURVE_STEERING_GAIN = 0.010
LANE_MAX_STEERING = 0.85
VESC_STEERING_SCALE = 0.55

# Recover from curve steering quickly when path becomes straight.
LANE_CENTER_RECOVERY_ALPHA = 0.85
LANE_STEERING_RECOVERY_ALPHA = 0.80

# ============================================================
# Obstacle detection stop behavior
# Added from terminal
# ============================================================

OBSTACLE_ENABLE = True

# Look for obstacles inside the lane center corridor.
# In bird's-eye mode this is the warped control image.
OBSTACLE_ROI_Y_START = 0.20
OBSTACLE_ROI_Y_END = 1.00

# Width around the lane center where obstacles matter.
# Increase if it misses obstacles near the lane edges.
# Decrease if it reacts to objects outside the path.
OBSTACLE_CORRIDOR_HALF_WIDTH_PX = 180

# Red/orange obstacle HSV range.
# Good test objects: orange cone, red cup, red/orange block.
OBSTACLE_HSV_LOW1 = (0, 80, 50)
OBSTACLE_HSV_HIGH1 = (20, 255, 255)

OBSTACLE_HSV_LOW2 = (160, 80, 50)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Ignore tiny noise.
OBSTACLE_MIN_AREA = 350

# Complete stop command.
OBSTACLE_STOP_THROTTLE = 0.0
OBSTACLE_STOP_STEERING = 0.0

# ============================================================
# Earlier + sharper curve turning, speed still 0.35
# ============================================================

# Keep actual speed at 0.35
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# Turn earlier: smaller = looks farther ahead into the curve
LANE_LOOKAHEAD_Y_FRACTION = 0.42

# See slightly farther ahead in the bird's-eye ROI
LANE_ROI_Y_START = 0.42

# Keep straights calm
LANE_STRAIGHT_STEERING_GAIN = 0.0035

# Make curve turns stronger
LANE_CURVE_STEERING_GAIN = 0.014
LANE_MAX_STEERING = 0.90

# Let the physical wheels turn more
VESC_STEERING_SCALE = 0.65

# Respond quicker, but not too jittery
LANE_SMOOTHING_ALPHA = 0.45
LANE_STEERING_SMOOTHING = 0.35
LANE_DEADBAND_PX = 16

# ============================================================
# Fix left-turn center too far right + reduce post-curve swerving
# ============================================================

# Keep actual speed fixed at 0.35
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# When only one line is visible, still treat it as the left boundary,
# but do NOT place the virtual center so far to the right.
# If magenta is still too far right, lower to 300.
# If magenta becomes too close to green line, raise to 360.
LANE_KEEP_SIDE = "right_of_left"
LANE_EXPECTED_WIDTH_PX = 320

# Look ahead enough for curves, but not so far that it grabs noisy far-field points.
LANE_LOOKAHEAD_Y_FRACTION = 0.48
LANE_ROI_Y_START = 0.45

# Reduce straight-line swerving after a curve.
LANE_STRAIGHT_STEERING_GAIN = 0.0028
LANE_DEADBAND_PX = 26
LANE_STEERING_SMOOTHING = 0.16
LANE_SMOOTHING_ALPHA = 0.24

# Recover quickly when the path becomes straight again.
LANE_CENTER_RECOVERY_ALPHA = 0.95
LANE_STEERING_RECOVERY_ALPHA = 0.90

# Keep curve steering strong enough, but not insane.
LANE_CURVE_STEERING_GAIN = 0.010
LANE_MAX_STEERING = 0.80
VESC_STEERING_SCALE = 0.55

# Move one-line fallback center more left
LANE_EXPECTED_WIDTH_PX = 280

# ============================================================
# LEFT TURN FIX TEST
# Added from terminal
# ============================================================

# Keep speed fixed.
VESC_MAX_SPEED_PERCENT = 0.3
LANE_TEST_THROTTLE = 0.3

# IMPORTANT:
# For left turns, if the visible green line is the RIGHT lane boundary,
# the virtual center should be placed to the LEFT of that line.
LANE_KEEP_SIDE = "left_of_right"

# Controls how far away from the single visible line the virtual center is.
# Lower = center closer to visible line.
# Higher = center farther away.
LANE_EXPECTED_WIDTH_PX = 280

# Turn earlier by looking farther ahead.
# Smaller = farther ahead into the curve.
LANE_LOOKAHEAD_Y_FRACTION = 0.40
LANE_ROI_Y_START = 0.40

# Make curve steering stronger.
LANE_CURVE_STEERING_GAIN = 0.014
LANE_MAX_STEERING = 0.70
VESC_STEERING_SCALE = 0.55

# Keep straight sections calmer after the curve.
LANE_STRAIGHT_STEERING_GAIN = 0.0025
LANE_DEADBAND_PX = 28
LANE_STEERING_SMOOTHING = 0.14
LANE_SMOOTHING_ALPHA = 0.24

# Recover quickly after leaving a curve.
LANE_CENTER_RECOVERY_ALPHA = 0.95
LANE_STEERING_RECOVERY_ALPHA = 0.90

# ============================================================
# Obstacle stop calibration
# ============================================================

OBSTACLE_ENABLE = True

# Search inside the lane-center corridor.
OBSTACLE_ROI_Y_START = 0.15
OBSTACLE_ROI_Y_END = 1.00
OBSTACLE_CORRIDOR_HALF_WIDTH_PX = 220

# Red/orange obstacle detection.
OBSTACLE_HSV_LOW1 = (0, 70, 40)
OBSTACLE_HSV_HIGH1 = (25, 255, 255)
OBSTACLE_HSV_LOW2 = (155, 70, 40)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Ignore small noise.
OBSTACLE_MIN_AREA = 500

# Stop threshold.
# Larger number = stop later/closer.
# Smaller number = stop earlier/farther away.
OBSTACLE_STOP_Y_FRAC = 0.72

# Require 2 consecutive frames before stopping.
OBSTACLE_CONFIRM_FRAMES = 2

# Complete stop command.
OBSTACLE_STOP_THROTTLE = 0.0
OBSTACLE_STOP_STEERING = 0.0

# Keep driving speed fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# ============================================================
# Lane-gated obstacle detection
# ============================================================

OBSTACLE_ENABLE = True

# This is only the broad search area.
# The new code still filters by the actual green lane boundaries.
OBSTACLE_CORRIDOR_HALF_WIDTH_PX = 260

# Only accept an obstacle if its bottom-center is this far inside the two lane boundaries.
# Increase if it still reacts to obstacles near/outside the lane.
# Decrease if it misses objects close to lane edges.
OBSTACLE_LANE_MARGIN_PX = 15

# Hide ignored outside-lane detections.
OBSTACLE_DRAW_IGNORED = False

# Stop line calibration for about 1 foot.
# Lower = stop earlier. Higher = stop later.
OBSTACLE_STOP_Y_FRAC = 0.72
OBSTACLE_CONFIRM_FRAMES = 2

# Keep speed fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# ============================================================
# Additional obstacle colors
# ============================================================

# Still detect red/orange obstacles.
OBSTACLE_HSV_LOW1 = (0, 70, 40)
OBSTACLE_HSV_HIGH1 = (25, 255, 255)
OBSTACLE_HSV_LOW2 = (155, 70, 40)
OBSTACLE_HSV_HIGH2 = (179, 255, 255)

# Also detect dark/black obstacles.
# Increase to 110 if it misses dark objects.
# Decrease to 60 if it detects too much floor/shadow.
OBSTACLE_DARK_V_MAX = 90

# Make detector a bit more sensitive.
OBSTACLE_MIN_AREA = 300

# Only stop for obstacles inside the lane boundaries.
OBSTACLE_LANE_MARGIN_PX = 15

# Keep speed fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# ============================================================
# Camera resolution / wider view override
# ============================================================

# Higher resolution for lane detection.
IMAGE_W = 800
IMAGE_H = 448
IMAGE_DEPTH = 3
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# Keep OAK-D camera active.
CAMERA_TYPE = "OAKD"
CAMERA_INDEX = 0

# Keep undistortion enabled.
OAK_UNDISTORT = True

# Wider view after undistortion.
# Higher = keeps more field of view, may show more curved edges/black borders.
# Lower = crops more, less distortion, but narrower view.
OAK_UNDISTORT_ALPHA = 0.5

# ============================================================
# Left turn / curve-following fix
# ============================================================

# Keep speed cap fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# Slow slightly only during sharper curves.
LANE_THROTTLE_TURN = 0.28
LANE_TURN_SLOWDOWN_STEER_THRESHOLD = 0.35

# Look farther ahead into the curve.
# Smaller = earlier turn response.
LANE_LOOKAHEAD_Y_FRACTION = 0.38
LANE_ROI_Y_START = 0.42

# Use heading/curve information, not only center error.
# Increase if it still turns too late.
# Decrease if it starts over-turning.
LANE_HEADING_GAIN = 0.0045

# Stronger curve steering, calmer straight steering.
LANE_CURVE_STEERING_GAIN = 0.013
LANE_STRAIGHT_STEERING_GAIN = 0.0025

# Keep the target safely inside the two lanes.
# Increase if it gets too close to a boundary.
# Decrease if it becomes too conservative.
LANE_BOUNDARY_MARGIN_PX = 50

# For left curves: negative moves target left.
# If it still cuts too far right, try -35.
# If it moves too far left, try -10.
LANE_LEFT_CURVE_TARGET_BIAS_PX = -25

# Reduce swerving after the curve straightens.
LANE_DEADBAND_PX = 28
LANE_SMOOTHING_ALPHA = 0.26
LANE_STEERING_SMOOTHING = 0.16
LANE_CENTER_RECOVERY_ALPHA = 0.95
LANE_STEERING_RECOVERY_ALPHA = 0.90

# Allow enough steering authority but do not max out too hard.
LANE_MAX_STEERING = 0.85
VESC_STEERING_SCALE = 0.60

# ============================================================
# Soften left-turn overcorrection
# ============================================================

# Keep speed fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# Reduce how much the target is pushed left during left curves.
# Was probably -25 or -35. Closer to 0 = less left bias.
LANE_LEFT_CURVE_TARGET_BIAS_PX = -8

# Reduce curve-heading steering so it does not dive into the turn too hard.
LANE_HEADING_GAIN = 0.0032

# Slightly reduce curve steering strength.
LANE_CURVE_STEERING_GAIN = 0.0105

# Slightly reduce physical steering authority.
VESC_STEERING_SCALE = 0.55
LANE_MAX_STEERING = 0.78

# Add a bit more smoothing so it does not snap left.
LANE_SMOOTHING_ALPHA = 0.24
LANE_STEERING_SMOOTHING = 0.14

# Keep straight recovery stable after the curve.
LANE_DEADBAND_PX = 28
LANE_CENTER_RECOVERY_ALPHA = 0.95
LANE_STEERING_RECOVERY_ALPHA = 0.90

# ============================================================
# Revert left-turn experiments / return to safer baseline
# ============================================================

# Keep speed fixed.
VESC_MAX_SPEED_PERCENT = 0.35
LANE_TEST_THROTTLE = 0.35

# Disable the extra left-curve bias behavior if code still has it.
LANE_LEFT_CURVE_TARGET_BIAS_PX = 0
LANE_HEADING_GAIN = 0.0

# Return to simpler steering behavior.
LANE_CURVE_STEERING_GAIN = 0.010
LANE_STRAIGHT_STEERING_GAIN = 0.0035
LANE_MAX_STEERING = 0.75
VESC_STEERING_SCALE = 0.55

# Return to moderate lookahead/smoothing.
LANE_LOOKAHEAD_Y_FRACTION = 0.48
LANE_ROI_Y_START = 0.45
LANE_SMOOTHING_ALPHA = 0.28
LANE_STEERING_SMOOTHING = 0.18
LANE_DEADBAND_PX = 24

# Keep straight recovery.
LANE_CENTER_RECOVERY_ALPHA = 0.90
LANE_STEERING_RECOVERY_ALPHA = 0.85

# ============================================================
# Wider camera perspective / less crop
# ============================================================

# Keep the same resolution.
IMAGE_W = 640
IMAGE_H = 400

# Preserve more of the camera field of view after undistortion.
# 0.1 = cleaner but more cropped
# 0.5 = wider
# 0.7 = much wider
# 1.0 = widest, but may show black borders / edge distortion
OAK_UNDISTORT = True
OAK_UNDISTORT_ALPHA = 0.7

# Slightly less wide / cleaner view
OAK_UNDISTORT_ALPHA = 0.5

# ============================================================
# Camera perspective balance: wider but less bent
# ============================================================

IMAGE_W = 640
IMAGE_H = 400

OAK_UNDISTORT = True

# 0.0 = least bent but most cropped/narrow
# 1.0 = widest but most black borders/bending
# 0.35-0.5 is usually the best compromise
OAK_UNDISTORT_ALPHA = 0.4

# ============================================================
# Revert camera perspective back to previous working view
# ============================================================

IMAGE_W = 640
IMAGE_H = 400

OAK_UNDISTORT = True
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")

# Previous working value: less bent / less wide
OAK_UNDISTORT_ALPHA = 0.1

# Revert bird's-eye perspective back to lane_center_follower defaults
LANE_BIRD_SRC_BOTTOM_LEFT = (0.05, 0.98)
LANE_BIRD_SRC_BOTTOM_RIGHT = (0.95, 0.98)
LANE_BIRD_SRC_TOP_LEFT = (0.32, 0.52)
LANE_BIRD_SRC_TOP_RIGHT = (0.68, 0.52)

LANE_BIRD_DST_BOTTOM_LEFT = (0.15, 1.00)
LANE_BIRD_DST_BOTTOM_RIGHT = (0.85, 1.00)
LANE_BIRD_DST_TOP_LEFT = (0.15, 0.00)
LANE_BIRD_DST_TOP_RIGHT = (0.85, 0.00)

# ============================================================
# Maximum camera peripheral view test
# ============================================================

IMAGE_W = 640
IMAGE_H = 400

OAK_UNDISTORT = True
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")

# Widest undistorted view.
# Expect more black borders / bending near edges.
OAK_UNDISTORT_ALPHA = 1.0

# ============================================================
# Revert camera FOV back to previous working view
# ============================================================

IMAGE_W = 640
IMAGE_H = 400

OAK_UNDISTORT = True
OAK_CALIBRATION_FILE = os.path.join(CAR_PATH, "camera_calibration.npz")

# Previous working value
OAK_UNDISTORT_ALPHA = 0.1

# ============================================================
# Revert depth/height/heatmap experiment
# ============================================================

# Return to RGB-only camera input for the original lane follower.
CV_CONTROLLER_INPUTS = ["cam/image_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]

# Disable depth visualization / height-distance experiment.
DEPTH_VIEW_MODE = "off"
DEPTH_OBSTACLE_ENABLE = False

# Keep original camera type.
CAMERA_TYPE = "OAKD"
IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3

# Previous working undistortion setting.
OAK_UNDISTORT = True
OAK_UNDISTORT_ALPHA = 0.1

# ============================================================
# Force revert: original RGB lane follower setup
# ============================================================

# RGB-only camera input.
CV_CONTROLLER_INPUTS = ["cam/image_array"]
CV_CONTROLLER_OUTPUTS = ["pilot/steering", "pilot/throttle", "cv/image_array"]

# Disable depth / heatmap / height-distance experiments.
DEPTH_VIEW_MODE = "off"
DEPTH_OBSTACLE_ENABLE = False

# Disable YOLO / external obstacle-avoidance experiments if manage.py checks these.
YOLO_OBSTACLE_ENABLE = False
OBSTACLE_AVOIDANCE_ENABLE = False
USE_OBSTACLE_AVOIDANCE = False
USE_YOLO_OBSTACLE_AVOIDANCE = False
USE_LIDAR = False

# Keep original camera setup.
CAMERA_TYPE = "OAKD"
IMAGE_W = 640
IMAGE_H = 400
IMAGE_DEPTH = 3

OAK_UNDISTORT = True
OAK_UNDISTORT_ALPHA = 0.1

# Keep original custom lane follower.
CV_CONTROLLER_MODULE = "lane_center_follower"
CV_CONTROLLER_CLASS = "LaneCenterFollower"
CV_CONTROLLER_CONDITION = "run_pilot"
