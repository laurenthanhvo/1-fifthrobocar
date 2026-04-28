# DSC 190 Working Car Documentation

**Last Updated:** 04/23/2026  
**Project Report:** `<insert project report link>`

This document explains how to power on the car, SSH into the Jetson, access the file system from VS Code, run DonkeyCar, use the web UI, work with the remote controller, launch camera/LiDAR nodes, use Foxglove, run sensor fusion, use GPS, and debug common issues.

> **Security Note:** This documentation may contain private IP addresses, usernames, and hardware-specific paths. Do not commit passwords, private IPs, or private access details to a public GitHub repository. Replace private values with placeholders such as `<JETSON_IP>` and `<JETSON_PASSWORD>` before publishing.

---

# Table of Contents

1. [System Overview](#1-system-overview)
2. [Safety and Hardware Notes](#2-safety-and-hardware-notes)
3. [Powering On the Car](#3-powering-on-the-car)
4. [Finding the Jetson IP Address](#4-finding-the-jetson-ip-address)
5. [SSH Access](#5-ssh-access)
6. [Connecting Through a Hotspot](#6-connecting-through-a-hotspot)
7. [Accessing the Jetson File System from VS Code](#7-accessing-the-jetson-file-system-from-vs-code)
8. [USB Device Names](#8-usb-device-names)
9. [Editing Files on the Jetson](#9-editing-files-on-the-jetson)
10. [DonkeyCar Project Setup](#10-donkeycar-project-setup)
11. [Running the Car with DonkeyCar](#11-running-the-car-with-donkeycar)
12. [DonkeyCar Web UI](#12-donkeycar-web-ui)
13. [Joystick and Remote Control](#13-joystick-and-remote-control)
14. [DonkeyCar Configuration Notes](#14-donkeycar-configuration-notes)
15. [Path Following and PID](#15-path-following-and-pid)
16. [Running the Car with ROS Keyboard Teleop](#16-running-the-car-with-ros-keyboard-teleop)
17. [ROS 2 Basic Commands](#17-ros-2-basic-commands)
18. [Docker Basic Commands](#18-docker-basic-commands)
19. [Camera and LiDAR ROS 2 Nodes](#19-camera-and-lidar-ros-2-nodes)
20. [Sensor Fusion](#20-sensor-fusion)
21. [Foxglove Visualization](#21-foxglove-visualization)
22. [GPS and Septentrio](#22-gps-and-septentrio)
23. [Working Version Check](#23-working-version-check)
24. [Common Troubleshooting](#24-common-troubleshooting)
25. [Useful Paths](#25-useful-paths)
26. [Useful Files](#26-useful-files)
27. [Useful References](#27-useful-references)
28. [ROS 2 Lane Detection Progress](#28-ros-2-lane-detection-progress)
29. [One-Line Detection](#29-one-line-detection)
30. [Basic Two-Line Lane Detection](#30-basic-two-line-lane-detection)
31. [Fitted Two-Line Lane Detection](#31-fitted-two-line-lane-detection)
32. [Curved Lane Detection](#32-curved-lane-detection)
33. [Recording and Replaying Lane Data](#33-recording-and-replaying-lane-data)
34. [Recommended Lane Detection Terminal Layout](#34-recommended-lane-detection-terminal-layout)
35. [How to Interpret Lane Error](#35-how-to-interpret-lane-error)
36. [Current Lane Detection Status](#36-current-lane-detection-status)
37. [Next Step After VESC/Controller Are Fixed](#37-next-step-after-vesccontroller-are-fixed)
38. [Updated Future Work](#38-updated-future-work)

---

# 1. System Overview

The car stack currently uses:

- **Jetson** as the main onboard computer
- **DonkeyCar** for driving, path following, joystick/web control, and VESC control
- **VESC** for motor control
- **Radio Master controller** or **DonkeyCar web UI** for manual driving
- **OAK camera** for image data
- **Livox LiDAR** for point cloud data
- **ROS 2** for sensor topics and bridge workflows
- **Foxglove** for live visualization
- **Septentrio GPS** for GPS data
- **Docker** for camera, LiDAR, and sensor fusion workflows

Main DonkeyCar project folder:

```bash
/home/jetson/projects/mycars/path_follower
```

Sensor fusion project folder:

```bash
/home/jetson/sensorfusion/ros2_camera_lidar_fusion
```

ROS 2 workspace:

```bash
/home/jetson/dsc190_ws
```

---

# 2. Safety and Hardware Notes

## Power Distribution

- Do **not** plug a `5V` device into a `12V` port. This can burn the component.
- `20V` is for the VESC because the drive motor needs higher power.
- If the servo does not work, check the small connector on the right side.
- The servo wiring colors are flipped on this setup:
  - Usually, black is ground.
  - On this setup, **white is ground**.
  - Match **white to black**.
- Ask for a lid if the electronics are exposed.

## Safe Driving Notes

Before testing movement:

1. Make sure the car has enough open space.
2. Keep the wheels lifted when testing drivetrain changes.
3. Start with low throttle.
4. Be ready to emergency stop.
5. Do not run high throttle while the car is lifted off the ground.

Do not set max speed too high while the car is off the ground. Without ground resistance, the car can behave unpredictably or shut down.

---

# 3. Powering On the Car

1. Connect the main power cable on the car.
2. Face the car forward.
3. Press the power button on the top-left corner of the power distribution board.
4. Press either the first or third button on the Jetson computer, which is the black box.

---

# 4. Finding the Jetson IP Address

## Current Known Jetson IP

```bash
192.168.139.178
```

This IP may change depending on the network or hotspot.

## If You Are Physically on the Jetson

Run:

```bash
myip
```

Or run:

```bash
ip addr show wlan0
```

Look for the `inet` field. The IP address is the first number string after `inet`.

Example:

```text
inet 192.168.139.178/24
```

In this example, the Jetson IP is:

```bash
192.168.139.178
```

---

# 5. SSH Access

From your personal computer:

```bash
ssh jetson@<JETSON_IP>
```

Example:

```bash
ssh jetson@192.168.139.178
```

If using X forwarding:

```bash
ssh -X jetson@<JETSON_IP>
```

Example:

```bash
ssh -X jetson@192.168.139.178
```

If you do not need X forwarding, use:

```bash
ssh -x jetson@<JETSON_IP>
```

Example:

```bash
ssh -x jetson@192.168.139.178
```

---

# 6. Connecting Through a Hotspot

1. Connect the Jetson to the hotspot using the Ubuntu network menu.
2. On the Jetson, find the IP address:

```bash
ifconfig
```

or:

```bash
ip addr show wlan0
```

3. Connect your personal computer to the same hotspot.
4. SSH into the Jetson:

```bash
ssh -X jetson@<HOTSPOT_JETSON_IP>
```

Example:

```bash
ssh -X jetson@10.53.210.191
```

---

# 7. Accessing the Jetson File System from VS Code

This section explains how to open and browse the Jetson file system directly from a local computer using **VS Code Remote SSH**.

> **Important:** Do not commit passwords or private credentials to GitHub. Replace private values with placeholders if this README is public.

---

## 7.1 Install the VS Code Remote SSH Extension

On your local computer:

1. Open VS Code.
2. Go to the Extensions tab.
3. Search for:

```text
Remote - SSH
```

4. Install the extension published by Microsoft.

---

## 7.2 Test SSH from Your Local Terminal

Before using VS Code, confirm SSH works from your local computer.

```bash
ssh jetson@<JETSON_IP>
```

Example:

```bash
ssh jetson@192.168.139.178
```

If this works, VS Code Remote SSH should also work.

---

## 7.3 Add the Jetson as a VS Code SSH Host

In VS Code:

1. Press:

```text
Cmd + Shift + P
```

2. Search for:

```text
Remote-SSH: Add New SSH Host
```

3. Enter:

```bash
ssh jetson@<JETSON_IP>
```

Example:

```bash
ssh jetson@192.168.139.178
```

4. When VS Code asks which SSH config file to update, select your user config file:

```text
/Users/<your-local-username>/.ssh/config
```

Example:

```text
/Users/laurenvo/.ssh/config
```

Do **not** select:

```text
/etc/ssh/ssh_config
```

---

## 7.4 Confirm the SSH Config Entry

VS Code should add an entry similar to this:

```sshconfig
Host ucsd-agx-03
    HostName <JETSON_IP>
    User jetson
```

Example:

```sshconfig
Host ucsd-agx-03
    HostName 192.168.139.178
    User jetson
```

You can manually edit this file from your local computer:

```bash
nano ~/.ssh/config
```

---

## 7.5 Connect to the Jetson from VS Code

In VS Code:

1. Press:

```text
Cmd + Shift + P
```

2. Search for:

```text
Remote-SSH: Connect to Host
```

3. Select:

```text
ucsd-agx-03
```

or select the Jetson IP if that is what appears.

4. If VS Code asks for the platform, choose:

```text
Linux
```

5. Enter the Jetson password when prompted.

Once connected, VS Code is running remotely on the Jetson.

---

## 7.6 Open the Project Folder

After connecting, VS Code will ask you to open a folder.

To open the main project folder, enter:

```text
/home/jetson/projects/mycars
```

Then click:

```text
OK
```

This folder contains the DonkeyCar projects:

```text
/home/jetson/projects/mycars/path_follower
/home/jetson/projects/mycars/cv_lane_follower
```

Useful files to inspect:

```text
/home/jetson/projects/mycars/path_follower/myconfig.py
/home/jetson/projects/mycars/path_follower/manage.py
/home/jetson/projects/mycars/path_follower/my_joystick.py
/home/jetson/projects/mycars/cv_lane_follower/myconfig.py
/home/jetson/projects/mycars/cv_lane_follower/manage.py
/home/jetson/projects/mycars/cv_lane_follower/oak_camera.py
```

---

## 7.7 Open the Entire Jetson Home Directory

To browse more of the Jetson file system, open:

```text
/home/jetson
```

This gives access to folders such as:

```text
/home/jetson/projects
/home/jetson/donkey
/home/jetson/dsc190_ws
/home/jetson/sensorfusion
/home/jetson/david
```

---

## 7.8 Open the Installed DonkeyCar Package

The installed DonkeyCar package is located inside the Python environment:

```text
/home/jetson/donkey/lib/python3.8/site-packages/donkeycar
```

Useful installed DonkeyCar template files:

```text
/home/jetson/donkey/lib/python3.8/site-packages/donkeycar/templates/cv_control.py
/home/jetson/donkey/lib/python3.8/site-packages/donkeycar/templates/path_follow.py
/home/jetson/donkey/lib/python3.8/site-packages/donkeycar/templates/cfg_cv_control.py
/home/jetson/donkey/lib/python3.8/site-packages/donkeycar/templates/cfg_path_follow.py
/home/jetson/donkey/lib/python3.8/site-packages/donkeycar/templates/complete.py
```

Avoid editing the installed DonkeyCar package directly. Instead, edit local project files under:

```text
/home/jetson/projects/mycars
```

---

## 7.9 Important VS Code Notes

If the folder picker shows:

```text
/home/jetson/
```

you can manually type the folder path you want:

```text
/home/jetson/projects/mycars
```

Then click:

```text
OK
```

Do **not** click:

```text
Show Local
```

because that switches the file browser back to your local computer instead of the Jetson.

---

## 7.10 Remote SSH Troubleshooting

### Permission denied

If VS Code shows:

```text
Permission denied (publickey,password)
```

make sure the SSH config contains the correct user:

```sshconfig
User jetson
```

A correct config looks like:

```sshconfig
Host ucsd-agx-03
    HostName <JETSON_IP>
    User jetson
```

Then reconnect using:

```text
Remote-SSH: Connect to Host
```

### Wrong IP address

If the Jetson IP changed, run this on the Jetson:

```bash
myip
```

or:

```bash
ip addr show wlan0
```

Then update your local SSH config:

```bash
nano ~/.ssh/config
```

Update:

```sshconfig
HostName <NEW_JETSON_IP>
```

### Test SSH manually

From your local terminal:

```bash
ssh jetson@<JETSON_IP>
```

If this does not work, VS Code Remote SSH will not work either.

---

# 8. USB Device Names

List all connected USB serial devices:

```bash
ls /dev/ttyACM*
```

Expected devices may include:

```bash
/dev/ttyACM0
/dev/ttyACM1
/dev/ttyACM2
/dev/ttyACM3
```

If devices are not detected, power cycle the cables by unplugging and replugging them.

USB names can change if ports are swapped. If the car stops working after moving USB cables, check the detected devices again:

```bash
ls /dev/ttyACM*
```

To identify each device:

```bash
for dev in /dev/ttyACM*; do
    echo "----- $dev -----"
    udevadm info -q property -n $dev | grep -E "ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_USB_INTERFACE_NUM="
done
```

Known device types:

```text
ChibiOS = VESC
Septentrio = GPS
```

Example:

```text
/dev/ttyACM0 = ChibiOS = VESC
/dev/ttyACM1 = Septentrio GPS
/dev/ttyACM2 = Septentrio GPS
```

If needed, inspect stable serial names:

```bash
ls -l /dev/serial/by-id/
```

---

# 9. Editing Files on the Jetson

Use `nano` for simple terminal editing:

```bash
nano <filename>
```

Example:

```bash
nano myconfig.py
```

Use `vim` if preferred:

```bash
vim myconfig.py
```

Use VS Code on the Jetson if available:

```bash
code --no-sandbox .
```

Example:

```bash
code --no-sandbox my_joystick.py
```

When editing through VS Code Remote SSH, open:

```text
/home/jetson/projects/mycars
```

and edit files through the VS Code file explorer.

---

# 10. DonkeyCar Project Setup

Activate the DonkeyCar Python environment:

```bash
source ~/donkey/bin/activate
```

The main DonkeyCar project path is:

```bash
~/projects/mycars/path_follower
```

Go to the project:

```bash
cd ~/projects/mycars/path_follower
```

Important files:

```text
manage.py
myconfig.py
my_joystick.py
test_js0_mapping.py
```

File purposes:

- `manage.py`: main file used to run the car
- `myconfig.py`: car configuration, VESC configuration, GPS configuration, and driving parameters
- `my_joystick.py`: joystick/controller mapping
- `test_js0_mapping.py`: script for checking joystick input mapping

---

# 11. Running the Car with DonkeyCar

## 11.1 Run with Joystick

Activate the environment:

```bash
source ~/donkey/bin/activate
```

Go to the path follower directory:

```bash
cd ~/projects/mycars/path_follower
```

Run with joystick enabled:

```bash
python manage.py drive --js
```

The `--js` flag is recommended when the physical joystick/controller is working.

A working run should eventually print:

```text
Recording Change = False
Setting Recording = False
```

---

## 11.2 Run Without Joystick

If the physical joystick is not working, run without `--js`:

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage.py drive
```

This uses the web UI for control.

---

## 11.3 Stop DonkeyCar

Use:

```text
Ctrl-C
```

Or kill any running `manage.py drive` process:

```bash
pkill -f "manage.py drive"
```

Check if any process is still running:

```bash
ps aux | grep '[m]anage.py'
```

---

# 12. DonkeyCar Web UI

After running:

```bash
python manage.py drive
```

or:

```bash
python manage.py drive --js
```

open the web UI:

```text
http://ucsd-agx-03.local:8887/drive
```

If `.local` does not work, use the Jetson IP:

```text
http://<JETSON_IP>:8887/drive
```

Example:

```text
http://192.168.139.178:8887/drive
```

Click:

```text
Start Vehicle
```

Then use the web joystick area to drive.

Common modes:

```text
(U)ser        = manual driving
Auto (S)teer = autopilot controls steering, user controls throttle
Full (A)uto  = autopilot controls steering and throttle
```

For manual driving, use:

```text
(U)ser
```

---

# 13. Joystick and Remote Control

## 13.1 General Remote Notes

- The pairing process is documented in the ECE 191 documentation.
- Use the scroller to navigate.
- Click using `select`.
- Go into `tools` on the controller interface when debugging.
- The right toggle controls forward/backward motion.
- The left toggle controls steering.

---

## 13.2 Check if the Controller Is Working

Check joystick device:

```bash
ls /dev/input/js*
```

Test joystick input:

```bash
jstest /dev/input/js0
```

If `jstest` is not installed:

```bash
sudo apt update
sudo apt install joystick
```

Then rerun:

```bash
jstest /dev/input/js0
```

Run the DonkeyCar joystick mapping test:

```bash
cd ~/projects/mycars/path_follower
python3 test_js0_mapping.py
```

This script uses `my_joystick.py` and shows which controller inputs are being pressed.

---

## 13.3 Debugging Controller Issues

If the controller does not work:

1. Power cycle the receiver by unplugging and replugging the controller USB cable.
2. Check that the receiver is solid red, not blinking.
3. If the controller says:

```text
Telemetry lost
```

that can happen during connection issues.

If the numbers change a lot on their own during joystick testing, the receiver may have a problem.

If the receiver on the car is blinking red for several seconds or minutes, the receiver and controller are likely not paired correctly.

---

## 13.4 Running with the Radio Master Controller

Before running the car:

1. Make sure the receiver on the car is solid red.
2. Turn on the Radio Master controller.
3. Confirm the receiver is still solid red.

Then run:

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage.py drive --js
```

If the receiver is blinking red, fix the pairing before driving.

---

# 14. DonkeyCar Configuration Notes

Open the config file:

```bash
cd ~/projects/mycars/path_follower
nano myconfig.py
```

Things commonly configured in `myconfig.py`:

- VESC serial port
- GPS serial port
- VESC speed scaling
- Steering scale
- Steering offset
- Throttle scale
- Baudrate
- Path following PID values
- Joystick/web control settings

Keep the VESC baudrate at:

```python
VESC_BAUDRATE = 115200
```

The current working drivetrain is:

```python
DRIVE_TRAIN_TYPE = "VESC"
```

Common VESC settings:

```python
VESC_MAX_SPEED_PERCENT = 0.2
VESC_SERIAL_PORT = "/dev/ttyACM0"
VESC_HAS_SENSOR = True
VESC_START_HEARTBEAT = True
VESC_BAUDRATE = 115200
VESC_TIMEOUT = 0.05
VESC_STEERING_SCALE = 0.5
VESC_STEERING_OFFSET = 0.45
```

If the car drives forward but veers off, adjust the steering scale or steering offset in `myconfig.py`.

---

# 15. Path Following and PID

The existing path following setup lives in:

```bash
~/projects/mycars/path_follower
```

Open the config:

```bash
cd ~/projects/mycars/path_follower
vim myconfig.py
```

or:

```bash
nano myconfig.py
```

Start the car:

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage.py drive --js
```

If the joystick is not working:

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage.py drive
```

Path recording workflow:

1. Start the vehicle.
2. Use the right trigger at least twice to record the `(0, 0)` origin point.
3. Use the left trigger to start recording the path.
4. Drive the car forward and around the desired route.
5. Stop recording.
6. Save/load the path as needed.
7. Use path following mode to follow the recorded route.

If the car goes out of control, lift it by the two back wheels.

This can happen when path following is malfunctioning.

Important path-following config values:

```python
PATH_FILENAME = "data/donkey_path.csv"
PATH_MIN_DIST = 0.2
PATH_SEARCH_LENGTH = None
PATH_LOOK_AHEAD = 2
PATH_LOOK_BEHIND = 1

PID_P = -0.1
PID_I = 0.005
PID_D = -0.3
PID_THROTTLE = 0.34
USE_CONSTANT_THROTTLE = False
```

---

# 16. Running the Car with ROS Keyboard Teleop

This uses one terminal for the DonkeyCar vehicle loop and another terminal for ROS keyboard teleop.

## Terminal 1: DonkeyCar Vehicle Loop

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage_ros_drive.py --drivetrain
```

## Terminal 2: ROS Keyboard Teleop

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash
ros2 run robocar_drive_bridge keyboard_teleop
```

Keyboard controls:

```text
W = increase throttle
A = decrease throttle / reverse if throttle becomes negative
S = decrease steering toward -1
D = increase steering toward 1
```

---

# 17. ROS 2 Basic Commands

List all visible ROS 2 topics:

```bash
ros2 topic list
```

Echo a topic:

```bash
ros2 topic echo <topic_name>
```

Examples:

```bash
ros2 topic echo /oak/rgb/image_raw
```

```bash
ros2 topic echo /livox/lidar
```

If nothing appears, then the topic is not publishing or the current shell/container cannot see it.

---

# 18. Docker Basic Commands

See running containers:

```bash
docker ps
```

See available Docker images:

```bash
docker images
```

Stop a container:

```bash
docker stop <container_name>
```

Enter a running container:

```bash
docker exec -it <container_name> /bin/bash
```

Example:

```bash
docker exec -it ros2_camera_lidar_fusion /bin/bash
```

---

# 19. Camera and LiDAR ROS 2 Nodes

The camera and LiDAR nodes are in:

```bash
~/david/real-last-try
```

Go to the repository:

```bash
cd ~/david/real-last-try
```

Build the camera and LiDAR Docker containers:

```bash
docker compose -f docker-compose.yml -f docker-compose.arm64.yml build
```

After the build finishes, open two terminals.

---

## Terminal 1: Launch Camera Node

```bash
cd ~/david/real-last-try
bash launch_camera_host.sh
```

---

## Terminal 2: Launch LiDAR Node

The LiDAR is connected through Ethernet and usually has an IP of the form:

```text
192.168.1.xx
```

The last two digits are usually:

```text
99
```

Launch the LiDAR node:

```bash
cd ~/david/real-last-try
bash launch_lidar_foxy_host.sh 99
```

Make sure the red-taped USB is plugged into `12V`. It can slip out.

---

## Check Camera and LiDAR Topics

After both nodes are running:

```bash
ros2 topic list
```

You should see:

```text
/livox/lidar
/oak/rgb/image_raw
```

Check the camera stream:

```bash
ros2 topic echo /oak/rgb/image_raw
```

Check the LiDAR stream:

```bash
ros2 topic echo /livox/lidar
```

Both should show streaming output.

---

# 20. Sensor Fusion

The sensor fusion project is located at:

```bash
~/sensorfusion/ros2_camera_lidar_fusion
```

The Docker directory is:

```bash
~/sensorfusion/ros2_camera_lidar_fusion/docker
```

---

## 20.1 Start Sensor Fusion Docker

After the camera and LiDAR nodes are running:

```bash
cd ~/sensorfusion/ros2_camera_lidar_fusion/docker
bash run.sh
```

Check running containers:

```bash
docker ps
```

You should see the sensor fusion container running.

---

## 20.2 Enter the Sensor Fusion Container Manually

```bash
docker exec -it ros2_camera_lidar_fusion /bin/bash
```

---

## 20.3 Check ROS Topics Inside the Container

Inside the container:

```bash
ros2 topic list
```

You should see:

```text
/livox/lidar
/oak/rgb/image_raw
```

Echo LiDAR:

```bash
ros2 topic echo /livox/lidar
```

Echo camera:

```bash
ros2 topic echo /oak/rgb/image_raw
```

---

## 20.4 Build and Launch Sensor Fusion

Inside the container:

```bash
cd /ros2_ws
```

Show available launch options:

```bash
bash launch.sh
```

Build the workspace:

```bash
colcon build
```

Source the build file for data collection:

```bash
source install/setup.bash
bash launch.sh 1
```

---

## 20.5 Camera Calibration

Inside the sensor fusion container:

```bash
cd /ros2_ws
bash launch.sh 1
```

This computes the checkerboard calibration.

After calibration, values are saved on the Jetson under:

```bash
~/sensorfusion/ros2_camera_lidar_fusion/config
```

Important calibration files:

```text
camera_extrinsic_calibration.yaml
camera_intrinsic_calibration.yaml
```

Open the config directory:

```bash
cd ~/sensorfusion/ros2_camera_lidar_fusion/config
ls
```

More checkerboard angles may be needed for better camera and LiDAR calibration.

---

## 20.6 LiDAR Calibration

Inside the sensor fusion container:

```bash
cd /ros2_ws
bash launch.sh 2
```

After LiDAR calibration, check that the fused output topic exists:

```bash
ros2 topic list
```

You should see:

```text
/sensorfusion_out
```

Echo the fused output:

```bash
ros2 topic echo /sensorfusion_out
```

---

# 21. Foxglove Visualization

Foxglove is used instead of RViz for live ROS visualization.

General workflow:

1. Start the camera node.
2. Start the LiDAR node.
3. Start the sensor fusion Docker container if needed.
4. Start the Foxglove bridge.
5. Open Foxglove in a browser.
6. Connect to the Jetson WebSocket.

---

## 21.1 Install Foxglove Bridge

If Foxglove bridge is not installed:

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

For this car on ROS Humble:

```bash
sudo apt install ros-humble-foxglove-bridge
```

If you get:

```text
E: Unable to locate package ros-humble-foxglove-bridge
```

run:

```bash
sudo apt update
```

Then retry:

```bash
sudo apt install ros-humble-foxglove-bridge
```

---

## 21.2 Launch Foxglove Bridge

Run:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Or specify the port manually:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Expected output:

```text
[foxglove_bridge]: Starting foxglove_bridge
[foxglove_bridge]: Server listening on port 8765
```

You should also see camera and LiDAR topics advertised:

```text
/oak/rgb/image_raw
/livox/lidar
/livox/imu
```

---

## 21.3 Connect Through Foxglove

Open a Chromium-based browser and go to:

```text
https://app.foxglove.dev
```

Connect using:

```text
ws://<JETSON_IP>:8765
```

Example:

```text
ws://192.168.139.178:8765
```

If access is required, ask the team member with UCSD email access.

---

## 21.4 Foxglove Bridge Notes

Default WebSocket port:

```text
8765
```

Default address:

```text
0.0.0.0
```

Useful Foxglove bridge options:

```text
port
address
topic_whitelist
service_whitelist
param_whitelist
client_topic_whitelist
capabilities
num_threads
min_qos_depth
max_qos_depth
include_hidden
use_sim_time
```

Capabilities include:

```text
clientPublish
parameters
parametersSubscribe
services
connectionGraph
assets
time
```

Diagnostic topic if client count publishing is enabled:

```text
/foxglove_bridge/client_count
```

---

## 21.5 Foxglove Security Notes

- TLS/WSS can be enabled if needed.
- If TLS is enabled, both `certfile` and `keyfile` must be provided.
- Asset URI allowlists should be configured carefully so sensitive files are not exposed.
- Foxglove bridge blocks unsafe URI paths with consecutive `..`.

---

## 21.6 Full Camera, LiDAR, Sensor Fusion, and Foxglove Workflow

### Terminal 1: Camera

```bash
cd ~/david/real-last-try
bash launch_camera_host.sh
```

### Terminal 2: LiDAR

```bash
cd ~/david/real-last-try
bash launch_lidar_foxy_host.sh 99
```

### Terminal 3: Sensor Fusion Docker

```bash
cd ~/sensorfusion/ros2_camera_lidar_fusion/docker
bash run.sh
```

Inside the container:

```bash
ros2 topic list
ros2 topic echo /oak/rgb/image_raw
ros2 topic echo /livox/lidar
```

### Terminal 4: Foxglove Bridge

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then open Foxglove and connect to:

```text
ws://<JETSON_IP>:8765
```

---

# 22. GPS and Septentrio

The GPS is directly connected through USB. A GPS driver is needed to publish GPS coordinates into ROS 2.

GPS workflow:

1. Read GPS from USB.
2. Use the GPS driver to publish GPS data into ROS 2.
3. Create/use ROS 2 navigation topics.
4. Let Foxglove subscribe to the ROS 2 navigation topics.

---

## 22.1 Check Raw GPS Over Serial

Run:

```bash
sudo picocom -b 115200 /dev/ttyACM3
```

If `/dev/ttyACM3` is wrong, check USB devices:

```bash
ls /dev/ttyACM*
```

Then retry with the correct device.

---

## 22.2 Show Raw Septentrio Topic

Terminal 1:

```bash
cd ~/dsc190_ws
source /opt/ros/foxy/setup.bash
ros2 launch septentrio_gnss_driver rover.py file_name:=septentrio.yaml
```

Terminal 2:

```bash
cd ~/dsc190_ws
source /opt/ros/foxy/setup.bash
ros2 topic list | grep -E 'navsatfix|gpsfix|pvt'
ros2 topic hz /pvtgeodetic
ros2 topic echo /pvtgeodetic
```

The command that shows the streaming values is:

```bash
ros2 topic echo /pvtgeodetic
```

---

# 23. Working Version Check

To see the currently working versions for the car, including ROS 2, DepthAI C++, DepthAI Python, camera, and launch setup:

```bash
cat WORKING_VERSIONS.txt
```

---

# 24. Common Troubleshooting

## 24.1 USB Devices Not Detected

Check devices:

```bash
ls /dev/ttyACM*
```

If missing, unplug and replug the USB cables.

---

## 24.2 USB Devices Swapped

If USB ports are changed, device names may swap.

Check devices:

```bash
ls /dev/ttyACM*
```

Identify devices:

```bash
for dev in /dev/ttyACM*; do
    echo "----- $dev -----"
    udevadm info -q property -n $dev | grep -E "ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_USB_INTERFACE_NUM="
done
```

Known device types:

```text
Septentrio = GPS
ChibiOS = VESC
```

Then update serial paths in:

```bash
nano ~/projects/mycars/path_follower/myconfig.py
```

---

## 24.3 VESC Permission Error

If the VESC port has permission issues:

```bash
sudo chmod a+rw /dev/ttyACM0
```

Then rerun DonkeyCar:

```bash
cd ~/projects/mycars/path_follower
source ~/donkey/bin/activate
python manage.py drive
```

---

## 24.4 VESC Firmware Response Error

If you see an error like:

```text
invalid literal for int() with base 10: 'None'
```

DonkeyCar opened the serial port, but `pyvesc` did not get a valid firmware response.

Check that `/dev/ttyACM0` is actually the VESC:

```bash
for dev in /dev/ttyACM*; do
    echo "----- $dev -----"
    udevadm info -q property -n $dev | grep -E "ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_USB_INTERFACE_NUM="
done
```

You want:

```text
ID_MODEL=ChibiOS_RT_Virtual_COM_Port
```

Also check:

```bash
sudo chmod a+rw /dev/ttyACM0
```

Make sure the car has main power and the VESC is powered.

---

## 24.5 Joystick Not Working

Check joystick device:

```bash
ls /dev/input/js*
```

Test joystick:

```bash
jstest /dev/input/js0
```

Run mapping test:

```bash
cd ~/projects/mycars/path_follower
python3 test_js0_mapping.py
```

If the controller does not respond, power cycle the receiver cable.

---

## 24.6 Receiver Blinking Red

If the receiver is blinking red, it is not properly paired with the controller.

Fix the controller pairing before running:

```bash
python manage.py drive --js
```

If the controller is not working, use the web UI:

```bash
python manage.py drive
```

---

## 24.7 DonkeyCar Web UI Not Opening

If `.local` is slow or does not work, use the direct IP:

```text
http://<JETSON_IP>:8887/drive
```

Example:

```text
http://192.168.139.178:8887/drive
```

Check whether the server is listening:

```bash
ss -ltnp | grep 8887
```

Test locally on the Jetson:

```bash
curl -I http://localhost:8887/drive
```

---

## 24.8 Car Drives Fine Lifted but Weird on the Ground

If the car behaves fine when lifted but makes a loud sound or behaves strangely on the ground, the issue is likely load-related.

Possible causes:

```text
1. Motor/VESC cogging under load because throttle is too low
2. Gear, belt, axle, or wheel hub slipping under load
3. Battery/main power sagging under load
4. Something physically blocking the drivetrain on the ground
```

Test one thing at a time:

```text
1. Steering only: left/right with no throttle
2. Throttle only: forward/backward with steering centered
```

If steering is quiet but forward/backward is loud, focus on the drive motor/drivetrain.

---

## 24.9 Foxglove Bridge Package Not Found

If this fails:

```bash
sudo apt install ros-humble-foxglove-bridge
```

Run:

```bash
sudo apt update
```

Then retry:

```bash
sudo apt install ros-humble-foxglove-bridge
```

---

## 24.10 Foxglove Running but Topics Missing

Check ROS topics:

```bash
ros2 topic list
```

Make sure these are present:

```text
/oak/rgb/image_raw
/livox/lidar
/livox/imu
```

If they are missing, restart the camera and LiDAR nodes.

---

## 24.11 Camera Topic Check

```bash
ros2 topic echo /oak/rgb/image_raw
```

Expected behavior: raw image message data streams in the terminal.

---

## 24.12 LiDAR Topic Check

```bash
ros2 topic echo /livox/lidar
```

Expected behavior: point cloud message data streams in the terminal.

---

## 24.13 Sensor Fusion Output Missing

Inside the sensor fusion container:

```bash
cd /ros2_ws
ros2 topic list
```

Check for:

```text
/sensorfusion_out
```

If missing, rerun calibration or relaunch sensor fusion:

```bash
bash launch.sh 1
bash launch.sh 2
```

Then check again:

```bash
ros2 topic echo /sensorfusion_out
```

---

# 25. Useful Paths

Main DonkeyCar path follower:

```bash
~/projects/mycars/path_follower
```

Experimental CV lane follower:

```bash
~/projects/mycars/cv_lane_follower
```

DonkeyCar Python environment:

```bash
~/donkey
```

Installed DonkeyCar package:

```bash
~/donkey/lib/python3.8/site-packages/donkeycar
```

Camera and LiDAR node repository:

```bash
~/david/real-last-try
```

Sensor fusion repository:

```bash
~/sensorfusion/ros2_camera_lidar_fusion
```

Sensor fusion Docker directory:

```bash
~/sensorfusion/ros2_camera_lidar_fusion/docker
```

Sensor fusion config directory:

```bash
~/sensorfusion/ros2_camera_lidar_fusion/config
```

ROS 2 workspace:

```bash
~/dsc190_ws
```

---

# 26. Useful Files

DonkeyCar main runner:

```text
manage.py
```

DonkeyCar ROS runner:

```text
manage_ros_drive.py
```

Car configuration:

```text
myconfig.py
```

Joystick configuration:

```text
my_joystick.py
```

Joystick mapping test:

```text
test_js0_mapping.py
```

Camera intrinsic calibration:

```text
camera_intrinsic_calibration.yaml
```

Camera extrinsic calibration:

```text
camera_extrinsic_calibration.yaml
```

Septentrio config:

```text
septentrio.yaml
```

Working versions:

```text
WORKING_VERSIONS.txt
```

---

# 27. Useful References

Foxglove bridge documentation:

```text
https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge
```

Foxglove SDK repository:

```text
https://github.com/foxglove/foxglove-sdk
```

Terminal output workflow reference:

```text
https://docs.google.com/document/d/1OQKwKXm2MO3m6HLtvVt6QCz0qEVv-6aCSwUk-aFW8tI/edit?tab=t.6sq152crbhdf
```

---

---

# 28. ROS 2 Lane Detection Progress

This section documents the ROS 2 lane detection work completed while the VESC/controller were not reliable. This pipeline does **not** drive the car yet. It only reads the OAK camera image, detects lane markings, computes lane-center error, publishes debug topics, and saves debug images for inspection.

---

## 28.1 Goal

The goal is to use **classical computer vision**, not deep learning, to detect lane markings from the OAK camera.

Current perception-only pipeline:

```text
/oak/rgb/image_raw
        ↓
lane_follower ROS 2 package
        ↓
detect colored lane markings
        ↓
compute lane center error
        ↓
publish debug topics and save debug images
```

This will later be connected to steering once the VESC/drivetrain and controller issues are fixed.

---

## 28.2 What Was Completed

Completed so far:

```text
Created ROS 2 lane_follower package
Implemented one-line detection
Implemented two-line lane detection
Implemented fitted two-line detection
Implemented curved-path lane detection
Recorded OAK camera data with rosbag
Verified lane-center error topics update correctly
Saved debug images to /tmp for inspection
```

The lane follower package is located at:

```bash
~/dsc190_ws/src/lane_follower
```

Important files:

```text
~/dsc190_ws/src/lane_follower/lane_follower/one_line_follower.py
~/dsc190_ws/src/lane_follower/lane_follower/two_line_follower.py
~/dsc190_ws/src/lane_follower/lane_follower/two_line_fit_follower.py
~/dsc190_ws/src/lane_follower/lane_follower/curved_lane_follower.py
~/dsc190_ws/src/lane_follower/setup.py
```

---

## 28.3 Start the OAK Camera Node

Open Terminal 1:

```bash
cd ~/david/real-last-try
bash launch_camera_host.sh
```

Expected output should include:

```text
Camera device initialized successfully
Publishing RGB frames on /oak/rgb/image_raw
```

The camera publishes:

```text
/oak/rgb/image_raw
```

If the camera disconnects and reconnects, the node may print something like:

```text
Camera disconnected during operation
Camera connection lost. Will attempt to reconnect...
Camera reconnected successfully
```

If this happens repeatedly, restart the camera node.

---

## 28.4 Source ROS 2 and the Workspace

Open a new terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash
```

Check that the camera topic exists:

```bash
ros2 topic list
```

Expected topic:

```text
/oak/rgb/image_raw
```

Optional camera rate check:

```bash
ros2 topic hz /oak/rgb/image_raw
```

Stop with:

```text
Ctrl-C
```

---

## 28.5 Build the Lane Follower Package

After editing any file in the `lane_follower` package, rebuild:

```bash
cd ~/dsc190_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select lane_follower
source ~/dsc190_ws/install/setup.bash
```

Check that ROS sees the executables:

```bash
ros2 pkg executables lane_follower
```

Expected output after all current nodes are added:

```text
lane_follower one_line_follower
lane_follower two_line_follower
lane_follower two_line_fit_follower
lane_follower curved_lane_follower
```

---

## 28.6 Fix Duplicate Package Build Error

If `colcon build` fails with:

```text
Duplicate package names not supported:
- lane_follower:
  - backups/lane_follower_backup_...
  - src/lane_follower
```

it means a backup copy of the package is still inside the workspace. Move backups completely outside `~/dsc190_ws`:

```bash
mkdir -p ~/ros_backups
mv ~/dsc190_ws/backups/lane_follower_backup_* ~/ros_backups/
```

If the backup is still inside `src`, move it out:

```bash
mkdir -p ~/ros_backups
mv ~/dsc190_ws/src/lane_follower_backup_* ~/ros_backups/
```

Then rebuild:

```bash
cd ~/dsc190_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select lane_follower
source ~/dsc190_ws/install/setup.bash
```

Check for package files inside the workspace:

```bash
find ~/dsc190_ws -name package.xml
```

There should only be one `lane_follower/package.xml` under:

```text
/home/jetson/dsc190_ws/src/lane_follower/package.xml
```

---

# 29. One-Line Detection

The one-line detector finds one colored line and computes its error from the image center.

---

## 29.1 Run One-Line Detection

For a blue line:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower one_line_follower --ros-args -p color:=blue
```

For a yellow line:

```bash
ros2 run lane_follower one_line_follower --ros-args -p color:=yellow
```

For a green line:

```bash
ros2 run lane_follower one_line_follower --ros-args -p color:=green
```

Expected startup output:

```text
One-line follower started.
Subscribing to: /oak/rgb/image_raw
Publishing debug image to: /lane/debug_image
Publishing mask to: /lane/mask
Publishing center error to: /lane/center_error
```

---

## 29.2 Check One-Line Output Topics

Check lane topics:

```bash
ros2 topic list | grep lane
```

Expected one-line topics:

```text
/lane/center_error
/lane/debug_image
/lane/mask
```

Check the center error:

```bash
ros2 topic echo /lane/center_error
```

Interpretation:

```text
error near 0      = line is centered
negative error    = line is left of the target center
positive error    = line is right of the target center
```

Example values observed:

```text
data: -0.025
data: 0.000
data: 0.031
```

---

# 30. Basic Two-Line Lane Detection

The basic two-line detector detects a left lane boundary and a right lane boundary, then computes the midpoint between them.

---

## 30.1 Run Two-Line Detection

For two blue lane lines:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower two_line_follower --ros-args -p color:=blue
```

For two yellow lane lines:

```bash
ros2 run lane_follower two_line_follower --ros-args -p color:=yellow
```

For two green lane lines:

```bash
ros2 run lane_follower two_line_follower --ros-args -p color:=green
```

Expected startup output:

```text
Two-line follower started.
Subscribing to: /oak/rgb/image_raw
Publishing debug image to: /lane/two_line_debug_image
Publishing mask to: /lane/two_line_mask
Publishing center error to: /lane/two_line_center_error
Publishing detected flag to: /lane/two_line_detected
Saving debug images to: /tmp/lane_debug
```

---

## 30.2 Check Two-Line Output Topics

Check the two-line center error:

```bash
ros2 topic echo /lane/two_line_center_error
```

Check whether both lanes are detected:

```bash
ros2 topic echo /lane/two_line_detected
```

Expected detection output:

```text
data: true
```

The node also logs the detected lane centers:

```text
BOTH LANES | error=-0.081 | L=124 | R=464
BOTH LANES | error=-0.055 | L=124 | R=481
```

Where:

```text
L = detected left lane x-position
R = detected right lane x-position
error = normalized center error
```

---

## 30.3 How Two-Line Error Works

The two-line detector computes:

```text
lane_center_x = (left_line_x + right_line_x) / 2
image_center_x = image_width / 2

error = (lane_center_x - image_center_x) / image_center_x
```

Interpretation:

```text
error near 0      = car/camera is centered between the two lanes
negative error    = computed lane center is left of image center
positive error    = computed lane center is right of image center
```

Example observation:

```text
Initial error: -0.08125
After moving the right lane farther right: -0.053125
```

This means the lane midpoint moved closer to the image center because the error moved closer to zero.

---

## 30.4 Basic Two-Line Debug Images

The basic two-line detector saves debug images to:

```bash
/tmp/lane_debug
```

Check saved files:

```bash
ls -lh /tmp/lane_debug
```

Expected files:

```text
two_line_debug.jpg
two_line_mask.png
```

Open these in VS Code Remote SSH:

```text
/tmp/lane_debug/two_line_debug.jpg
/tmp/lane_debug/two_line_mask.png
```

The debug image should show:

```text
cyan box        = region of interest
white line      = image center
green dots      = detected lane centers
magenta line    = computed lane center
text overlay    = lane detection status and error
```

The mask image should show:

```text
white pixels = detected tape/lane color
black pixels = background
```

---

# 31. Fitted Two-Line Lane Detection

The fitted two-line detector improves on the basic two-line detector. Instead of only using centroids, it fits a line to each lane boundary and computes the lane center at a lookahead row.

This is useful for cleaner straight or slightly curved paths.

---

## 31.1 Run Fitted Two-Line Detection

For blue tape:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.70 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=150
```

For yellow tape:

```bash
ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=yellow \
  -p roi_y_start:=0.70 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=150
```

Expected startup output:

```text
Two-line fitted lane follower started.
Subscribing to: /oak/rgb/image_raw
Publishing /lane/v2_center_error
Publishing /lane/v2_fake_steering
Saving debug images to /tmp/lane_debug_v2
```

---

## 31.2 Check Fitted Two-Line Output Topics

Check available topics:

```bash
ros2 topic list | grep lane
```

Expected fitted detector topics:

```text
/lane/v2_center_error
/lane/v2_fake_steering
/lane/v2_debug_image
/lane/v2_mask
/lane/v2_detected
```

Check lane center error:

```bash
ros2 topic echo /lane/v2_center_error
```

Check fake steering command:

```bash
ros2 topic echo /lane/v2_fake_steering
```

Check detection status:

```bash
ros2 topic echo /lane/v2_detected
```

Expected detection output:

```text
data: true
```

---

## 31.3 Fitted Two-Line Debug Images

The fitted detector saves debug images to:

```bash
/tmp/lane_debug_v2
```

Check saved files:

```bash
ls -lh /tmp/lane_debug_v2
```

Expected files:

```text
lane_v2_debug.jpg
lane_v2_mask.png
```

Open these in VS Code Remote SSH:

```text
/tmp/lane_debug_v2/lane_v2_debug.jpg
/tmp/lane_debug_v2/lane_v2_mask.png
```

The debug image should show:

```text
green lines             = fitted left/right lane boundaries
green dots              = detected lane positions at the lookahead row
magenta/pink line       = computed lane center
white vertical line     = image center
yellow horizontal line  = lookahead row
text overlay            = detection status, error, and fake steering
```

The mask image should show:

```text
white pixels = detected tape/lane color
black pixels = background
```

---

## 31.4 Tune the Fitted Detector

If the detector misses the lanes, use a wider ROI:

```bash
ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.55 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=100
```

If the detector sees too much background noise, use a lower/tighter ROI:

```bash
ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.80 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=200
```

If blue detection is too weak, use a broader custom HSV threshold:

```bash
ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=custom \
  -p h_low:=85 \
  -p s_low:=30 \
  -p v_low:=30 \
  -p h_high:=140 \
  -p s_high:=255 \
  -p v_high:=255 \
  -p roi_y_start:=0.70 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=150
```

If it detects too much background, tighten the blue threshold:

```bash
ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=custom \
  -p h_low:=95 \
  -p s_low:=80 \
  -p v_low:=80 \
  -p h_high:=125 \
  -p s_high:=255 \
  -p v_high:=255 \
  -p roi_y_start:=0.70 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=150
```

---

# 32. Curved Lane Detection

The curved lane detector is the current best perception node. It is designed to work for both straight paths and curved paths.

Instead of fitting one straight line to each lane boundary, it:

```text
1. Scans multiple horizontal bands in the image
2. Finds left and right lane positions in each band
3. Computes center points between the lanes
4. Fits a centerline curve through the center points
5. Chooses a lookahead target point
6. Computes center error and fake steering
```

This should handle paths that transition between straight and curved sections.

---

## 32.1 Run Curved Lane Detection

For blue tape:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.45 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=20 \
  -p lookahead_y_fraction:=0.75
```

For yellow tape:

```bash
ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=yellow \
  -p roi_y_start:=0.45 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=20 \
  -p lookahead_y_fraction:=0.75
```

Expected startup output:

```text
Curved lane follower started.
Subscribing to: /oak/rgb/image_raw
Publishing /lane/curved_center_error
Publishing /lane/curved_fake_steering
Saving debug images to /tmp/lane_debug_curved
```

---

## 32.2 Check Curved Lane Output Topics

Check center error:

```bash
ros2 topic echo /lane/curved_center_error
```

Check fake steering:

```bash
ros2 topic echo /lane/curved_fake_steering
```

Check detection status:

```bash
ros2 topic echo /lane/curved_detected
```

Expected detection output:

```text
data: true
```

---

## 32.3 Curved Lane Debug Images

The curved lane detector saves debug images to:

```bash
/tmp/lane_debug_curved
```

Check saved files:

```bash
ls -lh /tmp/lane_debug_curved
```

Expected files:

```text
curved_lane_debug.jpg
curved_lane_mask.png
```

Open these in VS Code Remote SSH:

```text
/tmp/lane_debug_curved/curved_lane_debug.jpg
/tmp/lane_debug_curved/curved_lane_mask.png
```

The debug image should show:

```text
green dots              = detected left/right lane points at multiple rows
pink dots               = computed lane center points
pink curve              = fitted centerline
yellow horizontal line  = lookahead row
yellow dot              = lookahead target point for steering
white vertical line     = image center
text overlay            = error, fake steering, and number of center points
```

The mask image should show:

```text
white pixels = detected tape/lane color
black pixels = background
```

---

## 32.4 Tune the Curved Lane Detector

If the curved path is not detected well, use a larger ROI:

```bash
ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.35 \
  -p roi_y_end:=1.00 \
  -p num_bands:=10 \
  -p min_pixels_per_band:=15 \
  -p lookahead_y_fraction:=0.70
```

If the detector sees too much noise, make the ROI stricter:

```bash
ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.55 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=40 \
  -p lookahead_y_fraction:=0.75
```

If blue thresholding needs tuning, use custom HSV:

```bash
ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=custom \
  -p h_low:=85 \
  -p s_low:=30 \
  -p v_low:=30 \
  -p h_high:=140 \
  -p s_high:=255 \
  -p v_high:=255 \
  -p roi_y_start:=0.45 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=20 \
  -p lookahead_y_fraction:=0.75
```

---

# 33. Recording and Replaying Lane Data

Rosbag can be used to record camera data once, then replay it later without moving or driving the car.

---

## 33.1 Record OAK Camera Data

Make sure the camera node is running.

Then open another terminal:

```bash
mkdir -p ~/lane_data

source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 bag record -o ~/lane_data/two_lane_test_$(date +%Y%m%d_%H%M%S) /oak/rgb/image_raw
```

Move the car by hand or move the lane tape by hand for 20–30 seconds.

Stop recording:

```text
Ctrl-C
```

Example recorded bag path:

```text
/home/jetson/lane_data/two_lane_test_20260424_153907
```

---

## 33.2 Replay Recorded Camera Data

Stop the live camera node if needed.

Replay the bag:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 bag play --loop ~/lane_data/<BAG_FOLDER_NAME>
```

Example:

```bash
ros2 bag play --loop ~/lane_data/two_lane_test_20260424_153907
```

Then run a detector in another terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.45 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=20 \
  -p lookahead_y_fraction:=0.75
```

This allows detector tuning without using the VESC or remote controller.

---

# 34. Recommended Lane Detection Terminal Layout

Use this layout for the current perception-only workflow.

---

## Terminal 1: Camera

```bash
cd ~/david/real-last-try
bash launch_camera_host.sh
```

---

## Terminal 2: Lane Detector

Current recommended detector:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.45 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=20 \
  -p lookahead_y_fraction:=0.75
```

For the fitted straight-lane detector:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 run lane_follower two_line_fit_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.70 \
  -p roi_y_end:=1.00 \
  -p min_pixels:=150
```

---

## Terminal 3: Check Error Output

For the curved detector:

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash

ros2 topic echo /lane/curved_center_error
```

Check fake steering:

```bash
ros2 topic echo /lane/curved_fake_steering
```

Check detection:

```bash
ros2 topic echo /lane/curved_detected
```

---

## Terminal 4: Check Debug Images

For curved detector:

```bash
ls -lh /tmp/lane_debug_curved
```

Open in VS Code Remote SSH:

```text
/tmp/lane_debug_curved/curved_lane_debug.jpg
/tmp/lane_debug_curved/curved_lane_mask.png
```

For fitted detector:

```bash
ls -lh /tmp/lane_debug_v2
```

Open in VS Code Remote SSH:

```text
/tmp/lane_debug_v2/lane_v2_debug.jpg
/tmp/lane_debug_v2/lane_v2_mask.png
```

For basic two-line detector:

```bash
ls -lh /tmp/lane_debug
```

Open in VS Code Remote SSH:

```text
/tmp/lane_debug/two_line_debug.jpg
/tmp/lane_debug/two_line_mask.png
```

---

# 35. How to Interpret Lane Error

The lane detectors compute a normalized error:

```text
error = (lane_center_x - image_center_x) / image_center_x
```

Interpretation:

```text
error near 0      = lane center is aligned with image center
negative error    = lane center appears left of image center
positive error    = lane center appears right of image center
```

Example:

```text
error = -0.4
```

means the lane center is significantly left of the image center.

For future control:

```text
If the steering direction is wrong, flip the sign of Kp.
```

---

# 36. Current Lane Detection Status

What works:

```text
OAK camera publishes /oak/rgb/image_raw
lane_follower package builds successfully
one_line_follower runs
two_line_follower runs
two_line_fit_follower runs
curved_lane_follower runs
/lane/center_error publishes one-line error
/lane/two_line_center_error publishes basic two-line midpoint error
/lane/v2_center_error publishes fitted two-line midpoint error
/lane/curved_center_error publishes curve-aware center error
fake steering outputs are published for fitted and curved detectors
debug images are saved to /tmp folders
rosbag recording works for /oak/rgb/image_raw
```

What is not connected yet:

```text
No steering command is being sent to the car yet
No throttle command is being sent to the car yet
VESC/drivetrain issue still needs to be fixed before autonomous driving
Remote controller pairing still needs to be fixed separately
```

---

# 37. Next Step After VESC/Controller Are Fixed

Once manual driving is reliable, connect the lane-center error to steering.

Basic control idea:

```python
steering = Kp * error + Kd * (error - previous_error)
```

Recommended starting workflow:

```text
1. Use manual throttle.
2. Let lane detector control steering only.
3. Keep speed very low.
4. Keep the car ready to be lifted for safety.
5. If steering goes the wrong way, flip the sign of Kp.
```

Start with the curved detector because it should handle both straight and curved paths:

```bash
ros2 run lane_follower curved_lane_follower --ros-args \
  -p color:=blue \
  -p roi_y_start:=0.45 \
  -p roi_y_end:=1.00 \
  -p num_bands:=8 \
  -p min_pixels_per_band:=20 \
  -p lookahead_y_fraction:=0.75
```

---

# 38. Updated Future Work

Potential future directions:

- Connect lane-center error to steering through the ROS drive bridge.
- Test autonomous steering with manual throttle first.
- Add low constant throttle only after steering is stable.
- Tune PID gains for lane centering.
- Improve robustness to lighting changes and shadows.
- Add better color threshold tuning for different track surfaces.
- Add support for non-blue/non-yellow lane markings.
- Integrate LiDAR or GPS only after camera lane following is stable.
- Add obstacle detection using LiDAR.
- Compare classical CV lane following with freespace segmentation later.
