# DSC 190 Working Car Documentation

**Last Updated:** 04/21/2026  
**Project Report:** (insert link later)

> **Important:** This README contains device IPs and the Jetson password. Do not commit this publicly unless those credentials are removed or replaced with placeholders.

---

## 1. Jetson Access

### Current Jetson IP

```bash
192.168.139.178
```

If you are physically on the Jetson, run:

```bash
myip
```

If the IP address changed, run:

```bash
ip addr show wlan0
```

Look for the `inet` field. The first number string after `inet` is the Jetson IP address.

---

### SSH into the Jetson

From your personal computer:

```bash
ssh -x jetson@<ip-address-of-jetson>
```

Example:

```bash
ssh -x jetson@192.168.139.178
```

If using X forwarding:

```bash
ssh -X jetson@<ip-address-of-jetson>
```

---

## 2. Connecting Through Hotspot

1. Connect the Jetson to the hotspot using the Ubuntu network menu.
2. On the Jetson, find the IP address:

```bash
ifconfig
```

3. Connect your personal computer to the same hotspot.
4. SSH into the Jetson:

```bash
ssh -X jetson@<hotspot-ip-address>
```

Example:

```bash
ssh -X jetson@10.53.210.191
```

---

## 3. Hardware and Power Notes

### Power Distribution

- Do **not** plug anything that needs `5V` into `12V`. This can burn the component.
- `20V` is for the VESC because the motor needs higher power.
- If the servo does not work, check the small connector on the right side.
- The servo wiring colors are flipped:
  - Usually black is ground.
  - On this setup, **white is ground**.
  - Match **white to black**.
- Ask for a lid if the electronics are exposed.

---

### Powering On the Jetson and Car

1. Connect the main power cable on the car.
2. Face the car forward.
3. Press the power button on the top-left corner of the power distribution board.
4. Press either the first or third button on the Jetson computer, which is the black box.

---

## 4. USB Device Names

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

Then update the correct serial device paths in:

```bash
~/projects/mycars/path_follower/myconfig.py
```

---

## 5. Working Version Check

To see the currently working versions for the car, including ROS 2, DepthAI C++, DepthAI Python, camera, and launch setup:

```bash
cat WORKING_VERSIONS.txt
```

---

## 6. Editing Files on the Jetson

Use `nano` for simple terminal editing:

```bash
nano <filename>
```

Example:

```bash
nano myconfig.py
```

Use VS Code if available:

```bash
code --no-sandbox .
```

Example for opening the joystick file:

```bash
code --no-sandbox my_joystick.py
```

If working outside a GUI environment, use:

```bash
nano my_joystick.py
```

or:

```bash
vim myconfig.py
```

---

## 7. VS Code Remote SSH Setup

To browse the Jetson file system from your computer:

1. Open VS Code on your computer.
2. Install the **Remote - SSH** extension.
3. Click the `+` button for a new remote.
4. Add the SSH command:

```bash
ssh jetson@<ip-address>
```

Example:

```bash
ssh jetson@192.168.139.178
```

5. Select platform:

```bash
Linux
```

---

## 8. DonkeyCar Setup

Go into the DonkeyCar container directory:

```bash
cd donkeycontainer/
```

Check the environment source script:

```bash
cat sourceForPathfollowercar.sh
```

Activate the environment:

```bash
source sourceForPathfollowercar.sh
```

The main DonkeyCar project path is:

```bash
~/projects/mycars/path_follower
```

Go to the path follower car directory:

```bash
cd ~/projects/mycars/path_follower
```

Important files:

```bash
manage.py
myconfig.py
my_joystick.py
```

File purposes:

- `manage.py`: main file used to run the car.
- `myconfig.py`: contains car configuration, VESC configuration, serial ports, baudrate, and movement parameters.
- `my_joystick.py`: contains controller and joystick mappings.

---

## 9. Running the Car with DonkeyCar and Joystick

Activate the DonkeyCar environment:

```bash
source ~/donkey/bin/activate
```

Go to the path follower directory:

```bash
cd ~/projects/mycars/path_follower
```

Run the car with joystick enabled:

```bash
python3 manage.py drive --js
```

or:

```bash
python manage.py drive --js
```

The `--js` flag is recommended because it automatically uses the joystick.

If you run without `--js`:

```bash
python3 manage.py drive
```

The car will default to the web UI. The terminal output will show the web URL for driving.

A working run should eventually print:

```bash
Recording Change = False
Setting Recording = False
```

---

## 10. DonkeyCar Web UI

To check if the joystick and web interface are working, open:

```bash
ucsd-agx03.local:8887/drive
```

If this does not work, use the Jetson IP address in the browser instead.

---

## 11. Joystick and Remote Control

### General Remote Notes

- The pairing process is in the ECE 191 documentation.
- Use the scroller to navigate.
- Click using `select`.
- Go into `tools` on the controller interface when debugging.
- The right toggle controls forward and backward motion.
- The left toggle controls steering.

---

### Check if the Controller is Working

Run:

```bash
jstest /dev/input/js0
```

You can also run:

```bash
python3 test_js0_mapping.py
```

From the path follower directory:

```bash
cd ~/projects/mycars/path_follower
python3 test_js0_mapping.py
```

This script uses the `my_joystick.py` file and shows which controller input is being pressed.

---

### Debugging Controller Issues

If the controller does not work:

1. Power cycle the receiver by unplugging and replugging the controller USB cable.
2. Check that the receiver is solid red, not blinking.
3. If the controller says:

```bash
Telemetry lost
```

that can happen during connection issues.

If the numbers change a lot on their own during joystick testing, the receiver may have a problem.

If the receiver on the car is blinking red for several seconds or minutes, the receiver and controller are likely not paired correctly.

---

## 12. Running the Car with the Radio Master Controller

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

## 13. DonkeyCar Configuration Notes

Open the config file:

```bash
cd ~/projects/mycars/path_follower
nano myconfig.py
```

or:

```bash
cd ~/projects/mycars/path_follower
code --no-sandbox .
```

Things that can be configured in `myconfig.py`:

- VESC serial port
- GPS serial port
- Steering scale
- Throttle scale
- Baudrate
- Car movement parameters

Keep the baudrate at:

```bash
115200
```

If the car drives forward but veers off, adjust the steering scale in `myconfig.py`.

Do not run max speed at `0.6` while the car is off the ground. Without resistance from the ground, the car may crash or shut down.

---

## 14. ROS 2 Basic Commands

List all visible ROS 2 topics:

```bash
ros2 topic list
```

Echo a topic:

```bash
ros2 topic echo <topic_name>
```

Example:

```bash
ros2 topic echo /oak/rgb/image_raw
```

Example:

```bash
ros2 topic echo /livox/lidar
```

If nothing appears, then the topic is not publishing or the current shell/container cannot see it.

---

## 15. Docker Basic Commands

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

## 16. Building and Running Camera and LiDAR ROS 2 Nodes

Go to the ROS node repository:

```bash
cd ~/david/real-last-try
```

Build the camera and LiDAR Docker containers:

```bash
docker compose -f docker-compose.yml -f docker-compose.arm64.yml build
```

After the build finishes, open two terminals.

---

### Terminal 1: Launch Camera Node

```bash
cd ~/david/real-last-try
bash launch_camera_host.sh
```

---

### Terminal 2: Launch LiDAR Node

The LiDAR is connected through Ethernet and usually has an IP of the form:

```bash
192.168.1.xx
```

The last two digits are usually:

```bash
99
```

Launch the LiDAR node:

```bash
cd ~/david/real-last-try
bash launch_lidar_foxy_host.sh 99
```

Make sure the red-taped USB is plugged into `12V`. It can slip out.

---

### Check Camera and LiDAR Topics

After both nodes are running:

```bash
ros2 topic list
```

You should see:

```bash
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

## 17. Sensor Fusion Docker Setup

After the camera and LiDAR nodes are running, go to the sensor fusion Docker directory:

```bash
cd ~/sensorfusion/ros2_camera_lidar_fusion/docker
```

Run the Docker environment:

```bash
bash run.sh
```

Check running containers:

```bash
docker ps
```

You should see three containers running.

Inside the Docker container, check visible ROS topics:

```bash
ros2 topic list
```

You should be able to see the outside camera and LiDAR topics:

```bash
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

## 18. Entering the Sensor Fusion Container Manually

If needed, enter the running sensor fusion container manually:

```bash
docker exec -it ros2_camera_lidar_fusion /bin/bash
```

---

## 19. Sensor Fusion Build and Launch

Inside the sensor fusion container:

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

Run the built package directory if needed:

```bash
./build/ros2_camera_lidar_fusion/
```

---

## 20. Camera Calibration

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

The important calibration files are:

```bash
camera_extrinsic_calibration.yaml
camera_intrinsic_calibration.yaml
```

Open the config directory from the Jetson:

```bash
cd ~/sensorfusion/ros2_camera_lidar_fusion/config
ls
```

More checkerboard angles are needed for better camera and LiDAR calibration.

---

## 21. LiDAR Calibration

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

```bash
/sensorfusion_out
```

Echo the fused output:

```bash
ros2 topic echo /sensorfusion_out
```

---

## 22. Foxglove Visualization

Foxglove is used instead of RViz for live ROS visualization.

The flow is:

1. Start the camera node.
2. Start the LiDAR node.
3. Start the sensor fusion Docker container if needed.
4. Start the Foxglove bridge.
5. Open Foxglove in a browser.
6. Connect to the Jetson WebSocket.

---

### Install Foxglove Bridge

If Foxglove bridge is not installed, run:

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

For this car on ROS Humble:

```bash
sudo apt install ros-humble-foxglove-bridge
```

If you get an error like:

```bash
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

### Launch Foxglove Bridge

Run:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Or specify the port manually:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Expected output should include something like:

```bash
[foxglove_bridge]: Starting foxglove_bridge
[foxglove_bridge]: Server listening on port 8765
```

You should also see camera and LiDAR topics advertised:

```bash
/oak/rgb/image_raw
/livox/lidar
/livox/imu
```

---

### Connect Foxglove

Open a Chromium-based browser and go to:

```bash
https://app.foxglove.dev
```

Connect using:

```bash
ws://<JETSON_IP>:8765
```

Example:

```bash
ws://192.168.139.178:8765
```

If access is required, ask Kanishk for UCSD email access.

---

### Foxglove Bridge Notes

Default WebSocket port:

```bash
8765
```

Default address:

```bash
0.0.0.0
```

Useful Foxglove bridge options:

```bash
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

```bash
clientPublish
parameters
parametersSubscribe
services
connectionGraph
assets
time
```

Diagnostic topic if client count publishing is enabled:

```bash
/foxglove_bridge/client_count
```

---

### Foxglove Security Notes

- TLS/WSS can be enabled if needed.
- If TLS is enabled, both `certfile` and `keyfile` must be provided.
- Asset URI allowlists should be configured carefully so sensitive files are not exposed.
- Foxglove bridge blocks unsafe URI paths with consecutive `..`.

---

### Building Foxglove Bridge From Source

Only do this if the apt install method does not work.

Clone the Foxglove SDK:

```bash
git clone https://github.com/foxglove/foxglove-sdk
```

Go to the ROS directory:

```bash
cd foxglove-sdk/ros
```

Build:

```bash
make
```

If Foxglove bridge was built outside the ROS workspace, source the setup file:

```bash
source install/local_setup.bash
```

Build Docker image:

```bash
make docker-build
```

Run tests:

```bash
make test
```

Docs:

```bash
https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge
https://github.com/foxglove/foxglove-sdk
```

---

## 23. Full Camera, LiDAR, Sensor Fusion, and Foxglove Workflow

Use this when starting visualization from scratch.

---

### Terminal 1: Camera

```bash
cd ~/david/real-last-try
bash launch_camera_host.sh
```

---

### Terminal 2: LiDAR

```bash
cd ~/david/real-last-try
bash launch_lidar_foxy_host.sh 99
```

---

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

---

### Terminal 4: Foxglove Bridge

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then open Foxglove and connect to:

```bash
ws://<JETSON_IP>:8765
```

---

## 24. Running the Car with ROS Keyboard Teleop

This uses one terminal for the DonkeyCar vehicle loop and another terminal for ROS keyboard teleop.

---

### Terminal 1: DonkeyCar Vehicle Loop

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage_ros_drive.py --drivetrain
```

---

### Terminal 2: ROS Keyboard Teleop

```bash
source /opt/ros/foxy/setup.bash
source ~/dsc190_ws/install/setup.bash
ros2 run robocar_drive_bridge keyboard_teleop
```

Keyboard controls:

```bash
W = increase throttle
A = decrease throttle / reverse if throttle becomes negative
S = decrease steering toward -1
D = increase steering toward 1
```

---

## 25. Path Following and PID

Open the config file:

```bash
cd ~/projects/mycars/path_follower
vim myconfig.py
```

Start the car normally:

```bash
source ~/donkey/bin/activate
cd ~/projects/mycars/path_follower
python manage.py drive --js
```

Path recording workflow:

1. Click the right trigger at least twice on the controller to record the `(0, 0)` origin point of the car.
2. Click the left trigger to start recording the path to follow.
3. Drive the car forward and then through a right turn.

If the car goes out of control, lift it by the two back wheels.

This can happen when path following is malfunctioning.

---

## 26. GPS and Septentrio

The GPS is directly connected through USB. It does not need a custom node for raw USB access, but a GPS driver is needed to publish GPS coordinates into ROS 2.

The GPS flow is:

1. Read GPS from USB.
2. Use the GPS driver to publish GPS data into ROS 2.
3. Create a ROS 2 navigation topic.
4. Let Foxglove subscribe to the ROS 2 navigation topic.

---

### Check Raw GPS Over Serial

Run:

```bash
sudo picocom -b 115200 /dev/ttyACM3
```

This should load GPS coordinates. You may need to run the command a few times.

If `/dev/ttyACM3` is wrong, check USB devices:

```bash
ls /dev/ttyACM*
```

Then retry with the correct device.

---

### Show Raw Septentrio Topic

Run this on the Jetson.

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

The command that actually shows the streaming `-20000000000.0` values is:

```bash
ros2 topic echo /pvtgeodetic
```

---

## 27. Common Troubleshooting

### USB Devices Not Detected

Check devices:

```bash
ls /dev/ttyACM*
```

If missing, unplug and replug the USB cables.

---

### USB Devices Swapped

If USB ports are changed, device names may swap.

Check devices:

```bash
ls /dev/ttyACM*
```

Run DonkeyCar to inspect USB output:

```bash
cd ~/projects/mycars/path_follower
python3 manage.py drive
```

Then update serial paths in:

```bash
nano ~/projects/mycars/path_follower/myconfig.py
```

Known device types:

```bash
Septentrio = GPS
ChibiOS = VESC
```

---

### Joystick Not Working

Check joystick device:

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

### Receiver Blinking Red

If the receiver is blinking red, it is not properly paired with the controller.

Fix the controller pairing before running:

```bash
python manage.py drive --js
```

---

### Foxglove Bridge Package Not Found

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

### Foxglove Running but Topics Missing

Check ROS topics:

```bash
ros2 topic list
```

Make sure these are present:

```bash
/oak/rgb/image_raw
/livox/lidar
/livox/imu
```

If they are missing, restart the camera and LiDAR nodes.

---

### Camera Topic Check

```bash
ros2 topic echo /oak/rgb/image_raw
```

Expected behavior: raw image message data streams in the terminal.

---

### LiDAR Topic Check

```bash
ros2 topic echo /livox/lidar
```

Expected behavior: point cloud message data streams in the terminal.

---

### Sensor Fusion Output Missing

Inside the sensor fusion container:

```bash
cd /ros2_ws
ros2 topic list
```

Check for:

```bash
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

## 28. Useful Paths

DonkeyCar path follower:

```bash
~/projects/mycars/path_follower
```

DonkeyCar container directory:

```bash
~/donkeycontainer
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

## 29. Useful Files

DonkeyCar main runner:

```bash
manage.py
```

DonkeyCar ROS runner:

```bash
manage_ros_drive.py
```

Car configuration:

```bash
myconfig.py
```

Joystick configuration:

```bash
my_joystick.py
```

Joystick mapping test:

```bash
test_js0_mapping.py
```

Camera intrinsic calibration:

```bash
camera_intrinsic_calibration.yaml
```

Camera extrinsic calibration:

```bash
camera_extrinsic_calibration.yaml
```

Septentrio config:

```bash
septentrio.yaml
```

Working versions:

```bash
WORKING_VERSIONS.txt
```

---

## 30. Useful References

Foxglove bridge documentation:

```bash
https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge
```

Foxglove SDK repository:

```bash
https://github.com/foxglove/foxglove-sdk
```

Terminal output workflow reference:

```bash
https://docs.google.com/document/d/1OQKwKXm2MO3m6HLtvVt6QCz0qEVv-6aCSwUk-aFW8tI/edit?tab=t.6sq152crbhdf
```

---

## 31. Future Work Notes

Potential future direction:

```bash
vision language action model
```

This likely refers to adding a VLA-style model on top of the current car stack after the core driving, sensor, GPS, and visualization pipeline is stable.
