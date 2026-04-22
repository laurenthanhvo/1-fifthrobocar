# UCSD RoboRacer (1/5 Off-Road RoboCar)

This README walks through:
1) Powering on the RoboCar (Jetson AGX)
2) Getting the camera feed running (SSH + scripts)

---

## Prerequisites

- **Wi-Fi network:** `UCSDRoboCar`
- **Power:** **14.8V battery** connected *or* an external power source
- **Remote desktop:** NoMachine installed on your laptop

---

## 1) How to Turn On the Car

1. **Connect your laptop to Wi-Fi**
   - Network: `UCSDRoboCar`

2. **Verify power is connected**
   - Ensure the car is connected to a **14.8V battery** or a valid **power source**.
   - **Safety:** Make sure the **14.8V battery is connected to the battery alarm** (low-voltage alarm) before turning the car on.

3. **Turn on main power**
   - Press the **main power button** down.
   - You should see the **power symbol turn blue**.

4. **Enable HDMI / Jetson boot**
   - On the **right side of the car**, there are **three buttons** next to each other.
   - Press the **left-most** button **until the HDMI light turns on and starts blinking**.

5. **Confirm boot**
   - The **fan should turn on**.
   - The car should appear in NoMachine as:
     - `ucsd-agx-03`

---

## 2) How to Access the Camera Feed

You will run commands in **two terminals**.

### Terminal #1 (SSH + Start Stream)

1. SSH into the Jetson:

   ```bash
   ssh jetson@192.168.11.156
   ```
   
   If this does not work, you can try this method:
   ```bash
   ssh jetson@ucsd-agx-03.local
   ```

3. Enter the password when prompted.

4. Go to the camera fusion directory:

   ```bash
   cd ece191/Camera_fusion2025/src/
   ```

5. Run the streaming script:

   ```bash
   python3 stream_UVC.py
   ```

---

### Terminal #2 (Run CV Test)

1. Open a **new terminal window**.

2. Go to the same directory:

   ```bash
   cd ece191/Camera_fusion2025/src/
   ```

3. Run the test launcher:

   ```bash
   python3 test
   ```

4. Run the OpenCV test:

   ```bash
   test_cv.py
   ```

---

### Accessing the Color Camera

1. Open a **new terminal window**.

2. Go into this directory:

   ```bash
   cd depthai-python/examples/ColorCamera/
   ```

3. Run the streaming script:
   
   ```bash
   python3 rgb_preview.py
   ```

This shows the color camera attached to the front of the car, displaying at around 20 frames per second. 

### Accessing the Monochrome Cameras

These cameras shows the view from a left and right point of view in monochrome. 

1. In a new (or old terminal after pressing Ctrl+C), change into this directory:
   
   ```bash
   cd depthai-python/examples/MonoCamera/
   ```

2. Run this streaming script:
   ```bash
   python3 mono_preview.py
   ```

### Accessing the Object Tracker

1. In a new terminal, change directories to this:

   ```bash
   cd depthai-python/examples/ObjectTracker/
   ```

2. Run the streaming script:
   ```bash
   python3 object_tracker.py
   ```

## Expected Results

* Car is reachable via SSH at `192.168.11.156` or `local`
* NoMachine shows the host as `ucsd-agx-03`
* Camera stream script runs in Terminal #1
* CV test runs in Terminal #2 and displays/validates the feed

---

## 3) DonkeyCar Access + Manual Driving (Verified on `ucsd-agx-03`)

This section is the correct startup flow for running DonkeyCar manual drive on the 1/5 off-road RoboCar.

### Environment + Paths (verified)

* Jetson host: `ucsd-agx-03` (or `ucsd-agx-03.local`)
* User: `jetson` (**not root**)
* Virtual env path: `~/donkey/bin/activate`
* Car project path: `~/projects/mycars/deep_learning_car`
* Drive entrypoint: `python manage.py drive`
* Web UI:

  * `http://ucsd-agx-03.local:8887/drive`
  * or `http://192.168.11.156:8887/drive`

---

### 3.1 Start DonkeyCar (manual driving + web UI)

1. SSH into Jetson:

   ```bash
   ssh jetson@ucsd-agx-03.local
   ```

   (or)

   ```bash
   ssh jetson@192.168.11.156
   ```

2. Confirm you are not root:

   ```bash
   whoami
   ```

   Expected:

   ```text
   jetson
   ```

3. Activate DonkeyCar virtual environment:

   ```bash
   source ~/donkey/bin/activate
   ```

   Optional check:

   ```bash
   which python
   ```

   Expected:

   ```text
   /home/jetson/donkey/bin/python
   ```

4. Go to the car project directory:

   ```bash
   cd ~/projects/mycars/deep_learning_car
   ```

5. (Optional) edit car overrides:

   ```bash
   nano myconfig.py
   ```

6. Start DonkeyCar:

   ```bash
   python manage.py drive
   ```

7. Open web controller in browser:

   ```text
   http://192.168.11.156:8887/drive
   ```

   (or)

   ```text
   http://ucsd-agx-03.local:8887/drive
   ```

---

### 3.2 Expected healthy startup logs

You should see lines like:

* `using donkey v5.0.0`
* `loading config file: .../deep_learning_car/config.py`
* `loading personal config over-rides from myconfig.py`
* `Starting Donkey Server...`
* `You can now go to ucsd-agx-03.local:8887 to drive your car.`
* `Creating VESC at port /dev/ttyACM0`
* `Starting vehicle at 20 Hz`

Camera can appear as either:

* `cfg.CAMERA_TYPE MOCK` (test mode), or
* `cfg.CAMERA_TYPE OAKD` (real OAK-D camera)

---

### 3.3 Controller notes

* Connect controller directly to Jetson (USB dongle/cable) or Bluetooth.
* If using PS4 controller, pair via Bluetooth before running `manage.py drive`.

> Note: HDMI is for display output, not controller data transport.

---

### 3.4 Recording behavior (what you should see)

When recording toggles ON in web UI/controller, logs show:

* `Recording Change = True`
* `Setting Recording = True`
* `recorded 10 records`, `recorded 20 records`, etc.

Data is saved under:

```text
~/projects/mycars/deep_learning_car/data/tub_*/
```

with a `manifest.json` in each tub folder.

---

### 3.5 After collecting training data

Train a simple lane-following model:

```bash
python manage.py train \
  --tub data \
  --model models/pilot.h5 \
  --type linear
```

Drive with trained model:

```bash
python manage.py drive --model models/pilot.h5
```

Then switch to AI mode in the web UI.

---

### 3.6 Clean shutdown

Stop with:

```bash
Ctrl + C
```

Expected normal shutdown lines:

* `Shutting down vehicle and its parts...`
* `Closing tub ...`
* `Closing manifest ...`
* Part profile summary table

You may occasionally see:

* `Task was destroyed but it is pending!`
* `KeyboardInterrupt` during teardown

---

### 3.7 Quick troubleshooting 

#### A) `-bash: ./: Is a directory`

You typed:

```bash
./ ls
```

Fix:

```bash
ls
```

#### B) `naon: command not found`

Typo. Use:

```bash
nano myconfig.py
```

#### C) Running as root breaks normal flow (`conda` not found, wrong env)

Use `jetson` account for DonkeyCar runtime.

#### D) OAK-D warning about unsupported resolution defaulting to 800P

This warning is non-fatal; startup can still succeed.

#### E) `404 GET /favicon.ico`

Harmless browser request, can be ignored.

---

### 3.8 Reference file location on Jetson

Your notes/readme file can be accessed at:

```bash
cd ~/donkeycontainer
nano donkeycar_readme.txt
```

---

## One-command sequence (copy/paste)

```bash
ssh jetson@ucsd-agx-03.local
source ~/donkey/bin/activate
cd ~/projects/mycars/deep_learning_car
python manage.py drive
```

Then open:

```text
http://192.168.11.156:8887/drive
```

---

## 4) Access LiDAR, Point Cloud, and Depth/Cameras

This section covers how to access:

1. Livox MID-360 LiDAR + point cloud in RViz
2. LiDAR ROS2 topic data in terminal
3. DepthAI depth/RGB/mono/object-tracking camera streams

---

### 4.1 LiDAR + Point Cloud (Livox MID-360)

#### A) Connect through NoMachine first

1. Open **NoMachine** and connect to:

   * `192.168.11.156`
2. Log in with the team credentials.
3. In the VM desktop, top-right network menu (**Ethernet**) → select:

   * **Profile 1**
4. Open a terminal in the VM.

---

#### B) Start Livox ROS2 driver + RViz point cloud

```bash
cd ~/lidar_test/src/ws_livox/src/livox_ros_driver2
./build.sh ROS2

cd ~/lidar_test/src/ws_livox
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

> If `source install/setup.bash` fails, you are likely in the wrong folder.
> It should be run from the workspace root: `~/lidar_test/src/ws_livox`.

---

### 4.2 How to check LiDAR data (distance / closeness)

From a new terminal:

```bash
cd ~/lidar_test/src/ws_livox
source install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /livox/lidar
```

* `/livox/lidar` prints a large stream of point data.
* As objects get closer, point coordinates/ranges should reflect shorter distances in the sensor frame.
* For easier visualization of near/far objects, use RViz point cloud view from the launch command above.

---

### 4.3 Access Depth, RGB, Mono (left/right), and Object Tracking Cameras

Start from Jetson home terminal:

#### A) Stereo depth preview

```bash
cd ~/depthai-python/examples/StereoDepth
python3 depth_preview.py
```

#### B) Color camera preview

```bash
cd ~/depthai-python/examples/ColorCamera
python3 rgb_preview.py
```

To inspect/edit preview size/settings:

```bash
cat rgb_preview.py
```

#### C) Object tracking

```bash
cd ~/depthai-python/examples/ObjectTracker
python3 object_tracker.py
python3 spatial_object_tracker.py
```

#### D) Mono cameras (left + right, black/white)

```bash
cd ~/depthai-python/examples/MonoCamera
python3 mono_preview.py
```

---

### 4.4 Quick troubleshooting

* **No data in RViz / ROS topics empty**

  * Confirm NoMachine session is on `192.168.11.156`
  * Confirm Ethernet is set to **Profile 1**
  * Re-source workspace:

    ```bash
    cd ~/lidar_test/src/ws_livox
    source install/setup.bash
    ```

* **`build.sh` not found**

  * Make sure you are in:
    `~/lidar_test/src/ws_livox/src/livox_ros_driver2`

* **DepthAI scripts not found**

  * Make sure you are under:
    `~/depthai-python/examples/...` before running each script

---

## 5) Camera Calibration Validation (OAK-D RGB) using Checkerboard

This section documents how we validated the **front OAK-D color camera** calibration (intrinsics + distortion) using a **checkerboard**, and computed **reprojection error** for both:

- **Factory calibration** stored on the OAK-D device (EEPROM)
- **Fresh OpenCV checkerboard calibration** (for comparison)

### Goal
- Confirm the camera is already calibrated and distortion correction works (straight lines become straight).
- Quantify calibration quality using **reprojection RMSE (pixels)**.
- Save artifacts (images + summary) for report/writeup.

### Prereqs
- Car is powered on and you can connect via **NoMachine** and/or **SSH**.
- OAK-D is working via DepthAI scripts.
- Checkerboard used:
  - **10 × 7 squares** → **9 × 6 interior corners**
  - Square size measured: **45 mm** → `0.045 m`

---

### 5.1 Open RGB camera feed (sanity check)

On the Jetson:

```bash
cd ~/depthai-python/examples/ColorCamera
python3 rgb_preview.py
````

Expected:

* Live RGB feed from the front OAK-D camera.

---

### 5.2 (Optional) Visual undistortion sanity check

DepthAI provides an example undistortion script:

```bash
cd ~/depthai-python/examples/ColorCamera
python3 rgb_undistort.py
```

Expected:

* Two windows: **Distorted** (raw) and **Undistorted**
* Undistorted image may look slightly **cropped/zoomed** (normal) because undistortion often crops valid pixels to avoid black borders.
* Straight edges near image boundaries should appear straighter in the undistorted view.

Notes:

* You may see warnings like “unsupported resolution… defaulting to 800P/720P” and Qt font warnings. These are non-fatal.

---

### 5.3 Capture checkerboard dataset (20–60 images)

We captured checkerboard images directly from the OAK-D ISP output at **1280×800**.

1. Go to ColorCamera examples:

```bash
cd ~/depthai-python/examples/ColorCamera
```

2. Run the capture script:

```bash
python3 capture_checkerboard.py
```

Controls:

* Press **s** in the camera window to save an image
* Press **q** to quit cleanly
  (Ctrl+C also exits, but prints a KeyboardInterrupt traceback—this is harmless.)

Saved image location:

* `~/camera_calib/images/`

Quick checks:

```bash
ls ~/camera_calib/images | wc -l
```

Capture guidelines (important for good corner detection):

* Fill ~30–70% of image with board (not tiny/far away)
* Keep board fully in frame (don’t crop edges)
* Vary pose: center + corners of FOV + near/far + tilted/skewed
* Avoid glare and motion blur

---

### 5.4 Compute reprojection error + save outputs

Run the validation/calibration script:

```bash
cd ~/depthai-python/examples/ColorCamera
python3 validate_checkerboard.py
```

This script:

* Detects checkerboard corners in the dataset
* Computes reprojection error for:

  * **Factory calibration** read from device EEPROM
  * **OpenCV calibration** computed from the checkerboard images
* Saves “before/after” undistortion images and a summary file

Outputs written to:

* `~/camera_calib/outputs/`

Contents:

* `sample_distorted.png`
* `sample_undistorted_factory.png`
* `sample_undistorted_opencv.png`
* `summary.txt`

Open the summary:

```bash
cat ~/camera_calib/outputs/summary.txt
```

Open the images (NoMachine GUI):

```bash
xdg-open ~/camera_calib/outputs/sample_distorted.png
xdg-open ~/camera_calib/outputs/sample_undistorted_factory.png
xdg-open ~/camera_calib/outputs/sample_undistorted_opencv.png
```

---

### 5.5 Results we observed (example from today)

Dataset:

* Total images: **56**
* Images used (corners found): **19**
* Board interior corners: **9×6**
* Square size: **0.045 m**
* Image size: **1280×800**

Reprojection RMSE (pixels):

* **Factory (EEPROM) mean RMSE:** ~**1.16 px**
* **OpenCV calibration mean RMSE:** ~**1.72 px**

Interpretation:

* Factory calibration performed **better** than our re-calibration on this dataset.
* Conclusion: **Use the device factory calibration** for rectification + measurement pipeline.

---

### 5.6 Download images + outputs to your laptop (keep a copy)

From your **laptop terminal** (Mac/Linux):

```bash
mkdir -p ~/Downloads/roboracer_cam_calib
scp -r jetson@192.168.11.156:/home/jetson/camera_calib/images ~/Downloads/roboracer_cam_calib/
scp -r jetson@192.168.11.156:/home/jetson/camera_calib/outputs ~/Downloads/roboracer_cam_calib/
```

Alternative hostname:

```bash
scp -r jetson@ucsd-agx-03.local:/home/jetson/camera_calib/images ~/Downloads/roboracer_cam_calib/
scp -r jetson@ucsd-agx-03.local:/home/jetson/camera_calib/outputs ~/Downloads/roboracer_cam_calib/
```

---

### 5.7 Troubleshooting

**Corners found is low (e.g., <10 usable images):**

* Board too small in frame → move closer
* Board partially cut off → keep full board visible
* Motion blur → hold still when pressing `s`
* Glare/reflections → change angle / lighting

**Undistorted image looks slightly zoomed:**

* Normal due to undistortion cropping to valid region (depends on implementation).

**KeyboardInterrupt traceback after quitting capture script:**

* Happens when using Ctrl+C. Prefer pressing **q** in the window for clean exit.

---

## Troubleshooting (Quick Checks)

* **NoMachine can’t find `ucsd-agx-03`:**

  * Re-check: main power symbol is **blue**, HDMI light is **blinking**, **fan is on**
  * Confirm your laptop is on `UCSDRoboCar`

* **SSH fails (`No route to host` / timeout):**

  * Confirm Wi-Fi `UCSDRoboCar`
  * Verify the Jetson is fully booted (fan on, HDMI blinking)

* **Command not found (e.g., `stream_UVC.py`):**

  * Confirm you are in:

    ```bash
    cd ece191/Camera_fusion2025/src/
    ```

---
