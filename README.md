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

Both were observed in your runs.

---

### 3.3 Controller notes

* Connect controller directly to Jetson (USB dongle/cable) or Bluetooth.
* In your logs, joystick input and web client connection were both working.
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

Those appeared in your logs after force interrupts and are common when stopping active async video tasks.

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

## Notes

* Keep Terminal #1 running while testing the feed in Terminal #2.
* If you power cycle the car, repeat the full “Turn On” steps before SSH.
