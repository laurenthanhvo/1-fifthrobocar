# UCSD RoboCar — Power On + Camera Feed Quickstart

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

3. Enter the password when prompted.

4. Go to the camera fusion directory:

   ```bash
   cd ece191/Camera_fusion2025/src/
   ```

5. Run the streaming script:

   ```bash
   stream_UVC.py
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

## Expected Results

* Car is reachable via SSH at `192.168.11.156`
* NoMachine shows the host as `ucsd-agx-03`
* Camera stream script runs in Terminal #1
* CV test runs in Terminal #2 and displays/validates the feed

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
