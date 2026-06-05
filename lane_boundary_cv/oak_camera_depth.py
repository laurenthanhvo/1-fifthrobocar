"""
oak_camera_depth.py
Drop-in replacement for oak_camera.py that outputs BOTH:
    cam/image_array  — RGB frame (uint8, H×W×3)
    cam/depth_array  — stereo depth map (uint16, H×W, values in mm, 0 = unknown)

The depth map is aligned to the RGB frame on-device via StereoDepth,
so every RGB pixel has a corresponding depth pixel — no extrinsic
calibration needed.

Usage in manage.py:
    from oak_camera_depth import OakCameraDepth

    V.add(
        OakCameraDepth(
            image_w=cfg.IMAGE_W,
            image_h=cfg.IMAGE_H,
            fps=cfg.DRIVE_LOOP_HZ,
            undistort=getattr(cfg, "OAK_UNDISTORT", False),
            calibration_file=getattr(cfg, "OAK_CALIBRATION_FILE", None),
            undistort_alpha=getattr(cfg, "OAK_UNDISTORT_ALPHA", 0.0),
        ),
        outputs=["cam/image_array", "cam/depth_array"],
        threaded=True,
    )

myconfig.py values:
    OAK_DEPTH_PRESET    = "HIGH_ACCURACY"  # or "HIGH_DENSITY"
    OAK_DEPTH_MEDIAN    = 5                # median filter kernel: 0=off, 3, 5, 7
    OAK_MONO_RESOLUTION = "400P"           # "400P" or "800P"
    OAK_EXTENDED_DISPARITY = False         # True = shorter min range (~35cm → ~17cm)
"""

import time
import threading
import cv2
import numpy as np
import depthai as dai


class OakCameraDepth:
    """
    Threaded Donkeycar part — outputs aligned RGB + stereo depth from OAK-D.

    Outputs:
        cam/image_array  — RGB numpy array  (H, W, 3) uint8
        cam/depth_array  — depth numpy array (H, W)   uint16, units = mm
                           0 = no reading (unknown distance)
    """

    def __init__(
        self,
        image_w=640,
        image_h=400,
        fps=20,
        undistort=False,
        calibration_file=None,
        undistort_alpha=0.0,
        depth_preset="HIGH_ACCURACY",
        median_filter=5,
        mono_resolution="800P",
        extended_disparity=False,
    ):
        self.image_w    = image_w
        self.image_h    = image_h
        self.fps        = fps
        self.running    = True

        # State shared between update() thread and run_threaded()
        self._lock        = threading.Lock()
        self._rgb_frame   = np.zeros((image_h, image_w, 3), dtype=np.uint8)
        self._depth_frame = np.zeros((image_h, image_w),    dtype=np.uint16)

        # Undistortion (same as oak_camera.py)
        self.undistort_map1 = None
        self.undistort_map2 = None
        self._undistort     = undistort
        self._cal_file      = calibration_file
        self._alpha         = float(np.clip(undistort_alpha, 0.0, 1.0))

        # ── Build DepthAI pipeline ─────────────────────────────────────────
        pipeline = dai.Pipeline()

        # --- Color camera ---
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setPreviewSize(image_w, image_h)
        cam_rgb.setPreviewKeepAspectRatio(True)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(fps)

        # --- Mono cameras for stereo ---
        mono_res_map = {
            "400P": dai.MonoCameraProperties.SensorResolution.THE_400_P,
            "800P": dai.MonoCameraProperties.SensorResolution.THE_800_P,
        }
        mono_res = mono_res_map.get(mono_resolution,
                                    dai.MonoCameraProperties.SensorResolution.THE_400_P)

        mono_left  = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(mono_res)
        mono_right.setResolution(mono_res)
        mono_left.setCamera("left")
        mono_right.setCamera("right")
        mono_left.setFps(fps)
        mono_right.setFps(fps)

        # --- StereoDepth node ---
        stereo = pipeline.create(dai.node.StereoDepth)

        # Preset: HIGH_ACCURACY for obstacles (less noise, fewer holes)
        preset_map = {
            "HIGH_ACCURACY": dai.node.StereoDepth.PresetMode.HIGH_ACCURACY,
            "HIGH_DENSITY":  dai.node.StereoDepth.PresetMode.HIGH_DENSITY,
        }
        stereo.setDefaultProfilePreset(
            preset_map.get(depth_preset,
                           dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        )

        # Align depth to color frame — no extrinsic calibration needed
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # Output depth at the same size as our color preview
        stereo.setOutputSize(image_w, image_h)

        # Quality settings
        stereo.setLeftRightCheck(True)       # reduces flickering artifacts
        stereo.setExtendedDisparity(extended_disparity)
        stereo.setSubpixel(False)            # keep uint16 mm output

        # Median filter to fill small holes
        median_map = {
            0: dai.StereoDepthConfig.MedianFilter.MEDIAN_OFF,
            3: dai.StereoDepthConfig.MedianFilter.KERNEL_3x3,
            5: dai.StereoDepthConfig.MedianFilter.KERNEL_5x5,
            7: dai.StereoDepthConfig.MedianFilter.KERNEL_7x7,
        }
        stereo.initialConfig.setMedianFilter(
            median_map.get(median_filter,
                           dai.StereoDepthConfig.MedianFilter.KERNEL_5x5)
        )

        # --- Link nodes ---
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # --- XLink outputs ---
        xout_rgb   = pipeline.create(dai.node.XLinkOut)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        # ── Start device ──────────────────────────────────────────────────
        self._device     = dai.Device(pipeline)
        self._q_rgb      = self._device.getOutputQueue("rgb",   maxSize=1, blocking=False)
        self._q_depth    = self._device.getOutputQueue("depth", maxSize=1, blocking=False)

        # Set up undistortion maps if requested
        self._setup_undistortion()

    # ------------------------------------------------------------------
    # Undistortion (identical to oak_camera.py)
    # ------------------------------------------------------------------

    def _setup_undistortion(self):
        if not self._undistort:
            print("OAK undistortion disabled.")
            return
        if not self._cal_file:
            print("WARNING: OAK_UNDISTORT=True but no OAK_CALIBRATION_FILE set.")
            self._undistort = False
            return
        try:
            with np.load(self._cal_file) as cal:
                K = np.array(cal["camera_matrix"], dtype=np.float64)
                D = np.array(cal["dist_coeffs"],   dtype=np.float64)
                cal_w = int(cal["image_width"].item())
                cal_h = int(cal["image_height"].item())

            if (cal_w, cal_h) != (self.image_w, self.image_h):
                raise RuntimeError(
                    f"Calibration size ({cal_w}×{cal_h}) != camera size "
                    f"({self.image_w}×{self.image_h})"
                )

            new_K, _ = cv2.getOptimalNewCameraMatrix(
                K, D, (self.image_w, self.image_h),
                self._alpha, (self.image_w, self.image_h)
            )
            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
                K, D, None, new_K,
                (self.image_w, self.image_h), cv2.CV_16SC2
            )
            print(f"OAK undistortion enabled (alpha={self._alpha}).")
        except Exception as e:
            print(f"WARNING: OAK undistortion setup failed: {e}")
            self._undistort = False

    # ------------------------------------------------------------------
    # Frame processing helpers
    # ------------------------------------------------------------------

    def _process_rgb(self, packet):
        frame = packet.getCvFrame()                       # BGR uint8
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)   # → RGB
        if self._undistort and self.undistort_map1 is not None:
            frame = cv2.remap(frame, self.undistort_map1,
                              self.undistort_map2, cv2.INTER_LINEAR)
        return frame

    def _process_depth(self, packet):
        # getCvFrame() returns uint16 depth in mm, 0 = unknown
        depth = packet.getCvFrame().astype(np.uint16)
        return depth

    # ------------------------------------------------------------------
    # Donkeycar threaded part interface
    # ------------------------------------------------------------------

    def update(self):
        """Background thread — grabs frames continuously."""
        while self.running:
            rgb_pkt   = self._q_rgb.tryGet()
            depth_pkt = self._q_depth.tryGet()

            if rgb_pkt is not None:
                rgb = self._process_rgb(rgb_pkt)
                with self._lock:
                    self._rgb_frame = rgb

            if depth_pkt is not None:
                depth = self._process_depth(depth_pkt)
                with self._lock:
                    self._depth_frame = depth

            time.sleep(1.0 / max(self.fps * 2, 1))   # poll at 2× frame rate

    def run_threaded(self):
        """Called each vehicle tick — returns latest RGB + depth."""
        with self._lock:
            return self._rgb_frame.copy(), self._depth_frame.copy()

    def run(self):
        """Fallback for non-threaded use."""
        rgb_pkt   = self._q_rgb.tryGet()
        depth_pkt = self._q_depth.tryGet()
        if rgb_pkt   is not None: self._rgb_frame   = self._process_rgb(rgb_pkt)
        if depth_pkt is not None: self._depth_frame = self._process_depth(depth_pkt)
        return self._rgb_frame, self._depth_frame

    def shutdown(self):
        self.running = False
        time.sleep(0.2)
        if hasattr(self, "_device"):
            self._device.close()
