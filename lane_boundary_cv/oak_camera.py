import time

import cv2
import numpy as np
import depthai as dai


# ============================================================
# OAK camera image settings
# ============================================================

# Enable this to correct the curved / fisheye-looking image.
UNDISTORT_OAK_IMAGE = True
OAK_CALIBRATION_FILE = "camera_calibration.npz"

# 0.0 = crop more, fewer black edges
# 1.0 = preserve more field of view, may show black edges
OAK_UNDISTORT_ALPHA = 0.0

# This was the previous working behavior.
PREVIEW_KEEP_ASPECT_RATIO = True


class OakCamera:
    """
    Minimal DepthAI/OAK camera part for DonkeyCar.

    DonkeyCar expects threaded camera parts to provide:
        update()
        run_threaded()
        shutdown()

    Output:
        cam/image_array as an RGB numpy array with shape (IMAGE_H, IMAGE_W, 3)
    """

    def __init__(
        self,
        image_w=160,
        image_h=120,
        fps=20,
        undistort=False,
        calibration_file=None,
        undistort_alpha=0.0,
    ):
        self.image_w = image_w
        self.image_h = image_h
        self.fps = fps
        self.running = True

        self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)

        self.undistort_map1 = None
        self.undistort_map2 = None

        self.undistort_oak_image = undistort
        self.calibration_file = calibration_file
        self.oak_undistort_alpha = float(np.clip(undistort_alpha, 0.0, 1.0))

        pipeline = dai.Pipeline()

        cam = pipeline.createColorCamera()

        # OV9782 requires 800_P or 720_P, so set 800_P explicitly.
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

        # DonkeyCar expects image arrays at the configured size.
        cam.setPreviewSize(image_w, image_h)
        cam.setPreviewKeepAspectRatio(PREVIEW_KEEP_ASPECT_RATIO)

        # Request RGB output from DepthAI.
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam.setInterleaved(False)
        cam.setFps(fps)

        xout = pipeline.createXLinkOut()
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)

        self.device = dai.Device(pipeline)
        self.queue = self.device.getOutputQueue(
            name="rgb",
            maxSize=1,
            blocking=False,
        )

        self._setup_undistortion()

    def _get_rgb_socket(self):
        """
        DepthAI versions differ in naming:
        - older versions use RGB
        - newer versions use CAM_A
        """
        try:
            return dai.CameraBoardSocket.RGB
        except AttributeError:
            return dai.CameraBoardSocket.CAM_A

    def _setup_undistortion(self):
        """
        Load checkerboard calibration and create undistortion maps.
        """
        if not self.undistort_oak_image:
            print("OAK undistortion disabled.")
            return

        if not self.calibration_file:
            print("WARNING: OAK undistortion enabled but no calibration file was set.")
            self.undistort_oak_image = False
            return

        try:
            with np.load(self.calibration_file) as calibration:
                K = np.array(calibration["camera_matrix"], dtype=np.float64)
                D = np.array(calibration["dist_coeffs"], dtype=np.float64)

                calibration_size = (
                    int(calibration["image_width"].item()),
                    int(calibration["image_height"].item()),
                )

            output_size = (self.image_w, self.image_h)

            if calibration_size != output_size:
                raise RuntimeError(
                    f"Calibration size {calibration_size} does not match "
                    f"camera size {output_size}"
                )

            new_K, _ = cv2.getOptimalNewCameraMatrix(
                K,
                D,
                output_size,
                self.oak_undistort_alpha,
                output_size,
            )

            self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
                K,
                D,
                None,
                new_K,
                output_size,
                cv2.CV_16SC2,
            )

            print(
                f"OAK undistortion enabled from {self.calibration_file} "
                f"(alpha={self.oak_undistort_alpha}, size={self.image_w}x{self.image_h})."
            )

        except Exception as e:
            print(f"WARNING: OAK undistortion setup failed: {e}")
            self.undistort_oak_image = False
            self.undistort_map1 = None
            self.undistort_map2 = None

    def _packet_to_rgb_frame(self, packet):
        """
        Convert the DepthAI packet into the RGB image DonkeyCar expects.
        """
        frame = packet.getCvFrame()

        # Keep your original conversion behavior.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if self.undistort_oak_image and self.undistort_map1 is not None:
            frame = cv2.remap(
                frame,
                self.undistort_map1,
                self.undistort_map2,
                interpolation=cv2.INTER_LINEAR,
            )

        return frame

    def update(self):
        """
        DonkeyCar calls this in a background thread when threaded=True.
        """
        while self.running:
            packet = self.queue.tryGet()

            if packet is not None:
                self.frame = self._packet_to_rgb_frame(packet)

            time.sleep(1.0 / max(self.fps, 1))

    def run_threaded(self):
        """
        DonkeyCar calls this from the vehicle loop to get the latest frame.
        """
        return self.frame

    def run(self):
        """
        Fallback if the part is ever used with threaded=False.
        """
        packet = self.queue.tryGet()

        if packet is not None:
            self.frame = self._packet_to_rgb_frame(packet)

        return self.frame

    def shutdown(self):
        self.running = False
        time.sleep(0.1)

        if hasattr(self, "device"):
            self.device.close()
