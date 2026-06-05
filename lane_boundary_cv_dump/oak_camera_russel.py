import time
import logging

import cv2
import numpy as np
import depthai as dai

logger = logging.getLogger(__name__)


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
        undistort_model="auto",
        undistort_alpha=0.0,
    ):
        self.image_w = image_w
        self.image_h = image_h
        self.fps = fps
        self.running = True
        self.undistort = undistort
        self.undistort_model = undistort_model
        self.undistort_alpha = float(np.clip(undistort_alpha, 0.0, 1.0))
        self.calib_data = None
        self.map_x = None
        self.map_y = None

        self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)

        pipeline = dai.Pipeline()

        cam = pipeline.createColorCamera()
        self.camera_socket = self._get_color_camera_socket()
        cam.setBoardSocket(self.camera_socket)

        # OV9782 requires 800_P or 720_P, so set 800_P explicitly.
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

        # DonkeyCar expects small image arrays like 160x120.
        cam.setPreviewSize(image_w, image_h)
        cam.setPreviewKeepAspectRatio(True)

        # DepthAI getCvFrame often behaves like an OpenCV frame.
        # We convert BGR to RGB before handing the image to DonkeyCar.
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam.setInterleaved(False)
        cam.setFps(fps)

        xout = pipeline.createXLinkOut()
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)

        self.device = dai.Device(pipeline)
        if self.undistort:
            self._init_undistortion()

        self.queue = self.device.getOutputQueue(
            name="rgb",
            maxSize=1,
            blocking=False,
        )

    @staticmethod
    def _get_color_camera_socket():
        """
        DepthAI renamed the RGB socket to CAM_A in newer releases.
        """
        if hasattr(dai.CameraBoardSocket, "CAM_A"):
            return dai.CameraBoardSocket.CAM_A

        return dai.CameraBoardSocket.RGB

    def _init_undistortion(self):
        """
        Read the OAK factory calibration and prepare remap tables once.
        """
        try:
            self.calib_data = self.device.readCalibration()
            self._build_undistortion_maps(self.image_w, self.image_h)
            logger.info("OAK camera undistortion enabled")
        except Exception:
            logger.exception("Could not initialize OAK camera undistortion")
            self.undistort = False

    def _build_undistortion_maps(self, width, height):
        """
        Build OpenCV remap tables for the current output frame size.
        """
        camera_matrix = np.array(
            self.calib_data.getCameraIntrinsics(
                self.camera_socket,
                width,
                height,
            ),
            dtype=np.float64,
        )
        dist_coeffs = np.array(
            self.calib_data.getDistortionCoefficients(self.camera_socket),
            dtype=np.float64,
        ).reshape(-1, 1)

        if dist_coeffs.size == 0:
            raise RuntimeError("OAK calibration did not provide distortion coefficients")

        model = str(self.undistort_model).lower()
        if model == "auto":
            model = "fisheye" if dist_coeffs.size == 4 else "perspective"

        image_size = (width, height)
        identity = np.eye(3)

        if model == "fisheye":
            fisheye_coeffs = dist_coeffs[:4]
            new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                camera_matrix,
                fisheye_coeffs,
                image_size,
                identity,
                balance=self.undistort_alpha,
            )
            self.map_x, self.map_y = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix,
                fisheye_coeffs,
                identity,
                new_camera_matrix,
                image_size,
                cv2.CV_16SC2,
            )
        else:
            new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix,
                dist_coeffs,
                image_size,
                self.undistort_alpha,
                image_size,
            )
            self.map_x, self.map_y = cv2.initUndistortRectifyMap(
                camera_matrix,
                dist_coeffs,
                None,
                new_camera_matrix,
                image_size,
                cv2.CV_16SC2,
            )

        self.image_w = width
        self.image_h = height

    def _packet_to_rgb_frame(self, packet):
        """
        Convert the DepthAI packet into the RGB image DonkeyCar expects.
        """
        frame = packet.getCvFrame()

        if self.undistort and self.map_x is not None and self.map_y is not None:
            frame_h, frame_w = frame.shape[:2]

            if frame_w != self.image_w or frame_h != self.image_h:
                self._build_undistortion_maps(frame_w, frame_h)

            frame = cv2.remap(frame, self.map_x, self.map_y, cv2.INTER_LINEAR)

        # This fixed the weird color issue in the Donkey Monitor.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

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
