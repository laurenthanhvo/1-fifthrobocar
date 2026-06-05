import time

import cv2
import numpy as np
import depthai as dai


# ============================================================
# OAK camera image settings
# ============================================================

UNDISTORT_OAK_IMAGE = True
OAK_CALIBRATION_FILE = "camera_calibration.npz"

# 0.0 = crop more, fewer black edges
# 1.0 = preserve more field of view, may show black edges
OAK_UNDISTORT_ALPHA = 0.0

# Keep this True to preserve your previous working camera view.
PREVIEW_KEEP_ASPECT_RATIO = True


class OakCamera:
    """
    Minimal DepthAI/OAK camera part for DonkeyCar.

    Outputs:
        cam/image_array          RGB image, shape (IMAGE_H, IMAGE_W, 3)
        cam/depth_array          depth map in millimeters, shape (IMAGE_H, IMAGE_W)
        cam/depth_heatmap_array  RGB heatmap, shape (IMAGE_H, IMAGE_W, 3)
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
        self.depth_mm = np.zeros((image_h, image_w), dtype=np.uint16)
        self.depth_heatmap = np.zeros((image_h, image_w, 3), dtype=np.uint8)

        self.undistort_map1 = None
        self.undistort_map2 = None

        self.undistort_oak_image = undistort
        self.calibration_file = calibration_file
        self.oak_undistort_alpha = float(np.clip(undistort_alpha, 0.0, 1.0))

        pipeline = dai.Pipeline()

        # ------------------------------------------------------------
        # Camera nodes
        # ------------------------------------------------------------

        cam = pipeline.createColorCamera()
        mono_left = pipeline.createMonoCamera()
        mono_right = pipeline.createMonoCamera()
        stereo = pipeline.createStereoDepth()

        xout_rgb = pipeline.createXLinkOut()
        xout_depth = pipeline.createXLinkOut()

        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        rgb_socket = self._get_rgb_socket()
        left_socket = self._get_left_socket()
        right_socket = self._get_right_socket()

        # ------------------------------------------------------------
        # RGB camera
        # ------------------------------------------------------------

        try:
            cam.setBoardSocket(rgb_socket)
        except Exception:
            pass

        # OV9782 requires 800_P or 720_P, so set 800_P explicitly.
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

        cam.setPreviewSize(image_w, image_h)
        cam.setPreviewKeepAspectRatio(PREVIEW_KEEP_ASPECT_RATIO)

        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam.setInterleaved(False)
        cam.setFps(fps)

        # ------------------------------------------------------------
        # Stereo mono cameras
        # ------------------------------------------------------------

        mono_left.setBoardSocket(left_socket)
        mono_right.setBoardSocket(right_socket)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        mono_left.setFps(fps)
        mono_right.setFps(fps)

        # ------------------------------------------------------------
        # Stereo depth
        # ------------------------------------------------------------

        try:
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        except Exception:
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)

        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        # Align depth to RGB camera when supported.
        try:
            stereo.setDepthAlign(rgb_socket)

            # DepthAI requires custom depth width to be multiple of 16.
            if image_w % 16 == 0:
                stereo.setOutputSize(image_w, image_h)
        except Exception as e:
            print(f"WARNING: depth alignment/output-size setup skipped: {e}")

        # ------------------------------------------------------------
        # Links
        # ------------------------------------------------------------

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        cam.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        self.device = dai.Device(pipeline)

        self.rgb_queue = self.device.getOutputQueue(
            name="rgb",
            maxSize=1,
            blocking=False,
        )

        self.depth_queue = self.device.getOutputQueue(
            name="depth",
            maxSize=1,
            blocking=False,
        )

        self._setup_undistortion()

    def _get_rgb_socket(self):
        if hasattr(dai.CameraBoardSocket, "CAM_A"):
            return dai.CameraBoardSocket.CAM_A
        return dai.CameraBoardSocket.RGB

    def _get_left_socket(self):
        if hasattr(dai.CameraBoardSocket, "CAM_B"):
            return dai.CameraBoardSocket.CAM_B
        return dai.CameraBoardSocket.LEFT

    def _get_right_socket(self):
        if hasattr(dai.CameraBoardSocket, "CAM_C"):
            return dai.CameraBoardSocket.CAM_C
        return dai.CameraBoardSocket.RIGHT

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

    def _make_depth_heatmap(self, depth_mm, min_mm=150, max_mm=2500):
        """
        Convert depth map to RGB heatmap.
        Closer = warmer/brighter.
        """
        depth = depth_mm.copy()

        # Invalid depth is usually 0. Treat it as far away visually.
        depth[depth == 0] = max_mm

        depth = np.clip(depth, min_mm, max_mm)

        depth_norm = ((max_mm - depth) / float(max_mm - min_mm) * 255.0)
        depth_norm = np.clip(depth_norm, 0, 255).astype(np.uint8)

        heatmap_bgr = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        heatmap_rgb = cv2.cvtColor(heatmap_bgr, cv2.COLOR_BGR2RGB)

        return heatmap_rgb

    def _packet_to_rgb_frame(self, packet):
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

    def _packet_to_depth_frame(self, packet):
        depth = packet.getFrame().astype(np.uint16)

        if depth.shape[:2] != (self.image_h, self.image_w):
            depth = cv2.resize(
                depth,
                (self.image_w, self.image_h),
                interpolation=cv2.INTER_NEAREST,
            )

        if self.undistort_oak_image and self.undistort_map1 is not None:
            depth = cv2.remap(
                depth,
                self.undistort_map1,
                self.undistort_map2,
                interpolation=cv2.INTER_NEAREST,
            )

        heatmap = self._make_depth_heatmap(depth)

        return depth, heatmap

    def update(self):
        """
        DonkeyCar calls this in a background thread when threaded=True.
        """
        while self.running:
            rgb_packet = self.rgb_queue.tryGet()
            depth_packet = self.depth_queue.tryGet()

            if rgb_packet is not None:
                self.frame = self._packet_to_rgb_frame(rgb_packet)

            if depth_packet is not None:
                self.depth_mm, self.depth_heatmap = self._packet_to_depth_frame(depth_packet)

            time.sleep(1.0 / max(self.fps, 1))

    def run_threaded(self):
        """
        Return RGB, depth map, and depth heatmap.
        """
        return self.frame, self.depth_mm, self.depth_heatmap

    def run(self):
        rgb_packet = self.rgb_queue.tryGet()
        depth_packet = self.depth_queue.tryGet()

        if rgb_packet is not None:
            self.frame = self._packet_to_rgb_frame(rgb_packet)

        if depth_packet is not None:
            self.depth_mm, self.depth_heatmap = self._packet_to_depth_frame(depth_packet)

        return self.frame, self.depth_mm, self.depth_heatmap

    def shutdown(self):
        self.running = False
        time.sleep(0.1)

        if hasattr(self, "device"):
            self.device.close()



# # oak_camera.py
# import time

# import cv2
# import numpy as np
# import depthai as dai


# # ============================================================
# # OAK camera image settings
# # ============================================================

# # Enable this to correct the curved / fisheye-looking image.
# UNDISTORT_OAK_IMAGE = True
# OAK_CALIBRATION_FILE = "camera_calibration.npz"

# # 0.0 = crop more, fewer black edges
# # 1.0 = preserve more field of view, may show black edges
# # Since you want a wider view, start high.
# OAK_UNDISTORT_ALPHA = 0.0

# # False usually fills the requested preview size better.
# # True preserves camera aspect ratio but can crop/pad depending on size.
# PREVIEW_KEEP_ASPECT_RATIO = True


# class OakCamera:
#     """
#     Minimal DepthAI/OAK camera part for DonkeyCar.

#     DonkeyCar expects threaded camera parts to provide:
#         update()
#         run_threaded()
#         shutdown()

#     Output:
#         cam/image_array as an RGB numpy array with shape (IMAGE_H, IMAGE_W, 3)
#     """

#     # def __init__(self, image_w=160, image_h=120, fps=20):
#     def __init__(self, image_w=160, image_h=120, fps=20, undistort=False, calibration_file=None, undistort_alpha=0.0):
#         self.image_w = image_w
#         self.image_h = image_h
#         self.fps = fps
#         self.running = True

#         self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)

#         # Old version with distortion
#         # self.undistort_oak_image = UNDISTORT_OAK_IMAGE
#         # self.oak_undistort_alpha = OAK_UNDISTORT_ALPHA
#         self.undistort_map1 = None
#         self.undistort_map2 = None

#         self.undistort_oak_image = undistort
#         self.calibration_file = calibration_file
#         self.oak_undistort_alpha = float(np.clip(undistort_alpha, 0.0, 1.0))
   

#         pipeline = dai.Pipeline()

#         cam = pipeline.createColorCamera()

#         # OV9782 requires 800_P or 720_P, so set 800_P explicitly.
#         cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

#         # DonkeyCar expects image arrays at the configured size.
#         cam.setPreviewSize(image_w, image_h)
#         cam.setPreviewKeepAspectRatio(PREVIEW_KEEP_ASPECT_RATIO)

#         # Request RGB output from DepthAI.
#         cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
#         cam.setInterleaved(False)
#         cam.setFps(fps)

#         xout = pipeline.createXLinkOut()
#         xout.setStreamName("rgb")
#         cam.preview.link(xout.input)

#         self.device = dai.Device(pipeline)
#         self.queue = self.device.getOutputQueue(
#             name="rgb",
#             maxSize=1,
#             blocking=False,
#         )

#         self._setup_undistortion()

#     def _get_rgb_socket(self):
#         """
#         DepthAI versions differ in naming:
#         - older versions use RGB
#         - newer versions use CAM_A
#         """
#         try:
#             return dai.CameraBoardSocket.RGB
#         except AttributeError:
#             return dai.CameraBoardSocket.CAM_A

#     # # Old version with distortion
#     # def _setup_undistortion(self):
#     #     """
#     #     Read the OAK calibration from the device and create undistortion maps.
#     #     """
#     #     if not self.undistort_oak_image:
#     #         print("OAK undistortion disabled.")
#     #         return

#     #     try:
#     #         rgb_socket = self._get_rgb_socket()
#     #         calib = self.device.readCalibration()

#     #         K = np.array(
#     #             calib.getCameraIntrinsics(
#     #                 rgb_socket,
#     #                 self.image_w,
#     #                 self.image_h,
#     #             ),
#     #             dtype=np.float32,
#     #         )

#     #         D = np.array(
#     #             calib.getDistortionCoefficients(rgb_socket),
#     #             dtype=np.float32,
#     #         )

#     #         # Use the first 5 standard OpenCV distortion coefficients:
#     #         # k1, k2, p1, p2, k3
#     #         D = D[:5]

#     #         new_K, _ = cv2.getOptimalNewCameraMatrix(
#     #             K,
#     #             D,
#     #             (self.image_w, self.image_h),
#     #             self.oak_undistort_alpha,
#     #             (self.image_w, self.image_h),
#     #         )

#     #         self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
#     #             K,
#     #             D,
#     #             None,
#     #             new_K,
#     #             (self.image_w, self.image_h),
#     #             cv2.CV_16SC2,
#     #         )

#     #         print(
#     #             f"OAK undistortion enabled "
#     #             f"(alpha={self.oak_undistort_alpha}, size={self.image_w}x{self.image_h})."
#     #         )

#     #     except Exception as e:
#     #         print(f"WARNING: OAK undistortion setup failed: {e}")
#     #         self.undistort_oak_image = False
#     #         self.undistort_map1 = None
#     #         self.undistort_map2 = None

#     def _setup_undistortion(self):
#         """
#         Load checkerboard calibration and create undistortion maps.
#         """
#         if not self.undistort_oak_image:
#             print("OAK undistortion disabled.")
#             return

#         if not self.calibration_file:
#             print("WARNING: OAK undistortion enabled but no calibration file was set.")
#             self.undistort_oak_image = False
#             return

#         try:
#             with np.load(self.calibration_file) as calibration:
#                 K = np.array(calibration["camera_matrix"], dtype=np.float64)
#                 D = np.array(calibration["dist_coeffs"], dtype=np.float64)

#                 calibration_size = (
#                     int(calibration["image_width"].item()),
#                     int(calibration["image_height"].item()),
#                 )

#             output_size = (self.image_w, self.image_h)

#             if calibration_size != output_size:
#                 raise RuntimeError(
#                     f"Calibration size {calibration_size} does not match "
#                     f"camera size {output_size}"
#                 )

#             new_K, _ = cv2.getOptimalNewCameraMatrix(
#                 K,
#                 D,
#                 output_size,
#                 self.oak_undistort_alpha,
#                 output_size,
#             )

#             self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
#                 K,
#                 D,
#                 None,
#                 new_K,
#                 output_size,
#                 cv2.CV_16SC2,
#             )

#             print(
#                 f"OAK undistortion enabled from {self.calibration_file} "
#                 f"(alpha={self.oak_undistort_alpha}, size={self.image_w}x{self.image_h})."
#             )

#         except Exception as e:
#             print(f"WARNING: OAK undistortion setup failed: {e}")
#             self.undistort_oak_image = False
#             self.undistort_map1 = None
#             self.undistort_map2 = None

#     def _packet_to_rgb_frame(self, packet):
#         """
#         Convert the DepthAI packet into the RGB image DonkeyCar expects.
#         """
#         frame = packet.getCvFrame()

#         # DepthAI/OpenCV often returns BGR-like frames here, so keep your original conversion.
#         frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         if self.undistort_oak_image and self.undistort_map1 is not None:
#             frame = cv2.remap(
#                 frame,
#                 self.undistort_map1,
#                 self.undistort_map2,
#                 interpolation=cv2.INTER_LINEAR,
#             )

#         return frame

#     def update(self):
#         """
#         DonkeyCar calls this in a background thread when threaded=True.
#         """
#         while self.running:
#             packet = self.queue.tryGet()

#             if packet is not None:
#                 self.frame = self._packet_to_rgb_frame(packet)

#             time.sleep(1.0 / max(self.fps, 1))

#     def run_threaded(self):
#         """
#         DonkeyCar calls this from the vehicle loop to get the latest frame.
#         """
#         return self.frame

#     def run(self):
#         """
#         Fallback if the part is ever used with threaded=False.
#         """
#         packet = self.queue.tryGet()

#         if packet is not None:
#             self.frame = self._packet_to_rgb_frame(packet)

#         return self.frame

#     def shutdown(self):
#         self.running = False
#         time.sleep(0.1)

#         if hasattr(self, "device"):
#             self.device.close()

# # import time

# # import cv2
# # import numpy as np
# # import depthai as dai


# # class OakCamera:
# #     """
# #     Minimal DepthAI/OAK camera part for DonkeyCar.

# #     DonkeyCar expects threaded camera parts to provide:
# #         update()
# #         run_threaded()
# #         shutdown()

# #     Output:
# #         cam/image_array as an RGB numpy array with shape (IMAGE_H, IMAGE_W, 3)
# #     """

# #     def __init__(self, image_w=160, image_h=120, fps=20):
# #         self.image_w = image_w
# #         self.image_h = image_h
# #         self.fps = fps
# #         self.running = True

# #         self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)

# #         pipeline = dai.Pipeline()

# #         cam = pipeline.createColorCamera()

# #         # OV9782 requires 800_P or 720_P, so set 800_P explicitly.
# #         cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

# #         # DonkeyCar expects small image arrays like 160x120.
# #         cam.setPreviewSize(image_w, image_h)
# #         cam.setPreviewKeepAspectRatio(True)

# #         # DepthAI getCvFrame often behaves like an OpenCV frame.
# #         # We convert BGR to RGB before handing the image to DonkeyCar.
# #         cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
# #         cam.setInterleaved(False)
# #         cam.setFps(fps)

# #         xout = pipeline.createXLinkOut()
# #         xout.setStreamName("rgb")
# #         cam.preview.link(xout.input)

# #         self.device = dai.Device(pipeline)
# #         self.queue = self.device.getOutputQueue(
# #             name="rgb",
# #             maxSize=1,
# #             blocking=False,
# #         )

# #     def _packet_to_rgb_frame(self, packet):
# #         """
# #         Convert the DepthAI packet into the RGB image DonkeyCar expects.
# #         """
# #         frame = packet.getCvFrame()

# #         # If colors look wrong in the web UI, this is the important conversion.
# #         frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

# #         return frame

# #     def update(self):
# #         """
# #         DonkeyCar calls this in a background thread when threaded=True.
# #         """
# #         while self.running:
# #             packet = self.queue.tryGet()

# #             if packet is not None:
# #                 self.frame = self._packet_to_rgb_frame(packet)

# #             time.sleep(1.0 / max(self.fps, 1))

# #     def run_threaded(self):
# #         """
# #         DonkeyCar calls this from the vehicle loop to get the latest frame.
# #         """
# #         return self.frame

# #     def run(self):
# #         """
# #         Fallback if the part is ever used with threaded=False.
# #         """
# #         packet = self.queue.tryGet()

# #         if packet is not None:
# #             self.frame = self._packet_to_rgb_frame(packet)

# #         return self.frame

# #     def shutdown(self):
# #         self.running = False
# #         time.sleep(0.1)

# #         if hasattr(self, "device"):
# #             self.device.close()









# ###############################################################
# # oak_camera_russel.py
# ###############################################################

# # import time
# # import logging

# # import cv2
# # import numpy as np
# # import depthai as dai

# # logger = logging.getLogger(__name__)


# # class OakCamera:
# #     """
# #     Minimal DepthAI/OAK camera part for DonkeyCar.

# #     DonkeyCar expects threaded camera parts to provide:
# #         update()
# #         run_threaded()
# #         shutdown()

# #     Output:
# #         cam/image_array as an RGB numpy array with shape (IMAGE_H, IMAGE_W, 3)
# #     """

# #     def __init__(
# #         self,
# #         image_w=160,
# #         image_h=120,
# #         fps=20,
# #         undistort=False,
# #         undistort_model="auto",
# #         undistort_alpha=0.0,
# #     ):
# #         self.image_w = image_w
# #         self.image_h = image_h
# #         self.fps = fps
# #         self.running = True
# #         self.undistort = undistort
# #         self.undistort_model = undistort_model
# #         self.undistort_alpha = float(np.clip(undistort_alpha, 0.0, 1.0))
# #         self.calib_data = None
# #         self.map_x = None
# #         self.map_y = None

# #         self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)

# #         pipeline = dai.Pipeline()

# #         cam = pipeline.createColorCamera()
# #         self.camera_socket = self._get_color_camera_socket()
# #         cam.setBoardSocket(self.camera_socket)

# #         # OV9782 requires 800_P or 720_P, so set 800_P explicitly.
# #         cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

# #         # DonkeyCar expects small image arrays like 160x120.
# #         cam.setPreviewSize(image_w, image_h)
# #         cam.setPreviewKeepAspectRatio(True)

# #         # DepthAI getCvFrame often behaves like an OpenCV frame.
# #         # We convert BGR to RGB before handing the image to DonkeyCar.
# #         cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
# #         cam.setInterleaved(False)
# #         cam.setFps(fps)

# #         xout = pipeline.createXLinkOut()
# #         xout.setStreamName("rgb")
# #         cam.preview.link(xout.input)

# #         self.device = dai.Device(pipeline)
# #         if self.undistort:
# #             self._init_undistortion()

# #         self.queue = self.device.getOutputQueue(
# #             name="rgb",
# #             maxSize=1,
# #             blocking=False,
# #         )

# #     @staticmethod
# #     def _get_color_camera_socket():
# #         """
# #         DepthAI renamed the RGB socket to CAM_A in newer releases.
# #         """
# #         if hasattr(dai.CameraBoardSocket, "CAM_A"):
# #             return dai.CameraBoardSocket.CAM_A

# #         return dai.CameraBoardSocket.RGB

# #     def _init_undistortion(self):
# #         """
# #         Read the OAK factory calibration and prepare remap tables once.
# #         """
# #         try:
# #             self.calib_data = self.device.readCalibration()
# #             self._build_undistortion_maps(self.image_w, self.image_h)
# #             logger.info("OAK camera undistortion enabled")
# #         except Exception:
# #             logger.exception("Could not initialize OAK camera undistortion")
# #             self.undistort = False

# #     def _build_undistortion_maps(self, width, height):
# #         """
# #         Build OpenCV remap tables for the current output frame size.
# #         """
# #         camera_matrix = np.array(
# #             self.calib_data.getCameraIntrinsics(
# #                 self.camera_socket,
# #                 width,
# #                 height,
# #             ),
# #             dtype=np.float64,
# #         )
# #         dist_coeffs = np.array(
# #             self.calib_data.getDistortionCoefficients(self.camera_socket),
# #             dtype=np.float64,
# #         ).reshape(-1, 1)

# #         if dist_coeffs.size == 0:
# #             raise RuntimeError("OAK calibration did not provide distortion coefficients")

# #         model = str(self.undistort_model).lower()
# #         if model == "auto":
# #             model = "fisheye" if dist_coeffs.size == 4 else "perspective"

# #         image_size = (width, height)
# #         identity = np.eye(3)

# #         if model == "fisheye":
# #             fisheye_coeffs = dist_coeffs[:4]
# #             new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
# #                 camera_matrix,
# #                 fisheye_coeffs,
# #                 image_size,
# #                 identity,
# #                 balance=self.undistort_alpha,
# #             )
# #             self.map_x, self.map_y = cv2.fisheye.initUndistortRectifyMap(
# #                 camera_matrix,
# #                 fisheye_coeffs,
# #                 identity,
# #                 new_camera_matrix,
# #                 image_size,
# #                 cv2.CV_16SC2,
# #             )
# #         else:
# #             new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
# #                 camera_matrix,
# #                 dist_coeffs,
# #                 image_size,
# #                 self.undistort_alpha,
# #                 image_size,
# #             )
# #             self.map_x, self.map_y = cv2.initUndistortRectifyMap(
# #                 camera_matrix,
# #                 dist_coeffs,
# #                 None,
# #                 new_camera_matrix,
# #                 image_size,
# #                 cv2.CV_16SC2,
# #             )

# #         self.image_w = width
# #         self.image_h = height

# #     def _packet_to_rgb_frame(self, packet):
# #         """
# #         Convert the DepthAI packet into the RGB image DonkeyCar expects.
# #         """
# #         frame = packet.getCvFrame()

# #         if self.undistort and self.map_x is not None and self.map_y is not None:
# #             frame_h, frame_w = frame.shape[:2]

# #             if frame_w != self.image_w or frame_h != self.image_h:
# #                 self._build_undistortion_maps(frame_w, frame_h)

# #             frame = cv2.remap(frame, self.map_x, self.map_y, cv2.INTER_LINEAR)

# #         # This fixed the weird color issue in the Donkey Monitor.
# #         frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

# #         return frame

# #     def update(self):
# #         """
# #         DonkeyCar calls this in a background thread when threaded=True.
# #         """
# #         while self.running:
# #             packet = self.queue.tryGet()

# #             if packet is not None:
# #                 self.frame = self._packet_to_rgb_frame(packet)

# #             time.sleep(1.0 / max(self.fps, 1))

# #     def run_threaded(self):
# #         """
# #         DonkeyCar calls this from the vehicle loop to get the latest frame.
# #         """
# #         return self.frame

# #     def run(self):
# #         """
# #         Fallback if the part is ever used with threaded=False.
# #         """
# #         packet = self.queue.tryGet()

# #         if packet is not None:
# #             self.frame = self._packet_to_rgb_frame(packet)

# #         return self.frame

# #     def shutdown(self):
# #         self.running = False
# #         time.sleep(0.1)

# #         if hasattr(self, "device"):
# #             self.device.close()


