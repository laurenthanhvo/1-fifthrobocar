"""
obstacle_avoidance_yolo.py
Donkeycar obstacle detector using YOLOv8 + Livox MID-360 lidar fusion.

Replaces the RGB-contour ObstacleDetector with a proper detection pipeline:
  1. YOLO detects objects and gives bounding boxes on the raw camera image
  2. Lidar points are projected onto the camera plane using extrinsic calibration
  3. Minimum lidar distance inside each bounding box = object distance
  4. Only boxes inside the "driving path zone" trigger avoidance

Requirements:
    pip install ultralytics

myconfig.py values needed:
    YOLO_MODEL_PATH       = "yolov8n.pt"          # or path to custom model
    YOLO_CONF_THRESHOLD   = 0.45                  # detection confidence cutoff
    YOLO_CLASSES          = None                  # None = all, or [0,1,2] = specific COCO classes
    LIDAR_EXTRINSIC_YAML  = "/path/to/extrinsic.yaml"
    OAK_CALIBRATION_FILE  = "camera_calibration.npz"   # already set
    PATH_ZONE_X           = (0.25, 0.75)          # fraction of image width = driving corridor
    PATH_ZONE_Y           = (0.20, 0.90)          # fraction of image height
    OBSTACLE_DEPTH_THRESH = 500                   # hard stop (mm)
    OBSTACLE_WARN_THRESH  = 900                   # slow down (mm)
    OBSTACLE_OVERLAY      = True
    OBSTACLE_CONFIRM_FRAMES = 3
    OBSTACLE_CLEAR_FRAMES   = 5
"""

import os
import cv2
import yaml
import numpy as np
import logging

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Calibration loaders
# ---------------------------------------------------------------------------

def load_extrinsic(yaml_path: str) -> np.ndarray:
    """Load 4x4 lidar-to-camera extrinsic matrix from YAML."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"Extrinsic calibration not found: {yaml_path}")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    T = np.array(data['extrinsic_matrix'], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix must be 4x4")
    return T


def load_camera_intrinsics(npz_path: str):
    """Load camera matrix and distortion coefficients from NPZ calibration."""
    with np.load(npz_path) as cal:
        K = np.array(cal['camera_matrix'], dtype=np.float64)
        D = np.array(cal['dist_coeffs'],   dtype=np.float64).flatten()
    return K, D


# ---------------------------------------------------------------------------
# Main detector
# ---------------------------------------------------------------------------

class YoloLidarObstacleDetector:
    """
    Donkeycar part — detects obstacles using YOLOv8 bounding boxes fused
    with Livox MID-360 lidar depth data.

    Vehicle memory inputs:
        cam/image_array      — raw RGB image from OAK camera
        lidar/raw_points     — (N,3) float32 array in meters (lidar frame)
                               from MID360LidarPart.  None if lidar not ready.
        cv/image_array       — bird's-eye overlay from lane follower (for display)

    Vehicle memory outputs:
        obstacle/detected    — bool
        obstacle/distance    — float, mm (0 = no obstacle)
        obstacle/position    — 'left' | 'center' | 'right' | 'none'
        cv/image_array       — annotated image (drawn on bird's-eye overlay)
    """

    def __init__(self, cfg):
        # ── YOLO ──────────────────────────────────────────────────────────
        from ultralytics import YOLO as _YOLO

        model_path = getattr(cfg, 'YOLO_MODEL_PATH', 'yolov8n.pt')
        self.model      = _YOLO(model_path)
        self.conf       = getattr(cfg, 'YOLO_CONF_THRESHOLD', 0.45)
        self.classes    = getattr(cfg, 'YOLO_CLASSES', None)   # None = all classes

        # Run on GPU if available (Jetson AGX has CUDA)
        try:
            self.model.to('cuda')
            logger.info("YOLO running on CUDA")
        except Exception:
            self.model.to('cpu')
            logger.info("YOLO running on CPU")

        # ── Calibration ───────────────────────────────────────────────────
        extrinsic_yaml = getattr(cfg, 'LIDAR_EXTRINSIC_YAML', None)
        cal_file       = getattr(cfg, 'OAK_CALIBRATION_FILE',  None)

        self._projection_ready = False
        if extrinsic_yaml and cal_file and os.path.isfile(extrinsic_yaml) and os.path.isfile(cal_file):
            try:
                self.T_lidar_to_cam   = load_extrinsic(extrinsic_yaml)
                self.camera_matrix, self.dist_coeffs = load_camera_intrinsics(cal_file)
                self._projection_ready = True
                logger.info("Lidar-camera projection calibration loaded")
            except Exception as e:
                logger.warning(f"Calibration load failed, lidar depth unavailable: {e}")
        else:
            logger.warning("LIDAR_EXTRINSIC_YAML or OAK_CALIBRATION_FILE not set — "
                           "lidar depth fusion disabled, using YOLO box size as proxy")

        # ── Driving path zone ─────────────────────────────────────────────
        # Only trigger avoidance for boxes whose centre falls inside this zone.
        # Adjust in myconfig.py to match your track width in the camera view.
        self.path_x = getattr(cfg, 'PATH_ZONE_X', (0.25, 0.75))
        self.path_y = getattr(cfg, 'PATH_ZONE_Y', (0.20, 0.90))

        # ── Distance thresholds (convert mm → m for lidar comparisons) ────
        self.stop_dist_mm = getattr(cfg, 'OBSTACLE_DEPTH_THRESH', 500)
        self.warn_dist_mm = getattr(cfg, 'OBSTACLE_WARN_THRESH',  900)

        # ── Debounce ──────────────────────────────────────────────────────
        self._confirm = getattr(cfg, 'OBSTACLE_CONFIRM_FRAMES', 3)
        self._clear   = getattr(cfg, 'OBSTACLE_CLEAR_FRAMES',   5)
        self._yes_count = 0
        self._no_count  = 0
        self._latched   = False

        self.overlay = getattr(cfg, 'OBSTACLE_OVERLAY', True)

        # Internal state
        self.detected  = False
        self.distance  = 0.0
        self.position  = 'none'

    # ------------------------------------------------------------------
    # Lidar → camera projection
    # ------------------------------------------------------------------

    def _project_lidar_to_image(self, lidar_pts, img_h, img_w):
        """
        Project (N,3) lidar points (meters, lidar frame) onto the image plane.
        Returns (u, v, depth) arrays for points that land inside the image.
        depth is in METERS.
        """
        if lidar_pts is None or len(lidar_pts) == 0:
            return np.array([]), np.array([]), np.array([])

        n = len(lidar_pts)
        ones = np.ones((n, 1), dtype=np.float64)
        pts_h = np.hstack([lidar_pts.astype(np.float64), ones])   # (N,4)

        # Transform to camera frame
        pts_cam = (self.T_lidar_to_cam @ pts_h.T).T[:, :3]        # (N,3)

        # Keep only points in front of camera (positive Z)
        mask_front = pts_cam[:, 2] > 0.05
        pts_cam    = pts_cam[mask_front]
        if len(pts_cam) == 0:
            return np.array([]), np.array([]), np.array([])

        # Project using camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        z = pts_cam[:, 2]
        u = (fx * pts_cam[:, 0] / z + cx).astype(int)
        v = (fy * pts_cam[:, 1] / z + cy).astype(int)

        # Keep only points inside image
        mask_img = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        return u[mask_img], v[mask_img], z[mask_img]

    def _build_depth_image(self, u, v, depth, img_h, img_w):
        """
        Build a per-pixel minimum-depth map from projected lidar points.
        Values are in METERS; 0 = no reading.
        """
        depth_img = np.zeros((img_h, img_w), dtype=np.float32)
        for i in range(len(u)):
            ui, vi, di = int(u[i]), int(v[i]), float(depth[i])
            if depth_img[vi, ui] == 0 or di < depth_img[vi, ui]:
                depth_img[vi, ui] = di
        return depth_img

    # ------------------------------------------------------------------
    # Path zone check
    # ------------------------------------------------------------------

    def _in_path_zone(self, cx, cy, img_w, img_h):
        """Return True if box centre (cx, cy) is inside the driving corridor."""
        x_lo = int(self.path_x[0] * img_w)
        x_hi = int(self.path_x[1] * img_w)
        y_lo = int(self.path_y[0] * img_h)
        y_hi = int(self.path_y[1] * img_h)
        return x_lo <= cx <= x_hi and y_lo <= cy <= y_hi

    def _lateral_position(self, cx, img_w):
        """Map box centre x → 'left' | 'center' | 'right'."""
        third = img_w / 3.0
        if cx < third:
            return 'left'
        elif cx < 2 * third:
            return 'center'
        return 'right'

    # ------------------------------------------------------------------
    # Distance estimation
    # ------------------------------------------------------------------

    def _box_distance_lidar(self, x1, y1, x2, y2, depth_img):
        """
        Minimum lidar depth (meters) inside the bounding box.
        Returns 0.0 if no lidar points fall inside the box.
        """
        roi = depth_img[y1:y2, x1:x2]
        valid = roi[roi > 0]
        if valid.size == 0:
            return 0.0
        return float(np.min(valid))

    def _box_distance_size_proxy(self, x1, y1, x2, y2, img_h, img_w):
        """
        Fallback distance proxy when lidar is unavailable.
        Larger box area ≈ closer object.
        Maps box area fraction 0→stop_dist, 1→warn_dist in mm.
        Returns distance in mm.
        """
        area_frac = ((x2 - x1) * (y2 - y1)) / (img_h * img_w)
        area_frac = min(area_frac, 1.0)
        dist_mm   = self.warn_dist_mm * (1.0 - area_frac)
        return dist_mm

    # ------------------------------------------------------------------
    # Overlay drawing
    # ------------------------------------------------------------------

    def _draw_overlay(self, img, detections, detected, distance, position,
                      img_h, img_w):
        """
        Draw YOLO bounding boxes and obstacle status on img (which may be
        the bird's-eye overlay from the lane follower).
        detections: list of (x1,y1,x2,y2, label, dist_mm, in_path)
        """
        out = np.copy(img)

        # Draw path zone rectangle
        pz_x1 = int(self.path_x[0] * img_w)
        pz_x2 = int(self.path_x[1] * img_w)
        pz_y1 = int(self.path_y[0] * img_h)
        pz_y2 = int(self.path_y[1] * img_h)
        cv2.rectangle(out, (pz_x1, pz_y1), (pz_x2, pz_y2), (255, 165, 0), 1)

        # Draw each detection
        for (x1, y1, x2, y2, label, dist_mm, in_path) in detections:
            color = (0, 0, 255) if in_path else (180, 180, 180)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            dist_txt = f"{dist_mm/1000:.2f}m" if dist_mm > 0 else "?m"
            cv2.putText(out, f"{label} {dist_txt}", (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
            cv2.putText(out, f"{label} {dist_txt}", (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Status text
        lines = [
            f"OBSTACLE: {'YES' if detected else 'NO'}",
            f"DIST: {distance:.0f}mm" if distance > 0 else "DIST: N/A",
            f"POS: {position.upper()}",
        ]
        x_txt = img_w - 165
        y_txt = 16
        for line in lines:
            cv2.putText(out, line, (x_txt, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(out, line, (x_txt, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            y_txt += 16

        return out

    # ------------------------------------------------------------------
    # Donkeycar part interface
    # ------------------------------------------------------------------

    def run(self, cam_img, lidar_points=None, overlay_img=None):
        """
        cam_img       : raw RGB numpy array from OAK camera
        lidar_points  : (N,3) float32 array in meters (lidar frame), or None
        overlay_img   : bird's-eye annotated image from lane follower for display

        Returns: detected (bool), distance (float, mm), position (str), out_img
        """
        if cam_img is None:
            return False, 0.0, 'none', overlay_img

        img_h, img_w = cam_img.shape[:2]

        # ── Build lidar depth image if calibration is available ───────────
        depth_img = None
        if self._projection_ready and lidar_points is not None and len(lidar_points) > 0:
            u, v, depth = self._project_lidar_to_image(lidar_points, img_h, img_w)
            if len(u) > 0:
                depth_img = self._build_depth_image(u, v, depth, img_h, img_w)

        # ── Run YOLO ──────────────────────────────────────────────────────
        # Convert RGB → BGR for YOLO (trained on BGR/OpenCV images)
        bgr = cv2.cvtColor(cam_img, cv2.COLOR_RGB2BGR)
        results = self.model(bgr, conf=self.conf, classes=self.classes, verbose=False)

        # ── Process detections ────────────────────────────────────────────
        best_dist_mm = 0.0
        best_pos     = 'none'
        detections   = []   # for overlay drawing

        for result in results:
            for box in result.boxes:
                b    = box.xyxy[0].cpu().numpy()
                x1   = max(0, int(b[0]))
                y1   = max(0, int(b[1]))
                x2   = min(img_w, int(b[2]))
                y2   = min(img_h, int(b[3]))
                label = result.names[int(box.cls[0])]
                conf  = float(box.conf[0])

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                in_path = self._in_path_zone(cx, cy, img_w, img_h)

                # Get distance
                if depth_img is not None:
                    dist_m  = self._box_distance_lidar(x1, y1, x2, y2, depth_img)
                    dist_mm = dist_m * 1000.0 if dist_m > 0 else 0.0
                else:
                    dist_mm = self._box_distance_size_proxy(x1, y1, x2, y2, img_h, img_w)

                detections.append((x1, y1, x2, y2, label, dist_mm, in_path))

                # Only consider boxes inside the driving path
                if not in_path:
                    continue

                # Track the closest in-path obstacle
                if dist_mm > 0 and (best_dist_mm == 0 or dist_mm < best_dist_mm):
                    best_dist_mm = dist_mm
                    best_pos     = self._lateral_position(cx, img_w)

        # ── Classify: is this a STOP-zone obstacle? ───────────────────────
        raw_detected = (best_dist_mm > 0 and best_dist_mm < self.stop_dist_mm)

        # ── Debounce ──────────────────────────────────────────────────────
        if raw_detected:
            self._yes_count += 1
            self._no_count   = 0
            if self._yes_count >= self._confirm:
                self._latched = True
        else:
            self._no_count  += 1
            self._yes_count  = 0
            if self._no_count >= self._clear:
                self._latched = False

        detected = self._latched
        self.detected = detected
        self.distance = best_dist_mm
        self.position = best_pos if best_dist_mm > 0 else 'none'

        # ── Draw overlay ──────────────────────────────────────────────────
        if self.overlay and overlay_img is not None:
            out_img = self._draw_overlay(
                overlay_img, detections, detected, self.distance,
                self.position, overlay_img.shape[0], overlay_img.shape[1]
            )
        else:
            out_img = overlay_img

        return detected, self.distance, self.position, out_img
