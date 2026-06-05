import math
import time
import threading
import logging
from pathlib import Path

import cv2
import numpy as np


logger = logging.getLogger(__name__)


def angle_diff_deg(a, b):
    """
    Smallest signed angular difference between a and b in degrees.
    """
    return (a - b + 180.0) % 360.0 - 180.0


class LidarReader:
    """
    Background 2D LiDAR reader.

    This assumes an RPLidar-style device using the rplidar Python package.
    If your LiDAR library is different, only this class needs to change.
    """

    def __init__(self, cfg):
        self.enable = bool(getattr(cfg, "LIDAR_ENABLE", False))
        self.port = getattr(cfg, "LIDAR_PORT", "/dev/ttyUSB0")
        self.baudrate = int(getattr(cfg, "LIDAR_BAUDRATE", 115200))

        self.min_distance_m = float(getattr(cfg, "LIDAR_MIN_DISTANCE_M", 0.05))
        self.max_distance_m = float(getattr(cfg, "LIDAR_MAX_DISTANCE_M", 5.0))

        self.scan_lock = threading.Lock()
        self.latest_scan = []
        self.running = False
        self.thread = None
        self.lidar = None

        if self.enable:
            self.start()

    def start(self):
        try:
            from rplidar import RPLidar

            self.lidar = RPLidar(self.port, baudrate=self.baudrate)
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

            logger.info(f"LiDAR started on {self.port}")

        except Exception as e:
            logger.warning(f"LiDAR disabled. Could not start LiDAR on {self.port}: {e}")
            self.enable = False
            self.running = False
            self.lidar = None

    def _loop(self):
        while self.running:
            try:
                for scan in self.lidar.iter_scans(max_buf_meas=1000):
                    points = []

                    for quality, angle_deg, distance_mm in scan:
                        distance_m = float(distance_mm) / 1000.0

                        if distance_m < self.min_distance_m:
                            continue

                        if distance_m > self.max_distance_m:
                            continue

                        points.append((float(angle_deg) % 360.0, distance_m))

                    with self.scan_lock:
                        self.latest_scan = points

                    if not self.running:
                        break

            except Exception as e:
                logger.warning(f"LiDAR read error: {e}")
                time.sleep(0.5)

    def get_distance_at_angle(self, target_angle_deg, window_deg=5.0):
        """
        Return median distance around target angle.
        """
        if not self.enable:
            return None

        with self.scan_lock:
            scan = list(self.latest_scan)

        if not scan:
            return None

        distances = []

        for angle_deg, distance_m in scan:
            if abs(angle_diff_deg(angle_deg, target_angle_deg)) <= window_deg:
                distances.append(distance_m)

        if not distances:
            return None

        # Median is more stable than min.
        return float(np.median(np.array(distances, dtype=np.float32)))

    def shutdown(self):
        self.running = False

        try:
            if self.lidar is not None:
                self.lidar.stop()
                self.lidar.disconnect()
        except Exception:
            pass


class YoloLidarObstacleDetector:
    """
    DonkeyCar part.

    Inputs:
        cam/image_array: raw RGB camera image
        cv/image_array: lane follower debug image

    Outputs:
        obstacle/detected
        obstacle/distance
        obstacle/height
        obstacle/label
        cv/image_array
    """

    def __init__(self, cfg):
        self.cfg = cfg

        self.model_path = getattr(cfg, "YOLO_MODEL_PATH", "models/yolo11n.pt")
        self.conf = float(getattr(cfg, "YOLO_CONFIDENCE", 0.35))
        self.imgsz = int(getattr(cfg, "YOLO_IMAGE_SIZE", 640))
        self.device = getattr(cfg, "YOLO_DEVICE", 0)

        self.target_classes = set(getattr(cfg, "YOLO_TARGET_CLASSES", []))
        self.run_every_n = int(getattr(cfg, "YOLO_RUN_EVERY_N_FRAMES", 2))
        self.frame_count = 0

        self.stop_distance_m = float(getattr(cfg, "YOLO_STOP_DISTANCE_M", 0.75))
        self.min_height_m = float(getattr(cfg, "YOLO_MIN_HEIGHT_M", 0.05))

        self.draw_debug = bool(getattr(cfg, "YOLO_DRAW_DEBUG", True))

        self.fx_px = float(getattr(cfg, "CAMERA_FX_PX", 287.82))
        self.fy_px = float(getattr(cfg, "CAMERA_FY_PX", 287.82))
        self._load_camera_intrinsics(cfg)

        self.lidar_front_angle_deg = float(getattr(cfg, "LIDAR_FRONT_ANGLE_DEG", 0.0))
        self.lidar_yaw_offset_deg = float(getattr(cfg, "LIDAR_YAW_OFFSET_DEG", 0.0))
        self.lidar_angle_sign = float(getattr(cfg, "LIDAR_CAMERA_ANGLE_SIGN", 1.0))
        self.lidar_window_deg = float(getattr(cfg, "LIDAR_ANGLE_WINDOW_DEG", 5.0))

        self.lidar = LidarReader(cfg)

        self.last_result = {
            "detected": False,
            "distance_m": -1.0,
            "height_m": -1.0,
            "label": "",
            "boxes": [],
        }

        self._setup_birdseye(cfg)

        from ultralytics import YOLO

        model_file = Path(self.model_path)
        if not model_file.exists():
            logger.warning(
                f"YOLO model path {self.model_path} not found. "
                f"Ultralytics may try to download it."
            )

        self.model = YOLO(self.model_path)

        logger.info(f"YOLO loaded: {self.model_path}")

    def _load_camera_intrinsics(self, cfg):
        """
        Load fx/fy from camera_calibration.npz if available.
        """
        cal_file = getattr(cfg, "OAK_CALIBRATION_FILE", None)

        if not cal_file:
            return

        try:
            with np.load(cal_file) as data:
                K = np.array(data["camera_matrix"], dtype=np.float32)
                self.fx_px = float(K[0, 0])
                self.fy_px = float(K[1, 1])

            logger.info(f"Loaded camera intrinsics fx={self.fx_px:.2f}, fy={self.fy_px:.2f}")

        except Exception as e:
            logger.warning(f"Could not load camera intrinsics, using fallback fx/fy: {e}")

    def _setup_birdseye(self, cfg):
        self.use_birdseye = bool(getattr(cfg, "LANE_USE_BIRDSEYE", True))

        self.bird_src_bottom_left = getattr(cfg, "LANE_BIRD_SRC_BOTTOM_LEFT", (0.05, 0.98))
        self.bird_src_bottom_right = getattr(cfg, "LANE_BIRD_SRC_BOTTOM_RIGHT", (0.95, 0.98))
        self.bird_src_top_right = getattr(cfg, "LANE_BIRD_SRC_TOP_RIGHT", (0.68, 0.52))
        self.bird_src_top_left = getattr(cfg, "LANE_BIRD_SRC_TOP_LEFT", (0.32, 0.52))

        self.bird_dst_bottom_left = getattr(cfg, "LANE_BIRD_DST_BOTTOM_LEFT", (0.15, 1.00))
        self.bird_dst_bottom_right = getattr(cfg, "LANE_BIRD_DST_BOTTOM_RIGHT", (0.85, 1.00))
        self.bird_dst_top_right = getattr(cfg, "LANE_BIRD_DST_TOP_RIGHT", (0.85, 0.00))
        self.bird_dst_top_left = getattr(cfg, "LANE_BIRD_DST_TOP_LEFT", (0.15, 0.00))

    def _bird_matrix(self, w, h):
        def pt(pair):
            return [pair[0] * w, pair[1] * h]

        src = np.float32([
            pt(self.bird_src_bottom_left),
            pt(self.bird_src_bottom_right),
            pt(self.bird_src_top_right),
            pt(self.bird_src_top_left),
        ])

        dst = np.float32([
            pt(self.bird_dst_bottom_left),
            pt(self.bird_dst_bottom_right),
            pt(self.bird_dst_top_right),
            pt(self.bird_dst_top_left),
        ])

        return cv2.getPerspectiveTransform(src, dst)

    def _transform_points_to_bird(self, points, w, h):
        if not self.use_birdseye:
            return np.array(points, dtype=np.float32)

        M = self._bird_matrix(w, h)
        pts = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(pts, M).reshape(-1, 2)

        return warped

    def _camera_x_to_lidar_angle(self, x_px, image_w):
        """
        Convert horizontal image pixel to approximate LiDAR bearing.
        """
        dx = float(x_px) - float(image_w) / 2.0
        camera_angle_deg = math.degrees(math.atan2(dx, max(self.fx_px, 1.0)))

        lidar_angle = (
            self.lidar_front_angle_deg
            + self.lidar_yaw_offset_deg
            + self.lidar_angle_sign * camera_angle_deg
        ) % 360.0

        return lidar_angle

    def _estimate_size(self, bbox, distance_m):
        """
        Estimate physical width/height from bbox size and LiDAR distance.
        """
        x1, y1, x2, y2 = bbox
        bbox_w = max(1.0, float(x2 - x1))
        bbox_h = max(1.0, float(y2 - y1))

        width_m = bbox_w * distance_m / max(self.fx_px, 1.0)
        height_m = bbox_h * distance_m / max(self.fy_px, 1.0)

        return width_m, height_m

    def _lane_bounds_from_debug_image(self, debug_img, foot_y):
        """
        Estimate lane boundaries from the green lane overlay in cv/image_array.

        This keeps YOLO obstacle filtering consistent with your current lane
        visualization without modifying LaneCenterFollower outputs.
        """
        h, w = debug_img.shape[:2]
        y = int(max(0, min(h - 1, foot_y)))

        y0 = max(0, y - 8)
        y1 = min(h, y + 9)

        band = debug_img[y0:y1, :, :]

        # RGB green lane overlay: lane mask color is usually (0, 255, 0).
        green = (
            (band[:, :, 1] > 170)
            & (band[:, :, 0] < 100)
            & (band[:, :, 2] < 100)
        )

        col_counts = np.sum(green, axis=0)
        active = col_counts >= 2

        runs = []
        start = None

        for i, val in enumerate(active):
            if val and start is None:
                start = i
            elif not val and start is not None:
                end = i - 1
                if end - start >= 5:
                    runs.append((start, end, (start + end) // 2))
                start = None

        if start is not None:
            end = len(active) - 1
            if end - start >= 5:
                runs.append((start, end, (start + end) // 2))

        if len(runs) >= 2:
            left = runs[0][2]
            right = runs[-1][2]
            return float(left), float(right)

        # Fallback: center corridor when lane overlay is weak.
        return float(w * 0.25), float(w * 0.75)

    def _is_inside_lane(self, debug_img, bird_foot_x, bird_foot_y):
        h, w = debug_img.shape[:2]

        left, right = self._lane_bounds_from_debug_image(debug_img, bird_foot_y)

        margin = 10.0
        return (bird_foot_x >= left + margin) and (bird_foot_x <= right - margin)

    def _run_yolo(self, img):
        result = self.model.predict(
            source=img,
            imgsz=self.imgsz,
            conf=self.conf,
            device=self.device,
            verbose=False,
        )[0]

        names = result.names
        boxes = []

        if result.boxes is None:
            return boxes

        for b in result.boxes:
            cls_id = int(b.cls[0])
            label = str(names.get(cls_id, cls_id))

            if self.target_classes and label not in self.target_classes:
                continue

            conf = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].cpu().numpy().tolist()

            boxes.append({
                "label": label,
                "conf": conf,
                "bbox": (float(x1), float(y1), float(x2), float(y2)),
            })

        return boxes

    def run(self, cam_img, debug_img):
        if cam_img is None:
            return False, -1.0, -1.0, "", debug_img

        if debug_img is None:
            debug_img = np.copy(cam_img)

        self.frame_count += 1

        # Reuse previous result on skipped frames for speed.
        if self.frame_count % max(self.run_every_n, 1) != 0:
            r = self.last_result
            return r["detected"], r["distance_m"], r["height_m"], r["label"], debug_img

        h, w = cam_img.shape[:2]

        detections = self._run_yolo(cam_img)

        best = None
        debug_out = np.copy(debug_img)

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            cx = 0.5 * (x1 + x2)
            foot_x_raw = cx
            foot_y_raw = y2

            bird_points = self._transform_points_to_bird(
                [
                    (x1, y1),
                    (x2, y1),
                    (x2, y2),
                    (x1, y2),
                    (foot_x_raw, foot_y_raw),
                ],
                w,
                h,
            )

            bird_box_pts = bird_points[:4].astype(np.int32)
            bird_foot = bird_points[4]
            bird_foot_x = float(bird_foot[0])
            bird_foot_y = float(bird_foot[1])

            if not self._is_inside_lane(debug_out, bird_foot_x, bird_foot_y):
                continue

            lidar_angle = self._camera_x_to_lidar_angle(cx, w)
            distance_m = self.lidar.get_distance_at_angle(
                lidar_angle,
                self.lidar_window_deg,
            )

            if distance_m is None:
                distance_m = -1.0
                width_m = -1.0
                height_m = -1.0
            else:
                width_m, height_m = self._estimate_size(det["bbox"], distance_m)

            should_stop = (
                distance_m > 0
                and distance_m <= self.stop_distance_m
                and height_m >= self.min_height_m
            )

            score_distance = distance_m if distance_m > 0 else 999.0

            candidate = {
                "det": det,
                "distance_m": float(distance_m),
                "width_m": float(width_m),
                "height_m": float(height_m),
                "should_stop": bool(should_stop),
                "bird_box_pts": bird_box_pts,
                "bird_foot": (int(bird_foot_x), int(bird_foot_y)),
                "score_distance": score_distance,
            }

            if best is None or candidate["score_distance"] < best["score_distance"]:
                best = candidate

        detected = False
        distance_m = -1.0
        height_m = -1.0
        label = ""

        if best is not None:
            det = best["det"]
            label = det["label"]
            distance_m = best["distance_m"]
            height_m = best["height_m"]
            detected = best["should_stop"]

            if self.draw_debug:
                pts = best["bird_box_pts"].reshape((-1, 1, 2))

                color = (255, 0, 0) if detected else (255, 255, 0)
                cv2.polylines(debug_out, [pts], True, color, 3)

                fx, fy = best["bird_foot"]
                cv2.circle(debug_out, (fx, fy), 5, color, -1)

                if distance_m > 0:
                    text = f"{label} {distance_m:.2f}m h={height_m:.2f}m"
                else:
                    text = f"{label} dist=? h=?"

                if detected:
                    text = "YOLO STOP: " + text
                else:
                    text = "YOLO: " + text

                x_text = int(np.min(best["bird_box_pts"][:, 0]))
                y_text = int(max(25, np.min(best["bird_box_pts"][:, 1]) - 8))

                cv2.putText(
                    debug_out,
                    text,
                    (x_text, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        self.last_result = {
            "detected": bool(detected),
            "distance_m": float(distance_m),
            "height_m": float(height_m),
            "label": str(label),
            "boxes": [],
        }

        return detected, distance_m, height_m, label, debug_out

    def shutdown(self):
        self.lidar.shutdown()


class ObstacleAvoider:
    """
    Takes the lane follower's steering/throttle and overrides throttle if YOLO says stop.
    """

    def __init__(self, cfg):
        self.stop_throttle = float(getattr(cfg, "YOLO_STOP_THROTTLE", 0.0))
        self.stop_steering = float(getattr(cfg, "YOLO_STOP_STEERING", 0.0))

    def run(self, steering, throttle, detected, distance_m):
        if detected:
            return self.stop_steering, self.stop_throttle

        return steering, throttle
