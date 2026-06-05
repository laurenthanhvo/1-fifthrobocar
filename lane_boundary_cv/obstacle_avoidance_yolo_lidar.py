import math
import time
import socket
import struct
import threading
import logging
from pathlib import Path

import cv2
import numpy as np


logger = logging.getLogger(__name__)


class LivoxMid360Reader:
    """
    Livox MID-360 UDP point cloud reader.

    Based on observed packets:
        LiDAR 192.168.1.199:56300 -> Jetson 192.168.1.50:56301
        UDP length 1380

    Parses Livox point data packets and stores recent Cartesian points.
    """

    def __init__(self, cfg):
        self.enable = bool(getattr(cfg, "ETH_LIDAR_ENABLE", False))
        self.lidar_ip = str(getattr(cfg, "ETH_LIDAR_IP", "192.168.1.199"))
        self.listen_port = int(getattr(cfg, "ETH_LIDAR_DATA_PORT", 56301))

        self.min_forward_m = float(getattr(cfg, "LIVOX_MIN_FORWARD_M", 0.05))
        self.max_forward_m = float(getattr(cfg, "LIVOX_MAX_FORWARD_M", 5.0))
        self.min_points = int(getattr(cfg, "LIVOX_MIN_POINTS_FOR_MATCH", 5))

        self.points_lock = threading.Lock()
        self.points_xyz_m = np.empty((0, 3), dtype=np.float32)

        self.running = False
        self.thread = None
        self.sock = None

        if self.enable:
            self.start()

    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", self.listen_port))
            self.sock.settimeout(0.5)

            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

            logger.info(
                f"Livox MID-360 UDP reader listening on :{self.listen_port}, "
                f"accepting packets from {self.lidar_ip}"
            )

        except Exception as e:
            logger.warning(f"Livox UDP reader disabled: {e}")
            self.enable = False
            self.running = False

    def _parse_packet(self, data):
        if len(data) < 36:
            return None

        version = data[0]
        pkt_len = struct.unpack_from("<H", data, 1)[0]
        dot_num = struct.unpack_from("<H", data, 5)[0]
        data_type = data[10]

        # Sanity check, but do not require exact equality because some devices
        # may pad packets.
        if pkt_len > len(data):
            return None

        points = []

        # Data type 1:
        # x,y,z int32 mm + reflectivity uint8 + tag uint8 = 14 bytes
        if data_type == 1:
            offset = 36
            stride = 14

            for i in range(dot_num):
                base = offset + i * stride
                if base + stride > len(data):
                    break

                x_mm, y_mm, z_mm = struct.unpack_from("<iii", data, base)
                points.append((x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0))

        # Data type 2:
        # x,y,z int16 in 10mm units + reflectivity + tag = 8 bytes
        elif data_type == 2:
            offset = 36
            stride = 8

            for i in range(dot_num):
                base = offset + i * stride
                if base + stride > len(data):
                    break

                x_raw, y_raw, z_raw = struct.unpack_from("<hhh", data, base)
                points.append((x_raw * 0.01, y_raw * 0.01, z_raw * 0.01))

        else:
            return None

        if not points:
            return None

        return np.array(points, dtype=np.float32)

    def _loop(self):
        recent_batches = []
        max_batches = 40

        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)

                if addr[0] != self.lidar_ip:
                    continue

                pts = self._parse_packet(data)
                if pts is None or pts.size == 0:
                    continue

                recent_batches.append(pts)
                if len(recent_batches) > max_batches:
                    recent_batches.pop(0)

                merged = np.vstack(recent_batches)

                # Basic forward filter.
                x = merged[:, 0]
                valid = (x >= self.min_forward_m) & (x <= self.max_forward_m)
                merged = merged[valid]

                with self.points_lock:
                    self.points_xyz_m = merged

            except socket.timeout:
                continue
            except Exception as e:
                logger.warning(f"Livox UDP read/parse error: {e}")
                time.sleep(0.1)

    def get_distance_at_camera_angle(
        self,
        camera_angle_deg,
        angle_sign=-1.0,
        angle_offset_deg=0.0,
        angle_window_deg=6.0,
    ):
        """
        Match a YOLO bbox horizontal camera angle to nearby LiDAR points.

        Assumption:
            Livox x = forward
            Livox y = lateral
            Livox z = vertical

        If matching is mirrored, change LIVOX_CAMERA_ANGLE_SIGN in myconfig.py.
        """
        if not self.enable:
            return None, None

        with self.points_lock:
            pts = self.points_xyz_m.copy()

        if pts.shape[0] < self.min_points:
            return None, None

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        horiz_range = np.sqrt(x * x + y * y)

        lidar_angle_deg = np.degrees(np.arctan2(y, x))

        target_angle = angle_sign * float(camera_angle_deg) + float(angle_offset_deg)

        diff = (lidar_angle_deg - target_angle + 180.0) % 360.0 - 180.0

        valid = (
            (np.abs(diff) <= float(angle_window_deg))
            & (x >= self.min_forward_m)
            & (x <= self.max_forward_m)
        )

        selected = pts[valid]
        selected_range = horiz_range[valid]

        if selected.shape[0] < self.min_points:
            return None, None

        # Use the closest cluster, not random far background.
        order = np.argsort(selected_range)
        selected = selected[order]
        selected_range = selected_range[order]

        cluster_n = min(30, selected.shape[0])
        cluster = selected[:cluster_n]
        cluster_range = selected_range[:cluster_n]

        distance_m = float(np.median(cluster_range))

        # Optional LiDAR vertical extent. This may include ground points,
        # so object height from camera bbox is still the main estimate.
        lidar_height_m = float(np.percentile(cluster[:, 2], 95) - np.percentile(cluster[:, 2], 5))

        return distance_m, lidar_height_m

    def shutdown(self):
        self.running = False

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass


class YoloLidarObstacleDetector:
    """
    DonkeyCar part.

    Inputs:
        cam/image_array
        cv/image_array

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

        self.fx_px = float(getattr(cfg, "CAMERA_FX_PX", 280.45))
        self.fy_px = float(getattr(cfg, "CAMERA_FY_PX", 280.47))
        self._load_camera_intrinsics(cfg)

        self.angle_sign = float(getattr(cfg, "LIVOX_CAMERA_ANGLE_SIGN", -1.0))
        self.angle_offset_deg = float(getattr(cfg, "LIVOX_CAMERA_ANGLE_OFFSET_DEG", 0.0))
        self.angle_window_deg = float(getattr(cfg, "LIVOX_ANGLE_WINDOW_DEG", 6.0))

        self.lidar = LivoxMid360Reader(cfg)

        self.last_result = {
            "detected": False,
            "distance_m": -1.0,
            "height_m": -1.0,
            "label": "",
            "debug_img": None,
        }

        self._setup_birdseye(cfg)

        from ultralytics import YOLO
        self.model = YOLO(self.model_path)

        logger.info(f"YOLO loaded: {self.model_path}")

    def _load_camera_intrinsics(self, cfg):
        cal_file = getattr(cfg, "OAK_CALIBRATION_FILE", None)

        if not cal_file:
            return

        try:
            with np.load(cal_file) as data:
                K = np.array(data["camera_matrix"], dtype=np.float32)
                # self.fx_px = float(K[0, 0])
                # self.fy_px = float(K[1, 1])
                cal_w = int(data["image_width"].item())
                cal_h = int(data["image_height"].item())

            # Changes added here - scale intrisnics to match current capture resolutions
            scale_x = float(cfg.IMAGE_W) / float(cal_w)
            scale_y = float(cfg.IMAGE_H) / float(cal_h)

            self.fx_px = float(K[0, 0]) * scale_x
            self.fy_px = float(K[1, 1]) * scale_y

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

    def _camera_x_to_angle(self, x_px, image_w):
        dx = float(x_px) - float(image_w) / 2.0
        return math.degrees(math.atan2(dx, max(self.fx_px, 1.0)))

    def _estimate_size_from_bbox(self, bbox, distance_m):
        x1, y1, x2, y2 = bbox

        bbox_w = max(1.0, float(x2 - x1))
        bbox_h = max(1.0, float(y2 - y1))

        width_m = bbox_w * distance_m / max(self.fx_px, 1.0)
        height_m = bbox_h * distance_m / max(self.fy_px, 1.0)

        return width_m, height_m

    def _lane_bounds_from_debug_image(self, debug_img, foot_y):
        """
        Estimate lane bounds from green overlay in cv/image_array.
        """
        h, w = debug_img.shape[:2]
        y = int(max(0, min(h - 1, foot_y)))

        y0 = max(0, y - 8)
        y1 = min(h, y + 9)

        band = debug_img[y0:y1, :, :]

        # RGB green overlay
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

        return float(w * 0.25), float(w * 0.75)

    def _is_inside_lane(self, debug_img, bird_foot_x, bird_foot_y):
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

        if self.frame_count % max(self.run_every_n, 1) != 0:
            r = self.last_result
            if r["debug_img"] is not None:
                return r["detected"], r["distance_m"], r["height_m"], r["label"], r["debug_img"]
            return r["detected"], r["distance_m"], r["height_m"], r["label"], debug_img

        h, w = cam_img.shape[:2]
        detections = self._run_yolo(cam_img)

        debug_out = np.copy(debug_img)
        best = None

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

            cam_angle_deg = self._camera_x_to_angle(cx, w)

            distance_m, lidar_height_m = self.lidar.get_distance_at_camera_angle(
                cam_angle_deg,
                angle_sign=self.angle_sign,
                angle_offset_deg=self.angle_offset_deg,
                angle_window_deg=self.angle_window_deg,
            )

            if distance_m is None:
                distance_m = -1.0
                width_m = -1.0
                height_m = -1.0
            else:
                width_m, height_m = self._estimate_size_from_bbox(det["bbox"], distance_m)

            should_stop = (
                distance_m > 0.0
                and distance_m <= self.stop_distance_m
                and height_m >= self.min_height_m
            )

            score_distance = distance_m if distance_m > 0.0 else 999.0

            candidate = {
                "det": det,
                "distance_m": float(distance_m),
                "width_m": float(width_m),
                "height_m": float(height_m),
                "lidar_height_m": float(lidar_height_m) if lidar_height_m is not None else -1.0,
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
            "debug_img": debug_out,
        }

        return detected, distance_m, height_m, label, debug_out

    def shutdown(self):
        self.lidar.shutdown()


class ObstacleAvoider:
    """
    Override lane follower steering/throttle if YOLO + LiDAR says stop.
    """

    def __init__(self, cfg):
        self.stop_throttle = float(getattr(cfg, "YOLO_STOP_THROTTLE", 0.0))
        self.stop_steering = float(getattr(cfg, "YOLO_STOP_STEERING", 0.0))

    def run(self, steering, throttle, detected, distance_m):
        if detected:
            return self.stop_steering, self.stop_throttle

        return steering, throttle
