import cv2
import numpy as np


class LaneCenterFollower:
    def __init__(self, pid, cfg):
        self.pid_st = pid

        # Same simple HSV lane detector as the original working version.
        self.lane_hsv_low = np.array(
            getattr(cfg, "LANE_HSV_LOW", (90, 50, 50)),
            dtype=np.uint8,
        )
        self.lane_hsv_high = np.array(
            getattr(cfg, "LANE_HSV_HIGH", (130, 255, 255)),
            dtype=np.uint8,
        )
        self.lane_mask_color = tuple(getattr(cfg, "LANE_MASK_COLOR", (0, 255, 0)))

        # ============================================================
        # Bird's-eye view
        # ============================================================

        self.use_birdseye = getattr(cfg, "LANE_USE_BIRDSEYE", True)

        # Source trapezoid in the raw camera image.
        self.bird_src_bottom_left = getattr(cfg, "LANE_BIRD_SRC_BOTTOM_LEFT", (0.05, 0.98))
        self.bird_src_bottom_right = getattr(cfg, "LANE_BIRD_SRC_BOTTOM_RIGHT", (0.95, 0.98))
        self.bird_src_top_right = getattr(cfg, "LANE_BIRD_SRC_TOP_RIGHT", (0.68, 0.52))
        self.bird_src_top_left = getattr(cfg, "LANE_BIRD_SRC_TOP_LEFT", (0.32, 0.52))

        # Destination rectangle in the bird's-eye image.
        self.bird_dst_bottom_left = getattr(cfg, "LANE_BIRD_DST_BOTTOM_LEFT", (0.15, 1.00))
        self.bird_dst_bottom_right = getattr(cfg, "LANE_BIRD_DST_BOTTOM_RIGHT", (0.85, 1.00))
        self.bird_dst_top_right = getattr(cfg, "LANE_BIRD_DST_TOP_RIGHT", (0.85, 0.00))
        self.bird_dst_top_left = getattr(cfg, "LANE_BIRD_DST_TOP_LEFT", (0.15, 0.00))

        # ============================================================
        # Detection / curve settings
        # ============================================================

        self.roi_y_start_frac = getattr(cfg, "LANE_ROI_Y_START", 0.20)
        self.roi_y_end_frac = getattr(cfg, "LANE_ROI_Y_END", 1.00)

        # Smaller = farther ahead into curve.
        self.lookahead_y_fraction = getattr(cfg, "LANE_LOOKAHEAD_Y_FRACTION", 0.42)

        self.num_bands = getattr(cfg, "LANE_NUM_BANDS", 12)
        self.min_pixels_per_side = getattr(cfg, "LANE_MIN_PIXELS_PER_SIDE", 10)

        # Smoothing.
        self.smoothed_lane_center_x = None
        self.smoothing_alpha = getattr(cfg, "LANE_SMOOTHING_ALPHA", 0.30)

        # Steering.
        self.steering_gain = getattr(cfg, "LANE_STEERING_GAIN", 0.065)
        self.max_steering = getattr(cfg, "LANE_MAX_STEERING", 0.75)
        self.steering_bias = getattr(cfg, "LANE_STEERING_BIAS", 0.0)

        # Throttle.
        self.test_throttle = getattr(cfg, "LANE_TEST_THROTTLE", 0.35)

        # ============================================================
        # Obstacle detection / stop behavior
        # ============================================================

        self.obstacle_enable = getattr(cfg, "OBSTACLE_ENABLE", True)

        # Detect obstacle inside a corridor centered around the lane center.
        # This is in the same control image used for lane following.
        self.obstacle_roi_y_start = getattr(cfg, "OBSTACLE_ROI_Y_START", 0.20)
        self.obstacle_roi_y_end = getattr(cfg, "OBSTACLE_ROI_Y_END", 1.00)
        self.obstacle_corridor_half_width_px = getattr(
            cfg,
            "OBSTACLE_CORRIDOR_HALF_WIDTH_PX",
            180,
        )

        # Red/orange obstacle HSV thresholds.
        # Red wraps around HSV hue, so use two ranges.
        self.obstacle_hsv_low1 = np.array(
            getattr(cfg, "OBSTACLE_HSV_LOW1", (0, 80, 50)),
            dtype=np.uint8,
        )
        self.obstacle_hsv_high1 = np.array(
            getattr(cfg, "OBSTACLE_HSV_HIGH1", (20, 255, 255)),
            dtype=np.uint8,
        )
        self.obstacle_hsv_low2 = np.array(
            getattr(cfg, "OBSTACLE_HSV_LOW2", (160, 80, 50)),
            dtype=np.uint8,
        )
        self.obstacle_hsv_high2 = np.array(
            getattr(cfg, "OBSTACLE_HSV_HIGH2", (179, 255, 255)),
            dtype=np.uint8,
        )

        # Ignore tiny blobs.
        self.obstacle_min_area = getattr(cfg, "OBSTACLE_MIN_AREA", 350)
        self.obstacle_min_bbox_height_px = getattr(cfg, "OBSTACLE_MIN_BBOX_HEIGHT_PX", 35)
        self.obstacle_min_bbox_width_px = getattr(cfg, "OBSTACLE_MIN_BBOX_WIDTH_PX", 20)
        self.obstacle_lane_mask_margin_px = getattr(cfg, "OBSTACLE_LANE_MASK_MARGIN_PX", 25)

        # Optional dark/black obstacle detection.
        # Useful for black bottles, dark boxes, shadows/objects.
        # Set OBSTACLE_DARK_V_MAX in myconfig.py to enable.
        self.obstacle_dark_v_max = getattr(cfg, "OBSTACLE_DARK_V_MAX", None)

        # If True, stop completely when obstacle is found.
        self.obstacle_stop_throttle = getattr(cfg, "OBSTACLE_STOP_THROTTLE", 0.0)
        self.obstacle_stop_steering = getattr(cfg, "OBSTACLE_STOP_STEERING", 0.0)

        # Stop-distance approximation.
        # In bird's-eye view, objects lower in the image are closer to the car.
        # Calibrate this by placing the obstacle 1 foot in front of the car.
        self.obstacle_stop_y_frac = getattr(cfg, "OBSTACLE_STOP_Y_FRAC", 0.72)

        # Require a few consecutive detections before stopping to avoid false positives.
        self.obstacle_confirm_frames = getattr(cfg, "OBSTACLE_CONFIRM_FRAMES", 2)
        self.obstacle_seen_count = 0

        # One-line fallback.
        self.allow_one_side_fallback = getattr(cfg, "LANE_ALLOW_ONE_SIDE_FALLBACK", True)
        self.expected_lane_width_px = getattr(cfg, "LANE_EXPECTED_WIDTH_PX", None)
        self.expected_lane_width_frac = getattr(cfg, "LANE_EXPECTED_WIDTH_FRAC", 0.35)
        self.learned_lane_width_px = None

        # If only one line is visible:
        # "right_of_left" means visible line is left boundary, center is to its right.
        # "left_of_right" means visible line is right boundary, center is to its left.
        
        # DO NOT CHANGE THIS!!
        # One-line fallback behavior:
        # "auto" = decide whether the visible line is left or right boundary
        # based on where it appears relative to the expected lane center.
        self.keep_side = getattr(cfg, "LANE_KEEP_SIDE", "auto")

        # If the visible line is very close to expected center, keep the last decision.
        self.one_line_side_margin_px = getattr(cfg, "LANE_ONE_LINE_SIDE_MARGIN_PX", 25)
        self.last_visible_side = "left"

        # Reject fake two-lane detections.
        self.min_lane_width_frac = getattr(cfg, "LANE_MIN_WIDTH_FRAC", 0.15)
        self.max_lane_width_frac = getattr(cfg, "LANE_MAX_WIDTH_FRAC", 0.85)

        self.smoothed_steering   = None
        self.steering_output_alpha = getattr(cfg, "LANE_STEERING_SMOOTHING", 0.25)
        self.steering_deadband_px  = getattr(cfg, "LANE_DEADBAND_PX", 10)

        # jump filter
        self.max_center_jump_px = getattr(cfg, "LANE_MAX_CENTER_JUMP_PX", 80)

        # Track left/right lane boundaries across bands and frames.
        # This is needed for curves where only one boundary is visible.
        self.last_left_x = None
        self.last_right_x = None
        self.track_max_gap_px = getattr(cfg, "LANE_TRACK_MAX_GAP_PX", 180)

        # Straight recovery mode:
        # After a sharp curve, quickly calm steering when the path becomes straight.
        self.straight_path_delta_px = getattr(cfg, "LANE_STRAIGHT_PATH_DELTA_PX", 45)
        self.straight_error_px = getattr(cfg, "LANE_STRAIGHT_ERROR_PX", 35)

        self.center_recovery_alpha = getattr(cfg, "LANE_CENTER_RECOVERY_ALPHA", 0.85)
        self.steering_recovery_alpha = getattr(cfg, "LANE_STEERING_RECOVERY_ALPHA", 0.75)

        self.straight_steering_gain = getattr(
            cfg,
            "LANE_STRAIGHT_STEERING_GAIN",
            self.steering_gain * 0.45,
        )

        self.curve_steering_gain = getattr(
            cfg,
            "LANE_CURVE_STEERING_GAIN",
            self.steering_gain,
        )

        # Curve-following improvement:
        # Use path heading/curvature in addition to lateral center error.
        self.heading_gain = getattr(cfg, "LANE_HEADING_GAIN", 0.004)

        # Keep the target away from lane boundaries so it does not cut corners.
        self.boundary_margin_px = getattr(cfg, "LANE_BOUNDARY_MARGIN_PX", 45)

        # Optional target bias on left turns.
        # Negative shifts target left during left curves.
        self.left_curve_target_bias_px = getattr(cfg, "LANE_LEFT_CURVE_TARGET_BIAS_PX", -20)

        # Slow down only while turning sharply.
        self.turn_throttle = getattr(cfg, "LANE_THROTTLE_TURN", self.test_throttle)
        self.turn_slowdown_steer_threshold = getattr(cfg, "LANE_TURN_SLOWDOWN_STEER_THRESHOLD", 0.35)
        # toggle button for heatmap
        self.show_depth = getattr(cfg, "DEPTH_OVERLAY_DEFAULT_ON", True)
        self.show_depth       = getattr(cfg, "DEPTH_OVERLAY_DEFAULT_ON", True)
        self.fullscreen_depth = False

        # ============================================================
        # DonkeyCar web view selector
        # ============================================================
        # Button 1: raw RGB
        # Button 2: bird's-eye lane + original obstacle stop
        # Button 3: obstacle focus view
        # Button 4: DepthAI object tracker slot
        # Button 5: depth/stereo heatmap
        self.view_mode = getattr(cfg, "DEFAULT_VIEW_MODE", "lane")

        # YOLO display-only view for Button 3.
        # This does NOT control throttle/steering. It only draws boxes on raw RGB.
        self.yolo_view_enable = bool(getattr(cfg, "YOLO_VIEW_ENABLE", True))
        self.yolo_view_model_path = getattr(
            cfg,
            "YOLO_VIEW_MODEL_PATH",
            getattr(cfg, "YOLO_MODEL_PATH", "models/yolo11n.pt"),
        )
        self.yolo_view_conf = float(getattr(
            cfg,
            "YOLO_VIEW_CONFIDENCE",
            getattr(cfg, "YOLO_CONFIDENCE", 0.30),
        ))
        self.yolo_view_imgsz = int(getattr(
            cfg,
            "YOLO_VIEW_IMAGE_SIZE",
            getattr(cfg, "YOLO_IMAGE_SIZE", 640),
        ))
        self.yolo_view_device = getattr(
            cfg,
            "YOLO_VIEW_DEVICE",
            getattr(cfg, "YOLO_DEVICE", 0),
        )
        self.yolo_view_run_every_n = int(getattr(cfg, "YOLO_VIEW_RUN_EVERY_N_FRAMES", 2))

        yolo_targets = getattr(cfg, "YOLO_VIEW_TARGET_CLASSES", None)
        if yolo_targets is None:
            yolo_targets = getattr(cfg, "YOLO_TARGET_CLASSES", [])
        self.yolo_view_target_classes = set(yolo_targets) if yolo_targets else set()

        self._yolo_view_model = None
        self._yolo_view_last_img = None
        self._yolo_view_frame_count = 0

        # Distance/height display for YOLO View 3.
        self.yolo_view_show_distance_height = bool(
            getattr(cfg, "YOLO_VIEW_SHOW_DISTANCE_HEIGHT", True)
        )
        self.yolo_view_depth_min_mm = float(getattr(cfg, "YOLO_VIEW_DEPTH_MIN_MM", 100))
        self.yolo_view_depth_max_mm = float(getattr(cfg, "YOLO_VIEW_DEPTH_MAX_MM", 5000))
        self.yolo_view_depth_center_frac = float(
            getattr(cfg, "YOLO_VIEW_DEPTH_CENTER_FRAC", 0.35)
        )
        # --- Kanishk Changes here
        # self.camera_fx_px = float(getattr(cfg, "CAMERA_FX_PX", 280.45))
        # self.camera_fy_px = float(getattr(cfg, "CAMERA_FY_PX", 280.47))
        self.camera_fx_px = float(getattr(cfg, "CAMERA_FX_PX", 560.91))
        self.camera_fy_px = float(getattr(cfg, "CAMERA_FY_PX", 560.94))
        self.camera_cx_px = float(getattr(cfg, "CAMERA_CX_PX", 616.58))  # 308.29 * 2
        self.camera_cy_px = float(getattr(cfg, "CAMERA_CY_PX", 433.66))  # 216.83 * 2

        # View 4: bird's-eye YOLO obstacle detector.
        # This labels all accepted YOLO boxes as OBSTACLE and can stop the car.
        self.yolo_bird_enable = bool(getattr(cfg, "YOLO_BIRD_ENABLE", True))
        self.yolo_bird_model_path = getattr(
            cfg,
            "YOLO_BIRD_MODEL_PATH",
            getattr(cfg, "YOLO_VIEW_MODEL_PATH", getattr(cfg, "YOLO_MODEL_PATH", "models/yolo11n.pt")),
        )
        self.yolo_bird_conf = float(getattr(
            cfg,
            "YOLO_BIRD_CONFIDENCE",
            getattr(cfg, "YOLO_VIEW_CONFIDENCE", 0.30),
        ))
        self.yolo_bird_imgsz = int(getattr(
            cfg,
            "YOLO_BIRD_IMAGE_SIZE",
            getattr(cfg, "YOLO_VIEW_IMAGE_SIZE", 640),
        ))
        self.yolo_bird_device = getattr(
            cfg,
            "YOLO_BIRD_DEVICE",
            getattr(cfg, "YOLO_VIEW_DEVICE", 0),
        )
        self.yolo_bird_run_every_n = int(getattr(cfg, "YOLO_BIRD_RUN_EVERY_N_FRAMES", 2))

        bird_targets = getattr(cfg, "YOLO_BIRD_TARGET_CLASSES", [])
        self.yolo_bird_target_classes = set(bird_targets) if bird_targets else set()

        self.yolo_bird_min_stop_height_m = float(getattr(cfg, "YOLO_BIRD_MIN_STOP_HEIGHT_M", 0.07))
        self.yolo_bird_stop_distance_m = float(getattr(cfg, "YOLO_BIRD_STOP_DISTANCE_M", 0.4572))
        self.yolo_bird_stop_throttle = float(getattr(cfg, "YOLO_BIRD_STOP_THROTTLE", 0.0))
        self.yolo_bird_stop_steering = float(getattr(cfg, "YOLO_BIRD_STOP_STEERING", 0.0))
        self.yolo_bird_require_inside_lane = bool(getattr(cfg, "YOLO_BIRD_REQUIRE_INSIDE_LANE", True))

        self.yolo_bird_depth_min_mm = float(getattr(cfg, "YOLO_BIRD_DEPTH_MIN_MM", 100))
        self.yolo_bird_depth_max_mm = float(getattr(cfg, "YOLO_BIRD_DEPTH_MAX_MM", 5000))
        self.yolo_bird_depth_center_frac = float(getattr(cfg, "YOLO_BIRD_DEPTH_CENTER_FRAC", 0.35))

        self._yolo_bird_model = None
        self._yolo_bird_last_img = None
        self._yolo_bird_last_stop = False
        self._yolo_bird_frame_count = 0
        self.yolo_bird_seen_count = 0
        self.yolo_bird_lane_overlap_min = float(getattr(cfg, "YOLO_BIRD_LANE_OVERLAP_MIN", 0.20))
        self.yolo_bird_base_frac = float(getattr(cfg, "YOLO_BIRD_BASE_FRAC", 0.25))
        self.yolo_bird_color_fallback = bool(getattr(cfg, "YOLO_BIRD_COLOR_FALLBACK", True))
        self.car_underbelly_clearance_m = float(
            getattr(cfg, "CAR_UNDERBELLY_CLEARANCE_M", self.yolo_bird_min_stop_height_m)
        )
        self.yolo_bird_dark_fallback = bool(getattr(cfg, "YOLO_BIRD_DARK_FALLBACK", True))
        self.yolo_bird_dark_v_max = int(getattr(cfg, "YOLO_BIRD_DARK_V_MAX", 65))
        self.yolo_bird_dark_min_area = float(getattr(cfg, "YOLO_BIRD_DARK_MIN_AREA", 450))
        self.yolo_bird_dark_min_width_px = int(getattr(cfg, "YOLO_BIRD_DARK_MIN_WIDTH_PX", 18))
        self.yolo_bird_dark_min_height_px = int(getattr(cfg, "YOLO_BIRD_DARK_MIN_HEIGHT_PX", 18))
        self.yolo_bird_max_dark_candidates = int(
            getattr(cfg, "YOLO_BIRD_MAX_DARK_CANDIDATES", 12)
        )



    # ============================================================
    # Bird's-eye transform
    # ============================================================

    def _birdseye_transform(self, img):
        h, w = img.shape[:2]

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

        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, (w, h))

        return warped

    # ============================================================
    # Mask utilities
    # ============================================================

    def _clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)

        # Remove small specks.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Fill small gaps in tape.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def _get_lane_width_px(self, image_w):
        if self.expected_lane_width_px is not None:
            return float(self.expected_lane_width_px)

        if self.learned_lane_width_px is not None:
            return float(self.learned_lane_width_px)

        return float(self.expected_lane_width_frac * image_w)

    def _update_lane_width(self, width_px):
        if width_px <= 0:
            return

        if self.learned_lane_width_px is None:
            self.learned_lane_width_px = float(width_px)
        else:
            self.learned_lane_width_px = (
                0.20 * float(width_px)
                + 0.80 * self.learned_lane_width_px
            )

    # ============================================================
    # Lane center extraction
    # ============================================================

    def _find_runs_in_band(self, band):
        """
        Find horizontal clusters of detected lane pixels inside one band.
        Each run is a possible lane boundary.
        """
        col_counts = np.sum(band > 0, axis=0).astype(np.float32)

        # Smooth the histogram so broken tape still forms a clean run.
        smooth_kernel = np.ones(7, dtype=np.float32) / 7.0
        col_counts = np.convolve(col_counts, smooth_kernel, mode="same")

        min_col_count = 2
        min_run_width = 4

        active = col_counts >= min_col_count

        runs = []
        start = None

        for x, is_active in enumerate(active):
            if is_active and start is None:
                start = x
            elif not is_active and start is not None:
                end = x - 1
                if end - start + 1 >= min_run_width:
                    score = float(np.sum(col_counts[start:end + 1]))
                    runs.append({
                        "start": int(start),
                        "end": int(end),
                        "center": int((start + end) // 2),
                        "score": score,
                    })
                start = None

        if start is not None:
            end = len(active) - 1
            if end - start + 1 >= min_run_width:
                score = float(np.sum(col_counts[start:end + 1]))
                runs.append({
                    "start": int(start),
                    "end": int(end),
                    "center": int((start + end) // 2),
                    "score": score,
                })

        return runs

    def _extract_center_points(self, mask, image_center_x):
        """
        Build center points across horizontal bands.

        Tracks left/right lane boundaries separately:
        - If both boundaries are visible, center = midpoint.
        - If only one boundary is visible, infer center using lane width.
        - Scans bottom-to-top because nearby lane detections are most reliable.
        """
        h, w = mask.shape
        center_points = []
        debug_points = []

        min_lane_width = int(self.min_lane_width_frac * w)
        max_lane_width = int(self.max_lane_width_frac * w)

        lane_width_est = self._get_lane_width_px(w)

        expected_center = (
            self.smoothed_lane_center_x
            if self.smoothed_lane_center_x is not None
            else image_center_x
        )

        track_left_x = self.last_left_x
        track_right_x = self.last_right_x
        prev_center_x = None

        # Bottom-to-top scan.
        for i in reversed(range(self.num_bands)):
            y0 = int(h * i / self.num_bands)
            y1 = int(h * (i + 1) / self.num_bands)
            center_y = (y0 + y1) // 2

            band = mask[y0:y1, :]

            left_x = None
            right_x = None
            center_x = None
            method = "none"

            if int(np.count_nonzero(band)) < self.min_pixels_per_side:
                debug_points.append({
                    "x": None,
                    "y": center_y,
                    "left_x": None,
                    "right_x": None,
                    "method": "empty",
                })
                continue

            runs = self._find_runs_in_band(band)

            if len(runs) == 0:
                debug_points.append({
                    "x": None,
                    "y": center_y,
                    "left_x": None,
                    "right_x": None,
                    "method": "no_runs",
                })
                continue

            # --------------------------------------------------------
            # Case 1: find both lane boundaries.
            # --------------------------------------------------------
            best_pair = None
            best_score = -1e9

            for a in range(len(runs)):
                for b in range(a + 1, len(runs)):
                    r1 = runs[a]
                    r2 = runs[b]

                    lx = min(r1["center"], r2["center"])
                    rx = max(r1["center"], r2["center"])
                    width = rx - lx

                    if not (min_lane_width <= width <= max_lane_width):
                        continue

                    pair_center = 0.5 * (lx + rx)

                    score = r1["score"] + r2["score"]

                    # Prefer expected lane width and expected center.
                    score -= 0.25 * abs(width - lane_width_est)
                    score -= 0.20 * abs(pair_center - expected_center)

                    # Prefer continuity from previous frame/band.
                    if track_left_x is not None:
                        score -= 0.35 * abs(lx - track_left_x)

                    if track_right_x is not None:
                        score -= 0.35 * abs(rx - track_right_x)

                    if score > best_score:
                        best_score = score
                        best_pair = (lx, rx, width)

            if best_pair is not None:
                left_x, right_x, lane_width = best_pair
                center_x = int((left_x + right_x) // 2)

                self._update_lane_width(lane_width)
                lane_width_est = self._get_lane_width_px(w)

                track_left_x = left_x
                track_right_x = right_x
                self.last_left_x = left_x
                self.last_right_x = right_x

                method = "both_tracked"

            # --------------------------------------------------------
            # Case 2: only one boundary visible.
            # --------------------------------------------------------
            elif self.allow_one_side_fallback:
                strongest = sorted(runs, key=lambda r: r["score"], reverse=True)[0]
                visible_x = int(strongest["center"])

                visible_side = None

                if track_left_x is not None and track_right_x is not None:
                    dist_left = abs(visible_x - track_left_x)
                    dist_right = abs(visible_x - track_right_x)

                    if dist_left < dist_right:
                        visible_side = "left"
                    else:
                        visible_side = "right"

                elif track_left_x is not None:
                    if abs(visible_x - track_left_x) < self.track_max_gap_px:
                        visible_side = "left"

                elif track_right_x is not None:
                    if abs(visible_x - track_right_x) < self.track_max_gap_px:
                        visible_side = "right"

                if visible_side is None:
                    if visible_x < image_center_x:
                        visible_side = "left"
                    else:
                        visible_side = "right"

                if visible_side == "left":
                    left_x = visible_x
                    right_x = None
                    center_x = int(visible_x + lane_width_est / 2.0)
                    center_x = max(0, min(w - 1, center_x))

                    track_left_x = visible_x
                    self.last_left_x = visible_x

                    method = "one_tracked_left"

                else:
                    left_x = None
                    right_x = visible_x
                    center_x = int(visible_x - lane_width_est / 2.0)
                    center_x = max(0, min(w - 1, center_x))

                    track_right_x = visible_x
                    self.last_right_x = visible_x

                    method = "one_tracked_right"

            # --------------------------------------------------------
            # Reject impossible jumps.
            # --------------------------------------------------------
            if center_x is not None:
                if prev_center_x is not None:
                    if abs(center_x - prev_center_x) > self.max_center_jump_px:
                        debug_points.append({
                            "x": None,
                            "y": center_y,
                            "left_x": None,
                            "right_x": None,
                            "method": "rejected_jump",
                        })
                        continue

                prev_center_x = center_x

                weight = 1.0 + 4.0 * (center_y / max(1.0, h))

                center_points.append({
                    "x": int(center_x),
                    "y": int(center_y),
                    "weight": float(weight),
                    "left_x": left_x,
                    "right_x": right_x,
                    "method": method,
                })

            debug_points.append({
                "x": center_x,
                "y": center_y,
                "left_x": left_x,
                "right_x": right_x,
                "method": method,
            })

        center_points = sorted(center_points, key=lambda p: p["y"])
        debug_points = sorted(debug_points, key=lambda p: p["y"])

        return center_points, debug_points
    

    # ============================================================
    # Web button view controls
    # ============================================================

    def set_view_raw(self):
        self.view_mode = "raw"
        print("View mode: raw RGB camera")

    def set_view_lane(self):
        self.view_mode = "lane"
        print("View mode: bird's-eye lane + original obstacle stop")

    def set_view_obstacle(self):
        self.view_mode = "obstacle"
        print("View mode: obstacle detection focus")

    def set_view_tracker(self):
        self.view_mode = "tracker"
        print("View mode: bird-eye YOLO obstacle detector")

    def set_view_depth(self):
        self.view_mode = "depth"
        print("View mode: depth/stereo heatmap")

    def _draw_view_label(self, img, label):
        out = np.copy(img)
        cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(
            out,
            label,
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return out


    def _get_yolo_view_model(self):
        if not getattr(self, "yolo_view_enable", True):
            return None

        if self._yolo_view_model is None:
            from ultralytics import YOLO
            self._yolo_view_model = YOLO(self.yolo_view_model_path)
            print(f"Loaded YOLO view model: {self.yolo_view_model_path}")

        return self._yolo_view_model


    # def _depth_distance_for_bbox(self, depth_img, bbox, image_shape):
    #     """
    #     Estimate object distance from the OAK-D depth map.

    #     Uses the center region of the YOLO box and returns median valid depth in meters.
    #     This is more stable than using a single pixel.
    #     """
    #     if depth_img is None:
    #         return None

    #     img_h, img_w = image_shape[:2]

    #     depth = depth_img

    #     # If depth resolution differs from RGB, resize it to match RGB.
    #     if depth.shape[0] != img_h or depth.shape[1] != img_w:
    #         depth = cv2.resize(depth, (img_w, img_h), interpolation=cv2.INTER_NEAREST)

    #     x1, y1, x2, y2 = bbox
    #     x1 = max(0, min(img_w - 1, int(x1)))
    #     x2 = max(0, min(img_w - 1, int(x2)))
    #     y1 = max(0, min(img_h - 1, int(y1)))
    #     y2 = max(0, min(img_h - 1, int(y2)))

    #     if x2 <= x1 or y2 <= y1:
    #         return None

    #     # Use center crop of bbox to avoid background at the edges.
    #     frac = max(0.10, min(1.00, float(self.yolo_view_depth_center_frac)))

    #     bw = x2 - x1
    #     bh = y2 - y1

    #     cx1 = int(x1 + (1.0 - frac) * 0.5 * bw)
    #     cx2 = int(x2 - (1.0 - frac) * 0.5 * bw)
    #     cy1 = int(y1 + (1.0 - frac) * 0.5 * bh)
    #     cy2 = int(y2 - (1.0 - frac) * 0.5 * bh)

    #     roi = depth[cy1:cy2, cx1:cx2].astype(np.float32)

    #     if roi.size == 0:
    #         return None

    #     valid = roi[
    #         (roi >= self.yolo_view_depth_min_mm)
    #         & (roi <= self.yolo_view_depth_max_mm)
    #     ]

    #     if valid.size < 20:
    #         return None

    #     distance_mm = float(np.median(valid))
    #     return distance_mm / 1000.0
    def _depth_distance_for_bbox(self, depth_img, bbox, image_shape):
        print(f"DEBUG bbox entry: depth={depth_img is not None} bbox={bbox} shape={image_shape[:2]}")
        if depth_img is None:
            print("DEBUG: depth_img is None")
            return None

        img_h, img_w = image_shape[:2]
        depth = depth_img

        if depth.shape[0] != img_h or depth.shape[1] != img_w:
            print(f"DEBUG: resizing depth {depth.shape} -> ({img_h},{img_w})")
            depth = cv2.resize(depth, (img_w, img_h), interpolation=cv2.INTER_NEAREST)

        x1, y1, x2, y2 = bbox
        x1 = max(0, min(img_w - 1, int(x1)))
        x2 = max(0, min(img_w - 1, int(x2)))
        y1 = max(0, min(img_h - 1, int(y1)))
        y2 = max(0, min(img_h - 1, int(y2)))

        frac = max(0.10, min(1.00, float(self.yolo_view_depth_center_frac)))
        bw = x2 - x1
        bh = y2 - y1

        cx1 = int(x1 + (1.0 - frac) * 0.5 * bw)
        cx2 = int(x2 - (1.0 - frac) * 0.5 * bw)
        cy1 = int(y1 + (1.0 - frac) * 0.5 * bh)
        cy2 = int(y2 - (1.0 - frac) * 0.5 * bh)

        roi = depth[cy1:cy2, cx1:cx2].astype(np.float32)

        print(f"DEBUG: roi shape={roi.shape} min={roi.min():.0f} max={roi.max():.0f} "
            f"nonzero={np.count_nonzero(roi)} size={roi.size}")

        valid = roi[
            (roi >= self.yolo_view_depth_min_mm)
            & (roi <= self.yolo_view_depth_max_mm)
        ]

        print(f"DEBUG: valid pixels={valid.size} threshold=20 "
            f"min_mm={self.yolo_view_depth_min_mm} max_mm={self.yolo_view_depth_max_mm}")

        # if valid.size < 20:
        min_valid = int(getattr(self, "yolo_view_depth_min_valid_px",
                getattr(self, "VIEW4_MIN_OBJECT_POINTS", 5)))
        if valid.size < min_valid:
            return None

        distance_mm = float(np.median(valid))
        return distance_mm / 1000.0

    def _estimate_object_size_from_bbox(self, bbox, distance_m):
        """
        Approximate real object width/height using pinhole camera model.

        height_m = bbox_pixel_height * distance_m / fy
        width_m  = bbox_pixel_width  * distance_m / fx
        """
        if distance_m is None or distance_m <= 0:
            return None, None

        x1, y1, x2, y2 = bbox
        bbox_w_px = max(1.0, float(x2 - x1))
        bbox_h_px = max(1.0, float(y2 - y1))

        width_m = bbox_w_px * distance_m / max(1.0, self.camera_fx_px)
        height_m = bbox_h_px * distance_m / max(1.0, self.camera_fy_px)

        return width_m, height_m


    def _render_yolo_raw_view(self, img, depth_img=None):
        """
        View 3: raw RGB camera feed with YOLO bounding boxes only.
        This is display-only and does not affect steering/throttle.
        """
        print(f"DEBUG view3: depth_img is {'None' if depth_img is None else depth_img.shape}")
        out = np.copy(img)
        # h_bird, w_bird = out.shape[:2]

        self._yolo_view_frame_count += 1
        run_every = max(1, int(getattr(self, "yolo_view_run_every_n", 2)))

        # Reuse previous drawn frame on skipped frames for speed.
        if (
            self._yolo_view_last_img is not None
            and self._yolo_view_frame_count % run_every != 0
        ):
            return self._yolo_view_last_img

        try:
            model = self._get_yolo_view_model()

            if model is None:
                return self._draw_view_label(out, "VIEW 3: YOLO DISABLED")

            # Ultralytics expects normal OpenCV/BGR-style ndarray input,
            # but DonkeyCar image is RGB, so convert RGB -> BGR for inference.
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            result = model.predict(
                source=bgr,
                imgsz=self.yolo_view_imgsz,
                conf=self.yolo_view_conf,
                device=self.yolo_view_device,
                verbose=False,
            )[0]

            names = result.names

            if result.boxes is not None:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    label = str(names.get(cls_id, cls_id))
                    conf = float(box.conf[0])

                    if self.yolo_view_target_classes and label not in self.yolo_view_target_classes:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                    # Draw on RGB output image.
                    cv2.rectangle(out, (x1, y1), (x2, y2), (255, 0, 0), 2)

                    distance_m = None
                    height_m = None

                    if self.yolo_view_show_distance_height:
                        distance_m = self._depth_distance_for_bbox(
                            depth_img,
                            (x1, y1, x2, y2),
                            img.shape,
                        )

                        _, height_m = self._estimate_object_size_from_bbox(
                            (x1, y1, x2, y2),
                            distance_m,
                        )

                    if distance_m is not None and height_m is not None:
                        text = f"{label} {conf:.2f} d={distance_m:.2f}m h={height_m:.2f}m"
                    else:
                        text = f"{label} {conf:.2f} d=? h=?"

                    cv2.putText(
                        out,
                        text,
                        (x1, max(20, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

            out = self._draw_view_label(out, "VIEW 3: YOLO OBJECT DETECTOR - RAW RGB")
            self._yolo_view_last_img = out
            return out

        except Exception as e:
            err_img = np.copy(img)
            cv2.putText(
                err_img,
                f"YOLO view error: {str(e)[:70]}",
                (8, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._yolo_view_last_img = err_img
            return err_img



    def _get_yolo_bird_model(self):
        if not getattr(self, "yolo_bird_enable", True):
            return None

        if self._yolo_bird_model is None:
            from ultralytics import YOLO
            self._yolo_bird_model = YOLO(self.yolo_bird_model_path)
            print(f"Loaded YOLO bird obstacle model: {self.yolo_bird_model_path}")

        return self._yolo_bird_model

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

    def _raw_points_to_bird(self, points, w, h):
        M = self._bird_matrix(w, h)
        pts = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(pts, M).reshape(-1, 2)
        return warped

    def _depth_distance_for_bbox_custom(self, depth_img, bbox, image_shape, min_mm, max_mm, center_frac):
        if depth_img is None:
            return None

        img_h, img_w = image_shape[:2]
        depth = depth_img

        if depth.shape[0] != img_h or depth.shape[1] != img_w:
            depth = cv2.resize(depth, (img_w, img_h), interpolation=cv2.INTER_NEAREST)

        x1, y1, x2, y2 = bbox
        x1 = max(0, min(img_w - 1, int(x1)))
        x2 = max(0, min(img_w - 1, int(x2)))
        y1 = max(0, min(img_h - 1, int(y1)))
        y2 = max(0, min(img_h - 1, int(y2)))

        if x2 <= x1 or y2 <= y1:
            return None

        frac = max(0.10, min(1.00, float(center_frac)))

        bw = x2 - x1
        bh = y2 - y1

        cx1 = int(x1 + (1.0 - frac) * 0.5 * bw)
        cx2 = int(x2 - (1.0 - frac) * 0.5 * bw)
        cy1 = int(y1 + (1.0 - frac) * 0.5 * bh)
        cy2 = int(y2 - (1.0 - frac) * 0.5 * bh)

        roi = depth[cy1:cy2, cx1:cx2].astype(np.float32)

        if roi.size == 0:
            return None

        valid = roi[(roi >= float(min_mm)) & (roi <= float(max_mm))]

        if valid.size < 20:
            return None

        return float(np.median(valid)) / 1000.0

    def _lane_bounds_from_green_overlay(self, bird_img, y):
        """
        Estimate lane bounds from the green lane overlay already drawn on debug_img.
        Returns (left_x, right_x), or (None, None) if the lanes cannot be inferred.
        """
        h, w = bird_img.shape[:2]
        y = int(max(0, min(h - 1, y)))

        y0 = max(0, y - 8)
        y1 = min(h, y + 9)

        band = bird_img[y0:y1, :, :]

        # RGB green lane overlay.
        green = (
            (band[:, :, 1] > 170)
            & (band[:, :, 0] < 120)
            & (band[:, :, 2] < 120)
        )

        col_counts = np.sum(green, axis=0)
        active = col_counts >= 2

        runs = []
        start = None

        for x, val in enumerate(active):
            if val and start is None:
                start = x
            elif not val and start is not None:
                end = x - 1
                if end - start + 1 >= 5:
                    runs.append((start, end, (start + end) // 2))
                start = None

        if start is not None:
            end = len(active) - 1
            if end - start + 1 >= 5:
                runs.append((start, end, (start + end) // 2))

        if len(runs) < 2:
            return None, None

        return float(runs[0][2]), float(runs[-1][2])

    def _bird_point_inside_lane(self, bird_img, x, y):
        if not getattr(self, "yolo_bird_require_inside_lane", True):
            return True

        left_x, right_x = self._lane_bounds_from_green_overlay(bird_img, y)

        if left_x is None or right_x is None:
            return False

        if left_x > right_x:
            left_x, right_x = right_x, left_x

        margin = float(getattr(self, "obstacle_lane_margin_px", 10))
        return (x >= left_x + margin) and (x <= right_x - margin)


    def _detect_dark_objects_in_lane_mask(self, control_img, lane_mask):
        """
        View 4-only fallback for black wheels / dark low objects.

        It detects dark blobs only inside the lane mask, then returns bboxes
        in bird's-eye/control-image coordinates.
        """
        hsv = cv2.cvtColor(control_img, cv2.COLOR_RGB2HSV)

        v = hsv[:, :, 2]
        s = hsv[:, :, 1]

        # Dark, but require some saturation/contrast so plain gray floor is less likely.
        dark_mask = ((v < self.yolo_bird_dark_v_max) & (s > 20)).astype(np.uint8) * 255

        # Only inside actual lane.
        dark_mask = cv2.bitwise_and(dark_mask, lane_mask)

        kernel = np.ones((5, 5), np.uint8)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            dark_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self.yolo_bird_dark_min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(c)

            if bw < self.yolo_bird_dark_min_width_px:
                continue

            if bh < self.yolo_bird_dark_min_height_px:
                continue

            candidates.append({
                "bbox": (int(x), int(y), int(bw), int(bh)),
                "area": area,
            })

        return sorted(candidates, key=lambda q: q["area"], reverse=True)

    def _depth_distance_for_bird_bbox(self, depth_img, bird_bbox, image_shape):
        """
        Use the same depth image, but sample using a bbox already in bird/control coordinates.
        This works because control_img/debug_img/depth_img are all same output size.
        """
        if depth_img is None:
            return None

        x, y, bw, bh = bird_bbox
        return self._depth_distance_for_bbox_custom(
            depth_img,
            (x, y, x + bw, y + bh),
            image_shape,
            self.yolo_bird_depth_min_mm,
            self.yolo_bird_depth_max_mm,
            self.yolo_bird_depth_center_frac,
        )



    def _detect_color_objects_raw(self, img):
        """
        Detect red/orange objects in the RAW camera image.
        This is for cones or red/orange obstacles YOLO may miss.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask1 = cv2.inRange(hsv, self.obstacle_hsv_low1, self.obstacle_hsv_high1)
        mask2 = cv2.inRange(hsv, self.obstacle_hsv_low2, self.obstacle_hsv_high2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self.obstacle_min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(c)

            if bw < self.obstacle_min_bbox_width_px:
                continue

            if bh < self.obstacle_min_bbox_height_px:
                continue

            candidates.append({
                "bbox": (int(x), int(y), int(x + bw), int(y + bh)),
                "area": area,
                "source": "COLOR",
                "conf": 1.00,
            })

        return sorted(candidates, key=lambda q: q["area"], reverse=True)


    def _bird_lane_mask_to_raw_mask(self, lane_mask_bird, raw_shape):
        """
        Convert the bird's-eye lane mask back into raw camera coordinates.
        This lets dark-object detection happen in raw RGB/depth space while
        still only accepting objects inside the detected lane.
        """
        raw_h, raw_w = raw_shape[:2]
        bird_h, bird_w = lane_mask_bird.shape[:2]

        M = self._bird_matrix(raw_w, raw_h)
        Minv = np.linalg.inv(M)

        raw_mask = cv2.warpPerspective(
            lane_mask_bird,
            Minv,
            (raw_w, raw_h),
            flags=cv2.INTER_NEAREST,
        )

        raw_mask = (raw_mask > 0).astype(np.uint8) * 255

        kernel = np.ones((5, 5), np.uint8)
        raw_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, kernel)

        return raw_mask

    def _detect_dark_objects_raw_in_lane(self, img, lane_mask_bird):
        """
        Detect black/dark unknown obstacles, like a tire, in RAW camera coordinates,
        but only inside the raw version of the detected lane mask.
        """
        raw_lane_mask = self._bird_lane_mask_to_raw_mask(lane_mask_bird, img.shape)

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        v = hsv[:, :, 2]

        dark_v_max = int(getattr(self, "yolo_bird_dark_v_max", 95))

        # Important: do NOT require saturation here.
        # Black rubber can have very low saturation, so requiring S > 15 can miss it.
        dark_mask = (v < dark_v_max).astype(np.uint8) * 255

        # Only keep dark pixels inside the lane.
        dark_mask = cv2.bitwise_and(dark_mask, raw_lane_mask)

        kernel = np.ones((3, 3), np.uint8)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            dark_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        min_area = float(getattr(self, "yolo_bird_dark_min_area", 120))
        min_w = int(getattr(self, "yolo_bird_dark_min_width_px", 8))
        min_h = int(getattr(self, "yolo_bird_dark_min_height_px", 8))

        candidates = []

        raw_h, raw_w = img.shape[:2]
        image_center_x = raw_w * 0.5

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(c)

            if bw < min_w or bh < min_h:
                continue

            # Prefer things lower in the image and closer to the center of the lane.
            cx = x + bw * 0.5
            cy = y + bh
            score = area + 2.0 * cy - 0.5 * abs(cx - image_center_x)

            candidates.append({
                "bbox": (int(x), int(y), int(x + bw), int(y + bh)),
                "area": area,
                "score": float(score),
                "source": "DARK",
                "conf": 1.00,
            })

        return sorted(candidates, key=lambda q: q["score"], reverse=True)


    def _detect_dark_objects_raw(self, img):
        """
        Detect dark/black objects in the RAW camera image.
        This is for unknown obstacles like a black wheel.

        Height will be computed from this raw bbox + depth, not from bird's-eye.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        v = hsv[:, :, 2]
        sat = hsv[:, :, 1]

        dark_v_max = int(getattr(self, "yolo_bird_dark_v_max", 70))

        # Dark + slight saturation to avoid plain gray floor as much as possible.
        mask = ((v < dark_v_max) & (sat > 15)).astype(np.uint8) * 255

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        min_area = float(getattr(self, "yolo_bird_dark_min_area", 350))
        min_w = int(getattr(self, "yolo_bird_dark_min_width_px", 15))
        min_h = int(getattr(self, "yolo_bird_dark_min_height_px", 15))

        candidates = []

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(c)

            if bw < min_w:
                continue

            if bh < min_h:
                continue

            candidates.append({
                "bbox": (int(x), int(y), int(x + bw), int(y + bh)),
                "area": area,
                "source": "DARK",
                "conf": 1.00,
            })

        return sorted(candidates, key=lambda q: q["area"], reverse=True)

    def _raw_bbox_base_to_bird_polygon(self, bbox, raw_w, raw_h):
        """
        Project only the bottom/base strip of a raw bbox into bird's-eye view.
        This avoids huge vertically stretched bird's-eye boxes.
        """
        x1, y1, x2, y2 = bbox

        box_h = max(1, y2 - y1)
        base_frac = max(0.05, min(0.60, float(getattr(self, "yolo_bird_base_frac", 0.20))))
        by1 = int(y2 - base_frac * box_h)

        bird_base = self._raw_points_to_bird(
            [
                (x1, by1),
                (x2, by1),
                (x2, y2),
                (x1, y2),
            ],
            raw_w,
            raw_h,
        )

        return bird_base



    def _depth_points_from_pixels(self, xs, ys, zs_m, image_shape):
        """
        Convert depth pixels to approximate 3D camera points.

        x = horizontal camera coordinate
        y = vertical camera coordinate
        z = forward depth
        """
        h, w = image_shape[:2]

        fx = max(1.0, float(getattr(self, "camera_fx_px", 280.45)))
        fy = max(1.0, float(getattr(self, "camera_fy_px", 280.47)))
        # ---- Kanishk's Changes here ----
        # cx = w * 0.5
        # cy = h * 0.5
        cx = float(getattr(self, "camera_cx_px", w * 0.5))
        cy = float(getattr(self, "camera_cy_px", h * 0.5))

        X = (xs.astype(np.float32) - cx) * zs_m / fx
        Y = (ys.astype(np.float32) - cy) * zs_m / fy
        Z = zs_m

        return np.stack([X, Y, Z], axis=1).astype(np.float32)

    def _estimate_height_from_depth_plane(self, depth_img, raw_bbox, raw_lane_mask, image_shape):
        """
        Estimate obstacle height from depth relative to nearby floor plane.

        This is better than:
            height = bbox_pixel_height * distance / fy

        because it measures how far object depth points stick up from the local
        ground plane.
        """
        if depth_img is None:
            return None, None

        raw_h, raw_w = image_shape[:2]

        depth = depth_img
        if depth.shape[0] != raw_h or depth.shape[1] != raw_w:
            depth = cv2.resize(depth, (raw_w, raw_h), interpolation=cv2.INTER_NEAREST)

        x1, y1, x2, y2 = [int(v) for v in raw_bbox]

        x1 = max(0, min(raw_w - 1, x1))
        x2 = max(0, min(raw_w - 1, x2))
        y1 = max(0, min(raw_h - 1, y1))
        y2 = max(0, min(raw_h - 1, y2))

        if x2 <= x1 or y2 <= y1:
            return None, None

        min_mm = float(getattr(self, "yolo_bird_depth_min_mm", 100))
        max_mm = float(getattr(self, "yolo_bird_depth_max_mm", 5000))

        # -----------------------------
        # Object points inside raw bbox.
        # -----------------------------
        obj_depth = depth[y1:y2, x1:x2].astype(np.float32)

        obj_valid = (obj_depth >= min_mm) & (obj_depth <= max_mm)

        if np.count_nonzero(obj_valid) < int(getattr(self, "VIEW4_MIN_OBJECT_POINTS", 20)):
            return None, None

        obj_depth_m_all = obj_depth[obj_valid] / 1000.0
        distance_m = float(np.median(obj_depth_m_all))

        # Keep pixels close to median object depth to reduce background leakage.
        depth_tol_m = float(getattr(self, "VIEW4_OBJECT_DEPTH_TOL_M", 0.30))
        obj_valid = obj_valid & (np.abs((obj_depth / 1000.0) - distance_m) <= depth_tol_m)

        obj_ys_local, obj_xs_local = np.where(obj_valid)

        if len(obj_xs_local) < int(getattr(self, "VIEW4_MIN_OBJECT_POINTS", 20)):
            return distance_m, None

        obj_xs = obj_xs_local + x1
        obj_ys = obj_ys_local + y1
        obj_zs_m = depth[obj_ys, obj_xs].astype(np.float32) / 1000.0

        # -----------------------------
        # Nearby ground points.
        # Use lane pixels around object, but exclude object bbox.
        # -----------------------------
        pad = int(getattr(self, "VIEW4_GROUND_SAMPLE_PAD_PX", 55))

        gx1 = max(0, x1 - pad)
        gx2 = min(raw_w - 1, x2 + pad)
        gy1 = max(0, y1 - pad)
        gy2 = min(raw_h - 1, y2 + pad)

        ground_depth = depth[gy1:gy2, gx1:gx2].astype(np.float32)
        ground_lane = raw_lane_mask[gy1:gy2, gx1:gx2] > 0

        ground_valid = (
            ground_lane
            & (ground_depth >= min_mm)
            & (ground_depth <= max_mm)
        )

        # Remove the object bbox from the ground sample.
        bx1 = x1 - gx1
        bx2 = x2 - gx1
        by1 = y1 - gy1
        by2 = y2 - gy1
        ground_valid[max(0, by1):max(0, by2), max(0, bx1):max(0, bx2)] = False

        # Prefer ground below/around the object, since that is local floor.
        if np.count_nonzero(ground_valid) < int(getattr(self, "VIEW4_MIN_GROUND_POINTS", 80)):
            lower_y1 = max(0, y2)
            lower_y2 = min(raw_h - 1, y2 + 2 * pad)
            lower_depth = depth[lower_y1:lower_y2, gx1:gx2].astype(np.float32)
            lower_lane = raw_lane_mask[lower_y1:lower_y2, gx1:gx2] > 0
            lower_valid = (
                lower_lane
                & (lower_depth >= min_mm)
                & (lower_depth <= max_mm)
            )

            ground_depth = lower_depth
            ground_valid = lower_valid
            gy1 = lower_y1
            gy2 = lower_y2

        if np.count_nonzero(ground_valid) < int(getattr(self, "VIEW4_MIN_GROUND_POINTS", 80)):
            return distance_m, None

        gy_local, gx_local = np.where(ground_valid)

        ground_xs = gx_local + gx1
        ground_ys = gy_local + gy1
        ground_zs_m = depth[ground_ys, ground_xs].astype(np.float32) / 1000.0

        # Limit point count for speed.
        max_ground = 1200
        if len(ground_xs) > max_ground:
            idx = np.linspace(0, len(ground_xs) - 1, max_ground).astype(np.int32)
            ground_xs = ground_xs[idx]
            ground_ys = ground_ys[idx]
            ground_zs_m = ground_zs_m[idx]

        max_obj = 800
        if len(obj_xs) > max_obj:
            idx = np.linspace(0, len(obj_xs) - 1, max_obj).astype(np.int32)
            obj_xs = obj_xs[idx]
            obj_ys = obj_ys[idx]
            obj_zs_m = obj_zs_m[idx]

        ground_pts = self._depth_points_from_pixels(
            ground_xs,
            ground_ys,
            ground_zs_m,
            image_shape,
        )

        obj_pts = self._depth_points_from_pixels(
            obj_xs,
            obj_ys,
            obj_zs_m,
            image_shape,
        )

        # -----------------------------
        # Fit ground plane with SVD.
        # plane: n dot p + d = 0
        # -----------------------------
        centroid = np.mean(ground_pts, axis=0)
        centered = ground_pts - centroid

        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except Exception:
            return distance_m, None

        normal = vh[-1]
        normal = normal / max(1e-6, np.linalg.norm(normal))
        d = -float(np.dot(normal, centroid))

        # Camera y-axis points downward in image coordinates.
        # Make normal point roughly upward so above-ground distances are positive.
        if normal[1] > 0:
            normal = -normal
            d = -d

        signed_dist = obj_pts @ normal + d

        # Positive distance = object sticking above local floor.
        above = signed_dist[signed_dist > 0.0]

        if above.size < 5:
            return distance_m, 0.0

        height_m = float(np.percentile(above, 95))

        # Clamp impossible noise spikes.
        height_m = max(0.0, min(height_m, 1.0))

        return distance_m, height_m


    def _render_yolo_bird_obstacle_view(
        self,
        img,
        depth_img,
        debug_img,
        control_img=None,
        debug_points=None,
        roi_y_start=0,
    ):
        """
        Clean View 4:

        - Use bird's-eye lane mask to define the valid driving corridor.
        - Ignore all YOLO/color/dark detections outside the two green lanes.
        - Compute distance/height only for detections inside the lane.
        - Display only lane-valid obstacles.
        """
        out = np.copy(debug_img)

        h_bird, w_bird = out.shape[:2]
        raw_h, raw_w = img.shape[:2]

        stop_y = int(h_bird * getattr(self, "obstacle_stop_y_frac", 0.72))
        cv2.line(out, (0, stop_y), (w_bird - 1, stop_y), (255, 255, 255), 2)

        # Bird's-eye lane mask from actual detected green lane boundaries.
        lane_mask = self._build_obstacle_lane_mask(
            debug_points,
            roi_y_start,
            out.shape,
        )

        # If we do not have a real two-lane mask, do not detect obstacles.
        if lane_mask is None or np.count_nonzero(lane_mask) < 100:
            out = self._draw_view_label(out, "VIEW 4: NO VALID LANE MASK")
            self._yolo_bird_last_img = out
            self._yolo_bird_last_stop = False
            return out, False

        # Draw the accepted lane region only.
        gate_contours, _ = cv2.findContours(
            lane_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(out, gate_contours, -1, (255, 255, 0), 1)

        # Convert bird lane mask back to raw camera coordinates.
        M = self._bird_matrix(raw_w, raw_h)
        Minv = np.linalg.inv(M)

        raw_lane_mask = cv2.warpPerspective(
            lane_mask,
            Minv,
            (raw_w, raw_h),
            flags=cv2.INTER_NEAREST,
        )
        raw_lane_mask = (raw_lane_mask > 0).astype(np.uint8) * 255

        self._yolo_bird_frame_count += 1
        run_every = max(1, int(getattr(self, "yolo_bird_run_every_n", 2)))

        if (
            self._yolo_bird_last_img is not None
            and self._yolo_bird_frame_count % run_every != 0
        ):
            return self._yolo_bird_last_img, self._yolo_bird_last_stop

        raw_stop_candidate = False
        accepted_count = 0

        def raw_bbox_base_to_bird(raw_bbox):
            x1, y1, x2, y2 = raw_bbox
            box_h = max(1, y2 - y1)

            base_frac = max(
                0.05,
                min(0.60, float(getattr(self, "yolo_bird_base_frac", 0.20))),
            )
            by1 = int(y2 - base_frac * box_h)

            return self._raw_points_to_bird(
                [
                    (x1, by1),
                    (x2, by1),
                    (x2, y2),
                    (x1, y2),
                ],
                raw_w,
                raw_h,
            )

        def bird_poly_lane_overlap(poly_pts):
            poly_pts = np.asarray(poly_pts, dtype=np.int32)

            if poly_pts.shape[0] < 3:
                return 0.0

            poly_mask = np.zeros_like(lane_mask)
            cv2.fillConvexPoly(poly_mask, poly_pts, 255)

            area = float(np.count_nonzero(poly_mask))
            if area <= 1.0:
                return 0.0

            inside = cv2.bitwise_and(poly_mask, lane_mask)
            return float(np.count_nonzero(inside)) / area

        def depth_distance_lane_gated(raw_bbox):
            """
            Estimate distance using only depth pixels that are:
            - inside the raw bbox
            - inside the raw lane mask
            - valid depth range

            This avoids taking depth from people/chairs/objects outside the lane.
            """
            if depth_img is None:
                return None

            depth = depth_img

            if depth.shape[0] != raw_h or depth.shape[1] != raw_w:
                depth = cv2.resize(
                    depth,
                    (raw_w, raw_h),
                    interpolation=cv2.INTER_NEAREST,
                )

            x1, y1, x2, y2 = [int(v) for v in raw_bbox]

            x1 = max(0, min(raw_w - 1, x1))
            x2 = max(0, min(raw_w - 1, x2))
            y1 = max(0, min(raw_h - 1, y1))
            y2 = max(0, min(raw_h - 1, y2))

            if x2 <= x1 or y2 <= y1:
                return None

            # Use the central portion of the bbox to avoid edges/background.
            frac = max(
                0.10,
                min(1.00, float(getattr(self, "yolo_bird_depth_center_frac", 0.65))),
            )

            bw = x2 - x1
            bh = y2 - y1

            cx1 = int(x1 + (1.0 - frac) * 0.5 * bw)
            cx2 = int(x2 - (1.0 - frac) * 0.5 * bw)
            cy1 = int(y1 + (1.0 - frac) * 0.5 * bh)
            cy2 = int(y2 - (1.0 - frac) * 0.5 * bh)

            roi_depth = depth[cy1:cy2, cx1:cx2].astype(np.float32)
            roi_lane = raw_lane_mask[cy1:cy2, cx1:cx2] > 0

            valid = roi_depth[
                roi_lane
                & (roi_depth >= float(self.yolo_bird_depth_min_mm))
                & (roi_depth <= float(self.yolo_bird_depth_max_mm))
            ]

            if valid.size < 10:
                return None

            # Median is more stable than a single pixel.
            return float(np.median(valid)) / 1000.0

        def process_candidate(raw_bbox, conf, source):
            nonlocal raw_stop_candidate, accepted_count, out

            x1, y1, x2, y2 = [int(v) for v in raw_bbox]

            x1 = max(0, min(raw_w - 1, x1))
            x2 = max(0, min(raw_w - 1, x2))
            y1 = max(0, min(raw_h - 1, y1))
            y2 = max(0, min(raw_h - 1, y2))

            if x2 <= x1 or y2 <= y1:
                return

            # Project only the object's base/footprint to bird view.
            bird_base = raw_bbox_base_to_bird((x1, y1, x2, y2))
            overlap = bird_poly_lane_overlap(bird_base)

            inside_lane = overlap >= float(getattr(self, "yolo_bird_lane_overlap_min", 0.35))

            # Critical fix: completely ignore outside-lane detections.
            if not inside_lane and bool(getattr(self, "view4_only_inside_lane", True)):
                return

            if bool(getattr(self, "VIEW4_USE_GROUND_PLANE_HEIGHT", True)):
                distance_m, height_m = self._estimate_height_from_depth_plane(
                    depth_img,
                    (x1, y1, x2, y2),
                    raw_lane_mask,
                    img.shape,
                )
            else:
                distance_m = depth_distance_lane_gated((x1, y1, x2, y2))
                _, height_m = self._estimate_object_size_from_bbox(
                    (x1, y1, x2, y2),
                    distance_m,
                )

            if distance_m is None:
                distance_m = depth_distance_lane_gated((x1, y1, x2, y2))

            if height_m is None:
                _, height_m = self._estimate_object_size_from_bbox(
                    (x1, y1, x2, y2),
                    distance_m,
                )

            foot_y = float(np.max(bird_base[:, 1]))
            close_by_stop_line = foot_y >= stop_y

            height_ok = (
                height_m is not None
                and height_m > self.car_underbelly_clearance_m
            )

            should_stop = bool(inside_lane and close_by_stop_line and height_ok)

            if should_stop:
                raw_stop_candidate = True

            accepted_count += 1

            if height_m is not None and height_m <= self.car_underbelly_clearance_m:
                status = "DRIVE OVER"
            elif not close_by_stop_line:
                status = "FAR"
            elif height_m is None:
                status = "NO HEIGHT"
            else:
                status = "STOP"

            color = (255, 0, 0) if should_stop else (255, 255, 0)

            bird_base_i = np.asarray(bird_base, dtype=np.int32)

            cv2.polylines(
                out,
                [bird_base_i.reshape((-1, 1, 2))],
                True,
                color,
                3,
            )

            cx = int(np.mean(bird_base_i[:, 0]))
            cy = int(np.max(bird_base_i[:, 1]))
            cv2.circle(out, (cx, cy), 5, color, -1)

            if distance_m is not None and height_m is not None:
                text = f"OBSTACLE {source} d={distance_m:.2f}m h={height_m:.2f}m {status}"
            elif distance_m is not None:
                text = f"OBSTACLE {source} d={distance_m:.2f}m h=? {status}"
            else:
                text = f"OBSTACLE {source} d=? h=? {status}"

            tx = int(max(5, min(out.shape[1] - 340, np.min(bird_base_i[:, 0]))))
            ty = int(max(45, min(out.shape[0] - 10, np.min(bird_base_i[:, 1]) - 8)))

            cv2.putText(
                out,
                text,
                (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # ------------------------------------------------------------
        # 1. YOLO detections, but only if their base is inside lane.
        # ------------------------------------------------------------
        try:
            model = self._get_yolo_bird_model()

            if model is not None:
                bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                result = model.predict(
                    source=bgr,
                    imgsz=self.yolo_bird_imgsz,
                    conf=self.yolo_bird_conf,
                    device=self.yolo_bird_device,
                    verbose=False,
                )[0]

                names = result.names

                if result.boxes is not None:
                    for box in result.boxes:
                        cls_id = int(box.cls[0])
                        raw_label = str(names.get(cls_id, cls_id))
                        conf = float(box.conf[0])

                        if self.yolo_bird_target_classes and raw_label not in self.yolo_bird_target_classes:
                            continue

                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                        process_candidate((x1, y1, x2, y2), conf, "YOLO")

        except Exception as e:
            cv2.putText(
                out,
                f"YOLO view4 error: {str(e)[:70]}",
                (8, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # ------------------------------------------------------------
        # 2. Red/orange fallback, also lane-gated.
        # ------------------------------------------------------------
        if getattr(self, "yolo_bird_raw_color_fallback", True):
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

            mask1 = cv2.inRange(hsv, self.obstacle_hsv_low1, self.obstacle_hsv_high1)
            mask2 = cv2.inRange(hsv, self.obstacle_hsv_low2, self.obstacle_hsv_high2)
            color_mask = cv2.bitwise_or(mask1, mask2)

            color_mask = cv2.bitwise_and(color_mask, raw_lane_mask)

            kernel = np.ones((5, 5), np.uint8)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                color_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            color_candidates = []

            for c in contours:
                area = float(cv2.contourArea(c))
                if area < self.obstacle_min_area:
                    continue

                x, y, bw, bh = cv2.boundingRect(c)

                if bw < self.obstacle_min_bbox_width_px:
                    continue

                if bh < self.obstacle_min_bbox_height_px:
                    continue

                score = area + 2.0 * (y + bh)
                color_candidates.append((score, (x, y, x + bw, y + bh)))

            for _, bbox in sorted(color_candidates, reverse=True)[:5]:
                process_candidate(bbox, 1.00, "COLOR")

        # ------------------------------------------------------------
        # 3. Dark fallback for black tire, also lane-gated.
        # ------------------------------------------------------------
        if getattr(self, "yolo_bird_raw_dark_fallback", True):
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            v = hsv[:, :, 2]

            dark_v_max = int(getattr(self, "yolo_bird_dark_v_max", 95))
            dark_full_mask = (v < dark_v_max).astype(np.uint8) * 255

            # Do not clip the object by the lane mask before contouring.
            # Build the bbox from the full dark object, then gate by whether
            # its base touches the lane.
            kernel = np.ones((3, 3), np.uint8)
            dark_mask = cv2.morphologyEx(dark_full_mask, cv2.MORPH_OPEN, kernel)
            dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                dark_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            min_area = float(getattr(self, "yolo_bird_dark_min_area", 120))
            min_w = int(getattr(self, "yolo_bird_dark_min_width_px", 8))
            min_h = int(getattr(self, "yolo_bird_dark_min_height_px", 8))
            max_dark = int(getattr(self, "yolo_bird_max_dark_candidates", 8))

            dark_candidates = []

            for c in contours:
                area = float(cv2.contourArea(c))
                if area < min_area:
                    continue

                x, y, bw, bh = cv2.boundingRect(c)

                if bw < min_w or bh < min_h:
                    continue

                # Gate by the bottom/base of the full dark contour.
                # This avoids losing the top of the tire while still requiring
                # the tire to be in the lane.
                base_y0 = int(y + 0.60 * bh)
                base_y1 = int(y + bh)
                base_x0 = int(x)
                base_x1 = int(x + bw)

                base_lane = raw_lane_mask[
                    max(0, base_y0):min(raw_h, base_y1),
                    max(0, base_x0):min(raw_w, base_x1),
                ]

                if base_lane.size == 0:
                    continue

                # Require at least a few bottom pixels touching the lane.
                if np.count_nonzero(base_lane) < 5:
                    continue

                cx = x + bw * 0.5
                cy = y + bh

                # Prefer closer/lower and centered objects.
                score = area + 2.0 * cy - 0.5 * abs(cx - raw_w * 0.5)

                dark_candidates.append((score, (x, y, x + bw, y + bh)))

            for _, bbox in sorted(dark_candidates, reverse=True)[:max_dark]:
                process_candidate(bbox, 1.00, "DARK")

        # Confirmation behavior.
        if raw_stop_candidate:
            self.yolo_bird_seen_count += 1
        else:
            self.yolo_bird_seen_count = 0

        stop_now = self.yolo_bird_seen_count >= self.obstacle_confirm_frames

        if stop_now:
            header = f"VIEW 4: LANE-GATED OBSTACLE ONLY - STOP ({accepted_count})"
        else:
            header = f"VIEW 4: LANE-GATED OBSTACLE ONLY ({accepted_count})"

        out = self._draw_view_label(out, header)

        self._yolo_bird_last_img = out
        self._yolo_bird_last_stop = stop_now

        return out, stop_now



    def _select_display_image(self, img, depth_img, debug_img):
        mode = getattr(self, "view_mode", "lane")

        if mode == "raw":
            return self._draw_view_label(img, "VIEW 1: RAW RGB CAMERA")

        if mode == "lane":
            return self._draw_view_label(
                debug_img,
                "VIEW 2: BIRD'S-EYE LANE + ORIGINAL OBSTACLE STOP",
            )

        if mode == "obstacle":
            return self._render_yolo_raw_view(img, depth_img)

        if mode == "tracker":
            out, _ = self._render_yolo_bird_obstacle_view(img, depth_img, debug_img)
            return out

        if mode == "depth":
            if depth_img is not None:
                return self._render_fullscreen_depth(depth_img, img)

            no_depth = np.copy(img)
            cv2.putText(
                no_depth,
                "VIEW 5: NO DEPTH FRAME AVAILABLE",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            return no_depth

        return debug_img


    def toggle_depth_overlay(self):
        self.show_depth = not self.show_depth
        print(f"Depth overlay: {'ON' if self.show_depth else 'OFF'}")
    
    def toggle_fullscreen_depth(self):
        self.fullscreen_depth = not self.fullscreen_depth
        print(f"Fullscreen depth: {'ON' if self.fullscreen_depth else 'OFF'}")

    def _render_fullscreen_depth(self, depth_img, rgb_img):
        """
        Returns a full-size colorized depth image with a small RGB
        picture-in-picture in the bottom-left so you still know where
        the car is pointing.
        """
        h, w = rgb_img.shape[:2]

        # Colorize depth at full resolution
        depth_clipped = np.clip(depth_img, 0, 3000).astype(np.float32)
        depth_norm    = (255 - (depth_clipped / 3000.0 * 255)).astype(np.uint8)  # inverted
        depth_color   = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        depth_color   = cv2.GaussianBlur(depth_color, (5, 5), 0)
        depth_color   = cv2.resize(depth_color, (w, h), interpolation=cv2.INTER_CUBIC)  # add this

        # Distance ruler on the right edge
        for dist_mm, label in [(500,"0.5m"),(1000,"1m"),(1500,"1.5m"),(2000,"2m"),(3000,"3m")]:
            y = int(h * (1.0 - dist_mm / 3000.0))
            cv2.line(depth_color, (w - 60, y), (w - 1, y), (255, 255, 255), 1)
            cv2.putText(depth_color, label, (w - 58, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Small RGB pip in bottom-left
        pip_w = w // 5
        pip_h = int(h * pip_w / w)
        pip   = cv2.resize(cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR), (pip_w, pip_h))
        # convert back to RGB to match debug_img colour order
        pip   = cv2.cvtColor(pip, cv2.COLOR_BGR2RGB)
        depth_color = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
        depth_color[h - pip_h - 2 : h - 2, 2 : pip_w + 2] = pip
        cv2.rectangle(depth_color,
                    (1, h - pip_h - 3),
                    (pip_w + 3, h - 1),
                    (255, 255, 255), 1)
        cv2.putText(depth_color, "RGB", (4, h - pip_h - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Mode label
        cv2.putText(depth_color, "DEPTH VIEW  (blue=far  red=near)",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return depth_color

    def _choose_curve_target(self, center_points, roi_h, image_w):
        """
        Choose target x at lookahead y using interpolation.

        Improvement:
        - Interpolates the virtual centerline.
        - Uses detected lane boundaries when available.
        - Clamps the target so it stays safely inside the two lane boundaries.
        - Adds a small left-curve bias so left turns do not cut too far right.
        """
        if len(center_points) == 0:
            return None

        lookahead_y = int(roi_h * self.lookahead_y_fraction)

        points = sorted(center_points, key=lambda p: p["y"])
        ys = np.array([p["y"] for p in points], dtype=np.float32)
        xs = np.array([p["x"] for p in points], dtype=np.float32)

        if len(points) >= 2:
            target_x = float(np.interp(lookahead_y, ys, xs))
        else:
            target_x = float(xs[0])

        # Estimate curve direction:
        # negative path_delta usually means left curve,
        # positive path_delta usually means right curve.
        if len(points) >= 2:
            far_x = float(points[0]["x"])
            near_x = float(points[-1]["x"])
            path_delta = far_x - near_x
        else:
            path_delta = 0.0

        # Small correction for left turns if the car is cutting too far right.
        if path_delta < 0:
            target_x += float(self.left_curve_target_bias_px)

        # Try to get lane boundaries at the lookahead row.
        left_pts = [(p["y"], p["left_x"]) for p in points if p.get("left_x") is not None]
        right_pts = [(p["y"], p["right_x"]) for p in points if p.get("right_x") is not None]

        def interp_boundary(boundary_points):
            if len(boundary_points) == 0:
                return None

            boundary_points = sorted(boundary_points, key=lambda q: q[0])
            by = np.array([q[0] for q in boundary_points], dtype=np.float32)
            bx = np.array([q[1] for q in boundary_points], dtype=np.float32)

            if len(boundary_points) >= 2:
                return float(np.interp(lookahead_y, by, bx))

            return float(bx[0])

        left_x = interp_boundary(left_pts)
        right_x = interp_boundary(right_pts)

        # If only one boundary exists, infer the other using lane width.
        lane_width = self._get_lane_width_px(image_w)

        if left_x is not None and right_x is None:
            right_x = left_x + lane_width

        if right_x is not None and left_x is None:
            left_x = right_x - lane_width

        # Boundary guard: keep target away from either lane line.
        if left_x is not None and right_x is not None:
            if left_x > right_x:
                left_x, right_x = right_x, left_x

            safe_left = left_x + self.boundary_margin_px
            safe_right = right_x - self.boundary_margin_px

            if safe_left < safe_right:
                target_x = max(safe_left, min(safe_right, target_x))

        return int(max(0, min(image_w - 1, target_x)))


    # ============================================================
    # Debug display
    # ============================================================

    def _draw_debug(
        self,
        debug_img,
        roi_y_start,
        roi_y_end,
        mask,
        debug_points,
        raw_target_x,
        lane_center_x,
        image_center_x,
        text,
    ):
        h, w = debug_img.shape[:2]

        debug_roi = debug_img[roi_y_start:roi_y_end, :]
        debug_roi[mask > 0] = self.lane_mask_color

        # Image/car center in red.
        cv2.line(debug_img, (image_center_x, 0), (image_center_x, h), (255, 0, 0), 2)

        # ROI box.
        cv2.rectangle(debug_img, (0, roi_y_start), (w - 1, roi_y_end - 1), (0, 255, 255), 2)

        # Lookahead row.
        lookahead_y = roi_y_start + int((roi_y_end - roi_y_start) * self.lookahead_y_fraction)
        cv2.line(debug_img, (0, lookahead_y), (w - 1, lookahead_y), (255, 255, 0), 2)

        center_path = []

        for p in debug_points:
            y = roi_y_start + p["y"]

            if p["left_x"] is not None:
                cv2.circle(debug_img, (int(p["left_x"]), y), 5, (0, 255, 255), -1)

            if p["right_x"] is not None:
                cv2.circle(debug_img, (int(p["right_x"]), y), 5, (0, 255, 255), -1)

            if p["x"] is not None:
                cv2.circle(debug_img, (int(p["x"]), y), 5, (255, 0, 255), -1)
                center_path.append((int(p["x"]), y))

        if len(center_path) >= 2:
            pts = np.array(center_path, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug_img, [pts], False, (255, 0, 255), 2)

        if raw_target_x is not None:
            cv2.line(
                debug_img,
                (raw_target_x, roi_y_start),
                (raw_target_x, roi_y_end),
                (255, 255, 0),
                1,
            )

        if lane_center_x is not None:
            cv2.line(
                debug_img,
                (lane_center_x, roi_y_start),
                (lane_center_x, roi_y_end),
                (255, 0, 255),
                2,
            )
            cv2.circle(debug_img, (lane_center_x, lookahead_y), 8, (255, 0, 255), -1)

        cv2.putText(
            debug_img,
            text,
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    def _overlay_depth(self, debug_img, depth_img):
        h, w = debug_img.shape[:2]

        # Clip to 3 m — plenty for a track, avoids washed-out far values

        depth_clipped = np.clip(depth_img, 0, 3000).astype(np.float32)
        depth_norm = (255 - (depth_clipped / 3000.0 * 255)).astype(np.uint8)  # inverted
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        depth_color = cv2.GaussianBlur(depth_color, (5, 5), 0)

        # Thumbnail: 1/4 of frame width, top-right corner
        thumb_w = w // 4
        thumb_h = int(depth_color.shape[0] * thumb_w / depth_color.shape[1])
        thumb = cv2.resize(depth_color, (thumb_w, thumb_h))

        x_off = w - thumb_w - 2
        y_off = 2
        debug_img[y_off:y_off + thumb_h, x_off:x_off + thumb_w] = thumb
        cv2.rectangle(debug_img,
                    (x_off - 1, y_off - 1),
                    (x_off + thumb_w, y_off + thumb_h),
                    (255, 255, 255), 1)
        cv2.putText(debug_img, "depth (blue=near)",
                    (x_off, y_off + thumb_h + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)


    def _lane_bounds_at_y(self, debug_points, roi_y_start, image_w, lane_center_x, y_img):
        """
        Estimate the left/right lane boundaries at a particular image y-position.

        This lets obstacle detection only count objects that are actually
        between the two detected lane boundaries.
        """
        lane_width = self._get_lane_width_px(image_w)

        left_pts = []
        right_pts = []
        center_pts = []

        if debug_points is not None:
            for p in debug_points:
                if p.get("y") is None:
                    continue

                yy = int(roi_y_start + p["y"])

                if p.get("left_x") is not None:
                    left_pts.append((yy, float(p["left_x"])))

                if p.get("right_x") is not None:
                    right_pts.append((yy, float(p["right_x"])))

                if p.get("x") is not None:
                    center_pts.append((yy, float(p["x"])))

        def interp_points(points):
            if len(points) == 0:
                return None

            points = sorted(points, key=lambda q: q[0])
            ys = np.array([q[0] for q in points], dtype=np.float32)
            xs = np.array([q[1] for q in points], dtype=np.float32)

            if len(points) == 1:
                return float(xs[0])

            return float(np.interp(y_img, ys, xs))

        left_x = interp_points(left_pts)
        right_x = interp_points(right_pts)
        center_x = interp_points(center_pts)

        if center_x is None:
            center_x = float(lane_center_x)

        # If both detected boundaries exist, use them directly.
        if left_x is not None and right_x is not None:
            pass

        # If only left boundary exists, infer right boundary from lane width.
        elif left_x is not None:
            right_x = left_x + lane_width

        # If only right boundary exists, infer left boundary from lane width.
        elif right_x is not None:
            left_x = right_x - lane_width

        # If neither boundary exists, fallback to center +/- half lane width.
        else:
            left_x = center_x - lane_width / 2.0
            right_x = center_x + lane_width / 2.0

        left_x = max(0.0, min(float(image_w - 1), left_x))
        right_x = max(0.0, min(float(image_w - 1), right_x))

        if left_x > right_x:
            left_x, right_x = right_x, left_x

        return left_x, right_x


    def _build_obstacle_lane_mask(self, debug_points, roi_y_start, image_shape):
        """
        Build a binary mask for the actual drivable lane interior.

        This uses only rows where BOTH left and right green lane boundaries
        were detected. This prevents outside-lane objects from being accepted
        through the old center-corridor fallback.
        """
        h, w = image_shape[:2]
        lane_mask = np.zeros((h, w), dtype=np.uint8)

        if debug_points is None:
            return lane_mask

        samples = []

        for pnt in debug_points:
            if pnt.get("y") is None:
                continue

            left_x = pnt.get("left_x")
            right_x = pnt.get("right_x")

            # Strict mode: only trust actual two-lane detections.
            if left_x is None or right_x is None:
                continue

            y = int(roi_y_start + pnt["y"])
            left_x = float(left_x)
            right_x = float(right_x)

            if left_x > right_x:
                left_x, right_x = right_x, left_x

            samples.append((y, left_x, right_x))

        # If we do not have enough real two-lane points, do not detect obstacles.
        # This avoids false stops when the lane itself is uncertain.
        if len(samples) < 2:
            return lane_mask

        samples = sorted(samples, key=lambda q: q[0])

        ys = np.array([q[0] for q in samples], dtype=np.float32)
        lefts = np.array([q[1] for q in samples], dtype=np.float32)
        rights = np.array([q[2] for q in samples], dtype=np.float32)

        y_min = max(0, int(np.min(ys)))
        y_max = min(h - 1, int(np.max(ys)))

        margin = int(getattr(self, "obstacle_lane_mask_margin_px", 18))

        for y in range(y_min, y_max + 1):
            lx = float(np.interp(y, ys, lefts))
            rx = float(np.interp(y, ys, rights))

            x0 = int(max(0, min(w - 1, lx + margin)))
            x1 = int(max(0, min(w - 1, rx - margin)))

            if x1 > x0:
                lane_mask[y, x0:x1] = 255

        # Keep only the configured obstacle ROI vertically.
        roi_y0 = int(h * self.obstacle_roi_y_start)
        roi_y1 = int(h * self.obstacle_roi_y_end)

        roi_gate = np.zeros_like(lane_mask)
        roi_gate[max(0, roi_y0):min(h, roi_y1), :] = 255
        lane_mask = cv2.bitwise_and(lane_mask, roi_gate)

        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

        return lane_mask

    def _detect_obstacle_inside_lane_mask(self, control_img, lane_mask):
        """
        Detect red/orange/dark obstacle pixels ONLY inside the actual lane mask.
        """
        if not self.obstacle_enable:
            return []

        hsv = cv2.cvtColor(control_img, cv2.COLOR_RGB2HSV)

        mask1 = cv2.inRange(hsv, self.obstacle_hsv_low1, self.obstacle_hsv_high1)
        mask2 = cv2.inRange(hsv, self.obstacle_hsv_low2, self.obstacle_hsv_high2)
        obstacle_mask = cv2.bitwise_or(mask1, mask2)

        dark_v_max = getattr(self, "obstacle_dark_v_max", None)
        if dark_v_max is not None:
            v = hsv[:, :, 2]
            dark_mask = (v < int(dark_v_max)).astype(np.uint8) * 255
            obstacle_mask = cv2.bitwise_or(obstacle_mask, dark_mask)

        # Main fix: remove anything outside the lane.
        obstacle_mask = cv2.bitwise_and(obstacle_mask, lane_mask)

        kernel = np.ones((5, 5), np.uint8)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            obstacle_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []

        for c in contours:
            area = float(cv2.contourArea(c))

            if area < self.obstacle_min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(c)

            # Reject flat/tiny floor-colored patches.
            if bw < self.obstacle_min_bbox_width_px:
                continue

            if bh < self.obstacle_min_bbox_height_px:
                continue

            candidates.append({
                "bbox": (int(x), int(y), int(bw), int(bh)),
                "area": area,
            })

        candidates = sorted(candidates, key=lambda q: q["area"], reverse=True)
        return candidates


    def _detect_obstacle_in_lane_corridor(self, control_img, lane_center_x):
        """
        Detect candidate red/orange obstacles near the lane center corridor.

        This function only finds candidates. The actual lane-boundary check
        happens in _apply_obstacle_stop().
        """
        if not self.obstacle_enable:
            return []

        h, w = control_img.shape[:2]

        y0 = int(h * self.obstacle_roi_y_start)
        y1 = int(h * self.obstacle_roi_y_end)

        cx = int(lane_center_x)
        half = int(self.obstacle_corridor_half_width_px)

        x0 = max(0, cx - half)
        x1 = min(w, cx + half)

        y0 = max(0, min(h - 1, y0))
        y1 = max(y0 + 1, min(h, y1))

        if x1 <= x0 or y1 <= y0:
            return []

        roi = control_img[y0:y1, x0:x1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        mask1 = cv2.inRange(hsv, self.obstacle_hsv_low1, self.obstacle_hsv_high1)
        mask2 = cv2.inRange(hsv, self.obstacle_hsv_low2, self.obstacle_hsv_high2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Optional dark object support if this config exists.
        dark_v_max = getattr(self, "obstacle_dark_v_max", None)
        if dark_v_max is not None:
            v = hsv[:, :, 2]
            dark_mask = (v < int(dark_v_max)).astype(np.uint8) * 255
            mask = cv2.bitwise_or(mask, dark_mask)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates = []

        for c in contours:
            area = float(cv2.contourArea(c))

            if area < self.obstacle_min_area:
                continue

            bx, by, bw, bh = cv2.boundingRect(c)

            full_x = x0 + bx
            full_y = y0 + by

            candidates.append({
                "bbox": (int(full_x), int(full_y), int(bw), int(bh)),
                "area": area,
            })

        candidates = sorted(candidates, key=lambda q: q["area"], reverse=True)
        return candidates

    def _apply_obstacle_stop(
        self,
        control_img,
        debug_img,
        lane_center_x,
        steering,
        throttle,
        debug_points=None,
        roi_y_start=0,
    ):
        """
        Strict lane-gated obstacle stop.

        Instead of searching a wide center corridor, this builds a mask from
        the actual detected left/right green lane boundaries and detects
        obstacles only inside that lane interior.
        """
        h, w = debug_img.shape[:2]

        stop_y = int(h * getattr(self, "obstacle_stop_y_frac", 0.72))
        cv2.line(debug_img, (0, stop_y), (w - 1, stop_y), (255, 255, 255), 2)

        lane_mask = self._build_obstacle_lane_mask(
            debug_points,
            roi_y_start,
            debug_img.shape,
        )

        # Draw the actual obstacle gate in yellow so you can see what counts.
        gate_contours, _ = cv2.findContours(
            lane_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(debug_img, gate_contours, -1, (255, 255, 0), 1)

        candidates = self._detect_obstacle_inside_lane_mask(
            control_img,
            lane_mask,
        )

        if len(candidates) == 0:
            self.obstacle_seen_count = 0
            return steering, throttle

        selected = candidates[0]
        x, y, bw, bh = selected["bbox"]

        foot_x = int(x + bw / 2)
        foot_y = int(y + bh)

        # Because the mask already removed outside-lane pixels, any remaining
        # contour is inside the detected lane.
        cv2.rectangle(
            debug_img,
            (x, y),
            (x + bw, y + bh),
            (255, 0, 0),
            3,
        )
        cv2.circle(debug_img, (foot_x, foot_y), 5, (255, 255, 255), -1)

        close_enough = foot_y >= stop_y

        if close_enough:
            self.obstacle_seen_count += 1
        else:
            self.obstacle_seen_count = 0

        if self.obstacle_seen_count >= self.obstacle_confirm_frames:
            cv2.putText(
                debug_img,
                "OBSTACLE - STOP",
                (x, max(25, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            steering = self.obstacle_stop_steering
            throttle = self.obstacle_stop_throttle

        else:
            cv2.putText(
                debug_img,
                "OBSTACLE",
                (x, max(25, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return steering, throttle



    # ============================================================
    # DonkeyCar run loop
    # ============================================================

    def run(self, img, depth_img = None):
        if img is None:
            return 0.0, 0.0, None

        steering = 0.0
        throttle = 0.0

        if self.use_birdseye:
            control_img = self._birdseye_transform(img)
            debug_img = np.copy(control_img)
            mode = "bird"
        else:
            control_img = img
            debug_img = np.copy(img)
            mode = "raw"

        h, w = control_img.shape[:2]
        image_center_x = w // 2

        roi_y_start = int(h * self.roi_y_start_frac)
        roi_y_end = int(h * self.roi_y_end_frac)
        roi_y_end = min(roi_y_end, h)

        roi = control_img[roi_y_start:roi_y_end, :]
        roi_h = roi.shape[0]

        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lane_hsv_low, self.lane_hsv_high)
        mask = self._clean_mask(mask)

        center_points, debug_points = self._extract_center_points(mask, image_center_x)
        raw_target_x = self._choose_curve_target(center_points, roi_h, w)

        if raw_target_x is not None:
            # ------------------------------------------------------------
            # Straight recovery logic
            # ------------------------------------------------------------
            # Estimate whether the detected center path is currently straight.
            # center_points are sorted top-to-bottom, so:
            #   far_x  = upper/far path center
            #   near_x = lower/near path center
            if len(center_points) >= 2:
                pts_for_shape = sorted(center_points, key=lambda p: p["y"])
                far_x = int(pts_for_shape[0]["x"])
                near_x = int(pts_for_shape[-1]["x"])
                path_delta = far_x - near_x
            else:
                path_delta = 0

            raw_error = raw_target_x - image_center_x

            is_straight = (
                abs(path_delta) <= self.straight_path_delta_px
                and abs(raw_error) <= self.straight_error_px
            )

            # ------------------------------------------------------------
            # Lane center smoothing
            # ------------------------------------------------------------
            # If we just came out of a curve and the path is straight again,
            # use a high alpha so the old curve target does not linger.
            if is_straight:
                center_alpha = self.center_recovery_alpha
            else:
                center_alpha = self.smoothing_alpha

            if self.smoothed_lane_center_x is None:
                self.smoothed_lane_center_x = raw_target_x
            else:
                self.smoothed_lane_center_x = (
                    center_alpha * raw_target_x
                    + (1.0 - center_alpha) * self.smoothed_lane_center_x
                )

            lane_center_x = int(self.smoothed_lane_center_x)
            error = lane_center_x - image_center_x

            # Ignore tiny errors on straights so the car does not wiggle.
            if abs(error) < self.steering_deadband_px:
                error = 0

            # ------------------------------------------------------------
            # Adaptive steering gain
            # ------------------------------------------------------------
            # Straight: lower gain so it stabilizes.
            # Curve: higher gain so it can still turn sharply.
            if is_straight:
                active_gain = self.straight_steering_gain
            else:
                active_gain = self.curve_steering_gain

            # Combine lateral correction with heading/curve correction.
            # error corrects current offset from center.
            # path_delta turns into the curve earlier before the offset gets large.
            heading_error = path_delta

            raw_steering = (
                active_gain * error
                + self.heading_gain * heading_error
                + self.steering_bias
            )

            raw_steering = max(min(raw_steering, self.max_steering), -self.max_steering)

            # ------------------------------------------------------------
            # Steering output smoothing
            # ------------------------------------------------------------
            # If path is straight again, or steering needs to reverse direction,
            # recover quickly instead of slowly carrying old curve steering.
            if self.smoothed_steering is None:
                self.smoothed_steering = raw_steering
            else:
                steering_sign_changed = raw_steering * self.smoothed_steering < 0
                steering_should_decay = abs(raw_steering) < 0.55 * abs(self.smoothed_steering)

                if is_straight or steering_sign_changed or steering_should_decay:
                    steering_alpha = self.steering_recovery_alpha
                else:
                    steering_alpha = self.steering_output_alpha

                self.smoothed_steering = (
                    steering_alpha * raw_steering
                    + (1.0 - steering_alpha) * self.smoothed_steering
                )

            steering = self.smoothed_steering

            # Keep normal speed on straights, but slow slightly in sharper curves.
            if (not is_straight) and abs(steering) >= self.turn_slowdown_steer_threshold:
                throttle = self.turn_throttle
            else:
                throttle = self.test_throttle

            drive_mode = "straight" if is_straight else "curve"

            text = (
                f"{mode} {drive_mode} | err:{error} d:{path_delta} "
                f"gain:{active_gain:.3f} steer:{steering:.2f} thr:{throttle:.2f} "
                f"bands:{len(center_points)}"
            )

            self._draw_debug(
                debug_img,
                roi_y_start,
                roi_y_end,
                mask,
                debug_points,
                raw_target_x,
                lane_center_x,
                image_center_x,
                text,
            )

        else:
            # Reset both smoothing states when the lane is lost
            self.smoothed_lane_center_x = None
            self.smoothed_steering = None

            self._draw_debug(
                debug_img,
                roi_y_start,
                roi_y_end,
                mask,
                debug_points,
                None,
                None,
                image_center_x,
                f"{mode} missing lane boundary - stopped",
            )

        # Obstacle stop layer:
        # lane following runs first, then obstacle detection can override throttle to 0.
        if raw_target_x is not None and getattr(self, "view_mode", "lane") != "tracker":
            steering, throttle = self._apply_obstacle_stop(
                control_img,
                debug_img,
                lane_center_x,
                steering,
                throttle,
                debug_points,
                roi_y_start,
            )

        if self.fullscreen_depth and depth_img is not None:
            out_img = self._render_fullscreen_depth(depth_img, img)
            return steering, throttle, out_img
        
        if depth_img is not None and self.show_depth:
            self._overlay_depth(debug_img, depth_img)

        if getattr(self, "view_mode", "lane") == "tracker":
            display_img, yolo_stop_now = self._render_yolo_bird_obstacle_view(
                img,
                depth_img,
                debug_img,
                control_img,
                debug_points,
                roi_y_start,
            )

            if yolo_stop_now:
                steering = self.yolo_bird_stop_steering
                throttle = self.yolo_bird_stop_throttle

            return steering, throttle, display_img

        display_img = self._select_display_image(img, depth_img, debug_img)
        return steering, throttle, display_img
