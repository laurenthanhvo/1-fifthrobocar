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
        Stop only if the obstacle bbox is inside the two detected lane boundaries.
        Obstacles outside the green lanes are ignored.
        """
        candidates = self._detect_obstacle_in_lane_corridor(
            control_img,
            lane_center_x,
        )

        h, w = debug_img.shape[:2]

        # Draw the broad search corridor in orange.
        cx = int(lane_center_x)
        half = int(self.obstacle_corridor_half_width_px)
        search_x0 = max(0, cx - half)
        search_x1 = min(w, cx + half)
        search_y0 = int(h * self.obstacle_roi_y_start)
        search_y1 = int(h * self.obstacle_roi_y_end)

        cv2.rectangle(
            debug_img,
            (search_x0, search_y0),
            (search_x1, search_y1),
            (255, 128, 0),
            1,
        )

        stop_y = int(h * getattr(self, "obstacle_stop_y_frac", 0.72))
        cv2.line(debug_img, (0, stop_y), (w - 1, stop_y), (255, 255, 255), 2)

        lane_margin = int(getattr(self, "obstacle_lane_margin_px", 10))
        draw_ignored = bool(getattr(self, "obstacle_draw_ignored", False))

        selected = None

        for cand in candidates:
            x, y, bw, bh = cand["bbox"]

            # Use the bottom center of the bbox as the object's ground contact point.
            foot_x = int(x + bw / 2)
            foot_y = int(y + bh)

            left_bound, right_bound = self._lane_bounds_at_y(
                debug_points,
                roi_y_start,
                w,
                lane_center_x,
                foot_y,
            )

            inside_lanes = (
                foot_x >= left_bound + lane_margin
                and foot_x <= right_bound - lane_margin
            )

            if inside_lanes:
                selected = {
                    "bbox": cand["bbox"],
                    "area": cand["area"],
                    "left_bound": left_bound,
                    "right_bound": right_bound,
                    "foot_x": foot_x,
                    "foot_y": foot_y,
                }
                break

            elif draw_ignored:
                # Optional: draw ignored outside-lane obstacles in gray.
                cv2.rectangle(
                    debug_img,
                    (x, y),
                    (x + bw, y + bh),
                    (120, 120, 120),
                    1,
                )
                cv2.putText(
                    debug_img,
                    "ignored outside lane",
                    (x, max(20, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (180, 180, 180),
                    1,
                    cv2.LINE_AA,
                )

        if selected is None:
            self.obstacle_seen_count = 0
            return steering, throttle

        x, y, bw, bh = selected["bbox"]
        foot_y = selected["foot_y"]

        # Draw the detected obstacle.
        cv2.rectangle(
            debug_img,
            (x, y),
            (x + bw, y + bh),
            (255, 0, 0),
            3,
        )

        # Draw the accepted lane bounds at the obstacle bottom position.
        lb = int(selected["left_bound"])
        rb = int(selected["right_bound"])
        cv2.line(debug_img, (lb, foot_y), (rb, foot_y), (0, 255, 255), 2)

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

    def run(self, img):
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
        if raw_target_x is not None:
            steering, throttle = self._apply_obstacle_stop(
                control_img,
                debug_img,
                lane_center_x,
                steering,
                throttle,
                debug_points,
                roi_y_start,
            )

        return steering, throttle, debug_img
