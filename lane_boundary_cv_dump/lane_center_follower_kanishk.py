import cv2
import numpy as np


class LaneCenterFollower:
    def __init__(self, pid, cfg):
        self.pid_st = pid
        self.overlay_image = getattr(cfg, "OVERLAY_IMAGE", True)

        # ============================================================
        # ROI / view settings
        # ============================================================
        self.roi_y_start_frac = getattr(cfg, "LANE_ROI_Y_START", 0.40)
        self.roi_y_end_frac = getattr(cfg, "LANE_ROI_Y_END", 1.00)

        # Farther lookahead helps curves.
        # Smaller = farther ahead, larger = closer to car.
        # CHANGED: 0.52 -> 0.28 to look further ahead and anticipate curves earlier.
        self.lookahead_y_fraction = getattr(cfg, "LANE_LOOKAHEAD_Y_FRACTION", 0.28)

        # Near point is used to estimate current path heading.
        self.near_y_fraction = getattr(cfg, "LANE_NEAR_Y_FRACTION", 0.88)

        self.num_bands = getattr(cfg, "LANE_NUM_BANDS", 14)

        # Ignore distorted edges.
        self.search_x_start_frac = getattr(cfg, "LANE_SEARCH_X_START_FRAC", 0.08)
        self.search_x_end_frac = getattr(cfg, "LANE_SEARCH_X_END_FRAC", 0.92)

        # ============================================================
        # Lane color detection
        # ============================================================
        # Main lane color mode.
        # Use "yellow" for yellow tape, "blue" for blue tape, "white" for white tape,
        # or "all" if you want all masks combined.
        self.color_mode = getattr(cfg, "LANE_COLOR_MODE", "yellow")

        # Yellow tape.
        self.yellow_h_low = getattr(cfg, "LANE_YELLOW_H_LOW", 15)
        self.yellow_h_high = getattr(cfg, "LANE_YELLOW_H_HIGH", 38)
        self.yellow_s_min = getattr(cfg, "LANE_YELLOW_S_MIN", 70)
        self.yellow_v_min = getattr(cfg, "LANE_YELLOW_V_MIN", 70)

        # White tape / washed-out tape.
        self.white_s_max = getattr(cfg, "LANE_WHITE_S_MAX", 75)
        self.white_v_min = getattr(cfg, "LANE_WHITE_V_MIN", 150)

        # Blue tape.
        self.blue_h_low = getattr(cfg, "LANE_BLUE_H_LOW", 80)
        self.blue_h_high = getattr(cfg, "LANE_BLUE_H_HIGH", 155)
        self.blue_s_min = getattr(cfg, "LANE_BLUE_S_MIN", 20)
        self.blue_v_min = getattr(cfg, "LANE_BLUE_V_MIN", 25)

        # Optional lighting enhancement.
        self.use_clahe = getattr(cfg, "LANE_USE_CLAHE", False)
        self.clahe_clip_limit = getattr(cfg, "LANE_CLAHE_CLIP_LIMIT", 2.0)
        self.clahe_tile_grid_size = getattr(cfg, "LANE_CLAHE_TILE_GRID_SIZE", 8)

        # ============================================================
        # Noise filtering
        # ============================================================
        self.kernel_size = getattr(cfg, "LANE_KERNEL_SIZE", 5)

        # 500 was too high. It can delete lane segments in curves.
        self.min_component_area = getattr(cfg, "LANE_MIN_COMPONENT_AREA", 45)

        self.min_band_pixels = getattr(cfg, "LANE_MIN_BAND_PIXELS", 5)
        self.min_run_width = getattr(cfg, "LANE_MIN_RUN_WIDTH", 2)
        self.min_column_count = getattr(cfg, "LANE_MIN_COLUMN_COUNT", 2)

        # Reject sudden fake detections, but do not make it too strict.
        # CHANGED: 120 -> 180 to avoid rejecting valid lane positions mid-curve.
        self.max_lane_jump_px = getattr(cfg, "LANE_MAX_JUMP_PX", 180)

        # ============================================================
        # Lane width / one-line fallback
        # ============================================================
        self.expected_lane_width_px = getattr(cfg, "LANE_EXPECTED_WIDTH_PX", None)
        self.expected_lane_width_frac = getattr(cfg, "LANE_EXPECTED_WIDTH_FRAC", 0.55)
        self.learned_lane_width_px = None

        self.min_lane_width_frac = getattr(cfg, "LANE_MIN_WIDTH_FRAC", 0.12)
        self.max_lane_width_frac = getattr(cfg, "LANE_MAX_WIDTH_FRAC", 0.98)

        self.allow_one_side_fallback = getattr(cfg, "LANE_ALLOW_ONE_SIDE_FALLBACK", True)

        # If only one line is visible:
        # "right_of_left" means visible line is left boundary, so drive to its right.
        # "left_of_right" means visible line is right boundary, so drive to its left.
        # "auto" decides based on whether the visible line is left/right of image center.
        self.keep_side = getattr(cfg, "LANE_KEEP_SIDE", "right_of_left")

        # ============================================================
        # Steering / throttle
        # ============================================================
        self.steering_gain = getattr(cfg, "LANE_STEERING_GAIN", 0.035)

        # This is the important curve term.
        # It makes the car turn into the curve using far-vs-near path direction.
        # CHANGED: 0.060 -> 0.15 so the heading term meaningfully steers into curves.
        self.heading_gain = getattr(cfg, "LANE_HEADING_GAIN", 0.15)

        self.max_steering = getattr(cfg, "LANE_MAX_STEERING", 0.85)
        self.steering_bias = getattr(cfg, "LANE_STEERING_BIAS", 0.0)
        self.invert_steering = getattr(cfg, "LANE_INVERT_STEERING", False)

        # CHANGED: 0.45 -> 0.70 so the smoothed target tracks lateral shifts faster
        # in curves instead of lagging behind.
        self.smoothing_alpha = getattr(cfg, "LANE_SMOOTHING_ALPHA", 0.70)
        self.smoothed_target_x = None

        self.throttle_straight = getattr(cfg, "LANE_THROTTLE_STRAIGHT", 0.45)
        self.throttle_turn = getattr(cfg, "LANE_THROTTLE_TURN", 0.30)
        self.throttle_sharp_turn = getattr(cfg, "LANE_THROTTLE_SHARP_TURN", 0.20)

        # Backwards compatibility with your old config.
        self.throttle_straight = getattr(cfg, "LANE_TEST_THROTTLE", self.throttle_straight)

        self.lane_mask_color = tuple(getattr(cfg, "LANE_MASK_COLOR", (0, 255, 0)))

        self.last_target_x = None
        self.last_near_x = None

    # ============================================================
    # Image processing
    # ============================================================

    def _enhance_lighting(self, roi):
        if not self.use_clahe:
            return roi

        lab = cv2.cvtColor(roi, cv2.COLOR_RGB2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_limit,
            tileGridSize=(self.clahe_tile_grid_size, self.clahe_tile_grid_size),
        )

        l_channel = clahe.apply(l_channel)
        enhanced_lab = cv2.merge((l_channel, a_channel, b_channel))
        return cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2RGB)

    def _clean_mask(self, mask):
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        # Remove isolated speckles.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Fill small tape gaps.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # One small dilation helps broken lane segments connect.
        mask = cv2.dilate(mask, kernel, iterations=1)

        return mask

    def _filter_components(self, mask):
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask,
            connectivity=8,
        )

        filtered = np.zeros_like(mask)

        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            width = stats[label, cv2.CC_STAT_WIDTH]
            height = stats[label, cv2.CC_STAT_HEIGHT]

            # Keep meaningful tape blobs, reject tiny noise.
            if area >= self.min_component_area and (width >= 3 or height >= 3):
                filtered[labels == label] = 255

        return filtered

    def _build_lane_mask(self, roi):
        roi_proc = self._enhance_lighting(roi)

        hsv = cv2.cvtColor(roi_proc, cv2.COLOR_RGB2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)

        yellow_mask = (
            (h_ch >= 15)
            & (h_ch <= 35)
            & (s_ch >= 80)
            & (v_ch >= 80)
        ).astype(np.uint8) * 255

        white_mask = (
            (s_ch <= 40)
            & (v_ch >= 200)
        ).astype(np.uint8) * 255

        blue_mask = (
            (h_ch >= 80)
            & (h_ch <= self.blue_h_high)
            & (s_ch >= self.blue_s_min)
            & (v_ch >= self.blue_v_min)
        ).astype(np.uint8) * 255

        if self.color_mode == "yellow":
            mask = yellow_mask
        elif self.color_mode == "white":
            mask = white_mask
        elif self.color_mode == "blue":
            mask = blue_mask
        else:
            mask = cv2.bitwise_or(yellow_mask, white_mask)
            mask = cv2.bitwise_or(mask, blue_mask)

        mask = self._clean_mask(mask)
        mask = self._filter_components(mask)

        return mask

    # ============================================================
    # Lane extraction
    # ============================================================

    def _find_runs(self, col_counts):
        active = col_counts >= self.min_column_count

        runs = []
        start = None

        for i, is_active in enumerate(active):
            if is_active and start is None:
                start = i
            elif not is_active and start is not None:
                end = i - 1
                if end - start + 1 >= self.min_run_width:
                    runs.append(
                        {
                            "start": int(start),
                            "end": int(end),
                            "center": int((start + end) // 2),
                            "score": float(np.sum(col_counts[start:end + 1])),
                        }
                    )
                start = None

        if start is not None:
            end = len(active) - 1
            if end - start + 1 >= self.min_run_width:
                runs.append(
                    {
                        "start": int(start),
                        "end": int(end),
                        "center": int((start + end) // 2),
                        "score": float(np.sum(col_counts[start:end + 1])),
                    }
                )

        return runs

    def _get_lane_width_px(self, image_width=None):
        if self.expected_lane_width_px is not None:
            return float(self.expected_lane_width_px)

        if self.learned_lane_width_px is not None:
            return float(self.learned_lane_width_px)

        if image_width is not None:
            return float(self.expected_lane_width_frac * image_width)

        return None

    def _update_lane_width(self, width_px):
        if width_px is None or width_px <= 0:
            return

        if self.learned_lane_width_px is None:
            self.learned_lane_width_px = float(width_px)
        else:
            self.learned_lane_width_px = (
                0.20 * float(width_px)
                + 0.80 * self.learned_lane_width_px
            )

    def _choose_pair_from_runs(self, runs, image_width):
        if len(runs) < 2:
            return None, None, None

        min_width = int(self.min_lane_width_frac * image_width)
        max_width = int(self.max_lane_width_frac * image_width)

        expected_width = self._get_lane_width_px(image_width)
        expected_center = self.last_target_x if self.last_target_x is not None else image_width / 2

        best = None
        best_score = -1e18

        for i in range(len(runs)):
            for j in range(i + 1, len(runs)):
                left_run = runs[i]
                right_run = runs[j]

                # Inner edges of lane boundaries.
                left_x = left_run["end"]
                right_x = right_run["start"]

                lane_width = right_x - left_x
                if lane_width < min_width or lane_width > max_width:
                    continue

                center_x = (left_x + right_x) / 2.0

                pixel_score = left_run["score"] + right_run["score"]
                width_penalty = abs(lane_width - expected_width)
                center_penalty = abs(center_x - expected_center)

                score = pixel_score - 1.0 * width_penalty - 0.8 * center_penalty

                if score > best_score:
                    best_score = score
                    best = {
                        "left_x": int(left_x),
                        "right_x": int(right_x),
                        "center_x": int(center_x),
                        "lane_width": int(lane_width),
                    }

        if best is None:
            return None, None, None

        return best["left_x"], best["right_x"], best["center_x"]

    def _choose_single_line_target(self, runs, image_width):
        if len(runs) == 0 or not self.allow_one_side_fallback:
            return None, None, None, "none"

        lane_width = self._get_lane_width_px(image_width)
        if lane_width is None:
            return None, None, None, "none"

        strongest = sorted(runs, key=lambda r: r["score"], reverse=True)[0]
        role = self.keep_side

        if role == "auto":
            if strongest["center"] < image_width / 2:
                role = "right_of_left"
            else:
                role = "left_of_right"

        if role == "right_of_left":
            # Visible line is left boundary. Stay to its right.
            left_x = strongest["end"]
            center_x = int(left_x + lane_width / 2.0)
            right_x = None
            method = "left_only"

        elif role == "left_of_right":
            # Visible line is right boundary. Stay to its left.
            right_x = strongest["start"]
            center_x = int(right_x - lane_width / 2.0)
            left_x = None
            method = "right_only"

        else:
            return None, None, None, "none"

        center_x = max(0, min(image_width - 1, center_x))
        return left_x, right_x, center_x, method

    def _extract_center_points(self, mask, image_center_x):
        h, w = mask.shape

        x_start = int(w * self.search_x_start_frac)
        x_end = int(w * self.search_x_end_frac)

        center_points = []
        debug_points = []

        for band_idx in range(self.num_bands):
            y0 = int(h * band_idx / self.num_bands)
            y1 = int(h * (band_idx + 1) / self.num_bands)
            center_y = (y0 + y1) // 2

            band = mask[y0:y1, x_start:x_end]

            if int(np.count_nonzero(band)) < self.min_band_pixels:
                continue

            col_counts = np.sum(band > 0, axis=0).astype(np.float32)

            # Smooth the histogram so broken tape becomes cleaner.
            smooth_kernel = np.ones(5, dtype=np.float32) / 5.0
            col_counts = np.convolve(col_counts, smooth_kernel, mode="same")

            local_runs = self._find_runs(col_counts)

            # Convert local x in search window back to full image x.
            runs = []
            for r in local_runs:
                runs.append(
                    {
                        "start": r["start"] + x_start,
                        "end": r["end"] + x_start,
                        "center": r["center"] + x_start,
                        "score": r["score"],
                    }
                )

            left_x, right_x, center_x = self._choose_pair_from_runs(runs, w)
            method = "both"

            if center_x is not None and left_x is not None and right_x is not None:
                self._update_lane_width(right_x - left_x)
            else:
                left_x, right_x, center_x, method = self._choose_single_line_target(runs, w)

            if center_x is None:
                debug_points.append(
                    {
                        "y": int(center_y),
                        "left_x": left_x,
                        "right_x": right_x,
                        "center_x": None,
                        "method": "none",
                    }
                )
                continue

            # Do not reject far/top bands too aggressively on curves.
            # Only reject huge jumps in lower bands where detection should be stable.
            if self.last_target_x is not None and band_idx > self.num_bands // 2:
                if abs(center_x - self.last_target_x) > self.max_lane_jump_px:
                    debug_points.append(
                        {
                            "y": int(center_y),
                            "left_x": left_x,
                            "right_x": right_x,
                            "center_x": None,
                            "method": "rejected_jump",
                        }
                    )
                    continue

            # CHANGED: Use uniform weights (1.0) instead of weighting lower/near bands
            # more heavily. The old near-band bias pulled the polynomial fit toward the
            # car, making far_x inaccurate and degrading curve anticipation. Uniform
            # weights let the quadratic fit represent the full path shape faithfully.
            weight = 1.0

            center_points.append(
                {
                    "x": int(center_x),
                    "y": int(center_y),
                    "weight": float(weight),
                    "left_x": left_x,
                    "right_x": right_x,
                    "method": method,
                }
            )

            debug_points.append(
                {
                    "y": int(center_y),
                    "left_x": left_x,
                    "right_x": right_x,
                    "center_x": int(center_x),
                    "method": method,
                }
            )

        return center_points, debug_points

    # ============================================================
    # Curve target and controller
    # ============================================================

    def _fit_curve_targets(self, center_points, roi_h, image_width):
        if len(center_points) == 0:
            return None, None, None

        far_y = int(roi_h * self.lookahead_y_fraction)
        near_y = int(roi_h * self.near_y_fraction)

        xs = np.array([p["x"] for p in center_points], dtype=np.float32)
        ys = np.array([p["y"] for p in center_points], dtype=np.float32)
        weights = np.array([p["weight"] for p in center_points], dtype=np.float32)

        try:
            if len(center_points) >= 4:
                coeffs = np.polyfit(ys, xs, deg=2, w=weights)
                far_x = int(np.polyval(coeffs, far_y))
                near_x = int(np.polyval(coeffs, near_y))
            elif len(center_points) >= 2:
                coeffs = np.polyfit(ys, xs, deg=1, w=weights)
                far_x = int(np.polyval(coeffs, far_y))
                near_x = int(np.polyval(coeffs, near_y))
            else:
                far_x = int(xs[0])
                near_x = int(xs[0])
        except Exception:
            far_point = min(center_points, key=lambda p: abs(p["y"] - far_y))
            near_point = min(center_points, key=lambda p: abs(p["y"] - near_y))
            far_x = int(far_point["x"])
            near_x = int(near_point["x"])

        far_x = max(0, min(image_width - 1, far_x))
        near_x = max(0, min(image_width - 1, near_x))

        return far_x, near_x, far_y

    def _compute_control(self, far_x, near_x, image_center_x):
        if self.smoothed_target_x is None:
            self.smoothed_target_x = float(far_x)
        else:
            self.smoothed_target_x = (
                self.smoothing_alpha * float(far_x)
                + (1.0 - self.smoothing_alpha) * self.smoothed_target_x
            )

        target_x = int(self.smoothed_target_x)

        # Lateral error: where the path is relative to the car/image center.
        lateral_error = target_x - image_center_x

        # Heading/curve error: where the path is going.
        # If far_x is left of near_x, the path curves left.
        heading_error = target_x - near_x

        steering = (
            self.steering_gain * lateral_error
            + self.heading_gain * heading_error
            + self.steering_bias
        )

        if self.invert_steering:
            steering = -steering

        steering = max(min(steering, self.max_steering), -self.max_steering)

        turn_ratio = abs(steering) / max(1e-6, self.max_steering)

        if turn_ratio > 0.70:
            throttle = self.throttle_sharp_turn
        elif turn_ratio > 0.35:
            throttle = self.throttle_turn
        else:
            throttle = self.throttle_straight

        return steering, throttle, target_x, lateral_error, heading_error

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
        far_x,
        near_x,
        target_x,
        image_center_x,
        text_lines,
    ):
        h, w = debug_img.shape[:2]
        roi_h = roi_y_end - roi_y_start

        debug_roi = debug_img[roi_y_start:roi_y_end, :]
        debug_roi[mask > 0] = self.lane_mask_color

        cv2.rectangle(debug_img, (0, roi_y_start), (w - 1, roi_y_end - 1), (0, 255, 255), 2)

        # Car/image center.
        cv2.line(debug_img, (image_center_x, 0), (image_center_x, h), (255, 0, 0), 2)

        far_y = roi_y_start + int(roi_h * self.lookahead_y_fraction)
        near_y = roi_y_start + int(roi_h * self.near_y_fraction)

        # Far lookahead row.
        cv2.line(debug_img, (0, far_y), (w - 1, far_y), (255, 255, 0), 2)

        # Near row.
        cv2.line(debug_img, (0, near_y), (w - 1, near_y), (0, 0, 255), 1)

        center_path = []

        for p in debug_points:
            y = roi_y_start + p["y"]

            if p["left_x"] is not None:
                cv2.circle(debug_img, (int(p["left_x"]), y), 5, (0, 255, 255), -1)

            if p["right_x"] is not None:
                cv2.circle(debug_img, (int(p["right_x"]), y), 5, (0, 255, 255), -1)

            if p["center_x"] is not None:
                cv2.circle(debug_img, (int(p["center_x"]), y), 5, (255, 0, 255), -1)
                center_path.append((int(p["center_x"]), y))

        if len(center_path) >= 2:
            center_path = sorted(center_path, key=lambda p: p[1])
            pts = np.array(center_path, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug_img, [pts], False, (255, 0, 255), 2)

        if far_x is not None:
            cv2.circle(debug_img, (int(far_x), far_y), 8, (255, 255, 0), -1)
            cv2.line(debug_img, (int(far_x), roi_y_start), (int(far_x), roi_y_end), (255, 255, 0), 1)

        if near_x is not None:
            cv2.circle(debug_img, (int(near_x), near_y), 8, (0, 0, 255), -1)

        if target_x is not None:
            cv2.line(debug_img, (int(target_x), roi_y_start), (int(target_x), roi_y_end), (255, 0, 255), 2)

        y_text = 15
        for line in text_lines:
            cv2.putText(
                debug_img,
                line,
                (5, y_text),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y_text += 17

    # ============================================================
    # DonkeyCar run loop
    # ============================================================

    def run(self, img):
        if img is None:
            return 0.0, 0.0, None

        steering = 0.0
        throttle = 0.0

        debug_img = np.copy(img)
        h, w = debug_img.shape[:2]
        image_center_x = w // 2

        roi_y_start = int(h * self.roi_y_start_frac)
        roi_y_end = int(h * self.roi_y_end_frac)
        roi_y_end = min(roi_y_end, h)

        roi = img[roi_y_start:roi_y_end, :]
        roi_h = roi.shape[0]

        mask = self._build_lane_mask(roi)
        center_points, debug_points = self._extract_center_points(mask, image_center_x)

        far_x, near_x, _ = self._fit_curve_targets(center_points, roi_h, w)

        if far_x is not None and near_x is not None:
            steering, throttle, target_x, lateral_error, heading_error = self._compute_control(
                far_x,
                near_x,
                image_center_x,
            )

            self.last_target_x = target_x
            self.last_near_x = near_x

            learned_width = self._get_lane_width_px(w)

            text_lines = [
                f"curve center | steer:{steering:.2f} thr:{throttle:.2f}",
                f"lat_err:{lateral_error} head_err:{heading_error} bands:{len(center_points)}",
                f"lane_width:{learned_width:.1f} color:{self.color_mode}",
            ]

            self._draw_debug(
                debug_img,
                roi_y_start,
                roi_y_end,
                mask,
                debug_points,
                far_x,
                near_x,
                target_x,
                image_center_x,
                text_lines,
            )

        else:
            self.smoothed_target_x = None

            left_count = int(np.count_nonzero(mask[:, :image_center_x]))
            right_count = int(np.count_nonzero(mask[:, image_center_x:]))

            text_lines = [
                f"NO CENTER PATH - stopped | Lpx:{left_count} Rpx:{right_count}",
                "Need lane points or one-line fallback with lane width",
            ]

            self._draw_debug(
                debug_img,
                roi_y_start,
                roi_y_end,
                mask,
                debug_points,
                None,
                None,
                None,
                image_center_x,
                text_lines,
            )

        return steering, throttle, debug_img