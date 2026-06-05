import cv2
import numpy as np


class LaneCenterFollower:
    """
    Bird's-eye sliding-window lane-boundary follower.

    This keeps the camera/bird's-eye flow from lane_center_follower.py, but uses
    the boundary-tracking idea from lane_center_follower_sw.py:
    - threshold lane-colored pixels
    - track left/right boundary fits over time
    - derive the lane center from those fitted boundaries
    - smooth and rate-limit the final steering target
    """

    def __init__(self, pid, cfg):
        self.pid_st = pid

        # Color threshold used by the active bird's-eye follower.
        self.lane_hsv_low = np.array(
            getattr(cfg, "LANE_HSV_LOW", (90, 50, 50)),
            dtype=np.uint8,
        )
        self.lane_hsv_high = np.array(
            getattr(cfg, "LANE_HSV_HIGH", (130, 255, 255)),
            dtype=np.uint8,
        )
        self.lane_mask_color = tuple(getattr(cfg, "LANE_MASK_COLOR", (0, 255, 0)))

        # Bird's-eye transform.
        self.use_birdseye = getattr(cfg, "LANE_USE_BIRDSEYE", True)
        self.bird_src_bottom_left = getattr(cfg, "LANE_BIRD_SRC_BOTTOM_LEFT", (0.08, 0.98))
        self.bird_src_bottom_right = getattr(cfg, "LANE_BIRD_SRC_BOTTOM_RIGHT", (0.92, 0.98))
        self.bird_src_top_left = getattr(cfg, "LANE_BIRD_SRC_TOP_LEFT", (0.38, 0.62))
        self.bird_src_top_right = getattr(cfg, "LANE_BIRD_SRC_TOP_RIGHT", (0.62, 0.62))
        self.bird_dst_bottom_left = getattr(cfg, "LANE_BIRD_DST_BOTTOM_LEFT", (0.20, 1.00))
        self.bird_dst_bottom_right = getattr(cfg, "LANE_BIRD_DST_BOTTOM_RIGHT", (0.80, 1.00))
        self.bird_dst_top_left = getattr(cfg, "LANE_BIRD_DST_TOP_LEFT", (0.20, 0.00))
        self.bird_dst_top_right = getattr(cfg, "LANE_BIRD_DST_TOP_RIGHT", (0.80, 0.00))
        self._bird_matrix = None
        self._bird_matrix_shape = None

        # ROI and search bounds in the transformed image.
        self.roi_y_start_frac = getattr(
            cfg,
            "LANE_ROI_Y_START",
            getattr(cfg, "LANE_ROI_Y_START_FRAC", 0.35),
        )
        self.roi_y_end_frac = getattr(cfg, "LANE_ROI_Y_END", 1.00)
        self.lookahead_y_frac = getattr(
            cfg,
            "LANE_LOOKAHEAD_Y_FRACTION",
            getattr(cfg, "LANE_LOOKAHEAD_Y_FRAC", 0.62),
        )
        self.search_x_start_frac = getattr(cfg, "LANE_SEARCH_X_START_FRAC", 0.0)
        self.search_x_end_frac = getattr(cfg, "LANE_SEARCH_X_END_FRAC", 1.0)
        self.image_center_offset_px = getattr(cfg, "LANE_IMAGE_CENTER_OFFSET_PX", 0)

        # Sliding-window lane search.
        self.num_windows = max(1, int(getattr(cfg, "LANE_SW_WINDOWS", getattr(cfg, "LANE_NUM_BANDS", 8))))
        self.window_margin = int(getattr(cfg, "LANE_SW_MARGIN", 35))
        self.min_window_pixels = int(
            getattr(cfg, "LANE_SW_MIN_WINDOW_PIXELS", getattr(cfg, "LANE_MIN_COLUMN_COUNT", 5))
        )
        self.min_lane_pixels = int(
            getattr(cfg, "LANE_SW_MIN_LANE_PIXELS", getattr(cfg, "LANE_MIN_PIXELS_PER_SIDE", 30))
        )
        self.min_one_side_pixels = int(
            getattr(cfg, "LANE_ONE_SIDE_MIN_PIXELS", self.min_lane_pixels)
        )
        self.previous_fit_margin = int(
            getattr(cfg, "LANE_PREVIOUS_FIT_MARGIN", self.window_margin + 15)
        )

        # Lane width validation and one-side fallback.
        self.allow_one_side_fallback = getattr(cfg, "LANE_ALLOW_ONE_SIDE_FALLBACK", True)
        self.expected_lane_width_px = getattr(cfg, "LANE_EXPECTED_WIDTH_PX", None)
        self.expected_lane_width_frac = getattr(cfg, "LANE_EXPECTED_WIDTH_FRAC", 0.55)
        self.learned_lane_width_px = None
        self.lane_width_alpha = float(
            np.clip(getattr(cfg, "LANE_WIDTH_SMOOTHING_ALPHA", 0.15), 0.0, 1.0)
        )
        self.min_lane_width_frac = getattr(cfg, "LANE_MIN_WIDTH_FRAC", 0.20)
        self.max_lane_width_frac = getattr(cfg, "LANE_MAX_WIDTH_FRAC", 0.75)
        self.min_lane_width_px = getattr(cfg, "LANE_MIN_WIDTH_PX", None)
        self.max_lane_width_px = getattr(cfg, "LANE_MAX_WIDTH_PX", None)

        # Mask cleanup.
        self.kernel_size = int(getattr(cfg, "LANE_KERNEL_SIZE", 5))
        self.min_component_area = int(
            getattr(
                cfg,
                "LANE_MIN_COMPONENT_AREA",
                getattr(cfg, "LANE_MIN_BLOB_AREA", 20),
            )
        )

        # Target smoothing and memory.
        self.smoothed_lane_center_x = None
        self.smoothing_alpha = float(
            np.clip(getattr(cfg, "LANE_SMOOTHING_ALPHA", 0.28), 0.0, 1.0)
        )
        self.fallback_smoothing_alpha = float(
            np.clip(
                getattr(cfg, "LANE_FALLBACK_SMOOTHING_ALPHA", min(self.smoothing_alpha, 0.10)),
                0.0,
                1.0,
            )
        )
        self.target_history = []
        self.target_median_window = int(getattr(cfg, "LANE_TARGET_MEDIAN_WINDOW", 5))
        self.max_target_step_px = getattr(cfg, "LANE_MAX_TARGET_STEP_PX", 10)
        self.fallback_max_target_step_px = getattr(
            cfg,
            "LANE_FALLBACK_MAX_TARGET_STEP_PX",
            5,
        )
        self.max_center_jump_px = getattr(
            cfg,
            "LANE_MAX_CENTER_JUMP_PX",
            getattr(cfg, "LANE_MAX_JUMP_PX", 120),
        )

        self.last_lane_center_x = None
        self.last_left_fit = None
        self.last_right_fit = None
        self.last_steering = 0.0
        self.missed_frames = 0
        self.max_missed_frames = int(getattr(cfg, "LANE_MAX_MISSED_FRAMES", 2))
        self.dropout_throttle = getattr(cfg, "LANE_DROPOUT_THROTTLE", 0.0)

        # Steering and throttle.
        self.steering_gain = getattr(cfg, "LANE_STEERING_GAIN", 0.06)
        self.max_steering = getattr(cfg, "LANE_MAX_STEERING", 0.80)
        self.steering_bias = getattr(cfg, "LANE_STEERING_BIAS", 0.0)
        self.invert_steering = getattr(cfg, "LANE_INVERT_STEERING", False)

        self.test_throttle = getattr(cfg, "LANE_TEST_THROTTLE", 0.30)
        self.min_throttle = getattr(cfg, "LANE_THROTTLE_MIN", 0.20)
        self.turn_throttle = getattr(cfg, "LANE_THROTTLE_TURN", self.test_throttle)
        self.one_side_throttle = getattr(cfg, "LANE_ONE_SIDE_THROTTLE", self.turn_throttle)
        self.throttle_slowdown = getattr(cfg, "LANE_THROTTLE_SLOWDOWN", 0.0)

    # ============================================================
    # Bird's-eye and mask utilities
    # ============================================================

    def _fractional_points(self, h, w):
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
        return src, dst

    def _birdseye_transform(self, img):
        if not self.use_birdseye:
            return img

        h, w = img.shape[:2]
        shape = (h, w)
        if self._bird_matrix is None or self._bird_matrix_shape != shape:
            src, dst = self._fractional_points(h, w)
            self._bird_matrix = cv2.getPerspectiveTransform(src, dst)
            self._bird_matrix_shape = shape

        return cv2.warpPerspective(img, self._bird_matrix, (w, h))

    def _filter_components(self, mask):
        if self.min_component_area <= 0:
            return mask

        contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        filtered = np.zeros_like(mask)

        for contour in contours:
            if cv2.contourArea(contour) >= self.min_component_area:
                cv2.drawContours(filtered, [contour], -1, 255, thickness=cv2.FILLED)

        return filtered

    def _make_lane_mask(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lane_hsv_low, self.lane_hsv_high)

        kernel_size = max(1, self.kernel_size)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = self._filter_components(mask)

        h, w = mask.shape[:2]
        x0 = int(np.clip(w * self.search_x_start_frac, 0, w - 1))
        x1 = int(np.clip(w * self.search_x_end_frac, x0 + 1, w))
        if x0 > 0:
            mask[:, :x0] = 0
        if x1 < w:
            mask[:, x1:] = 0

        return mask

    # ============================================================
    # Fit, validation, and center derivation
    # ============================================================

    def _fit_x_by_y(self, ys, xs):
        if len(xs) < 2 or len(np.unique(ys)) < 2:
            return None

        degree = 2 if len(np.unique(ys)) >= 3 else 1
        try:
            return np.poly1d(np.polyfit(ys, xs, degree))
        except Exception:
            return None

    def _width_limits(self, image_w):
        min_width = (
            float(self.min_lane_width_px)
            if self.min_lane_width_px is not None
            else float(self.min_lane_width_frac * image_w)
        )
        max_width = (
            float(self.max_lane_width_px)
            if self.max_lane_width_px is not None and self.max_lane_width_px > 0
            else float(self.max_lane_width_frac * image_w)
        )
        return min_width, max_width

    def _lane_width_is_plausible(self, lane_width_px, image_w):
        min_width, max_width = self._width_limits(image_w)
        return min_width <= lane_width_px <= max_width

    def _get_lane_width_px(self, image_w):
        if self.expected_lane_width_px is not None:
            return float(self.expected_lane_width_px)

        if self.learned_lane_width_px is not None:
            return float(self.learned_lane_width_px)

        return float(self.expected_lane_width_frac * image_w)

    def _update_lane_width(self, lane_width_px, image_w):
        if not self._lane_width_is_plausible(lane_width_px, image_w):
            return

        if self.learned_lane_width_px is None:
            self.learned_lane_width_px = float(lane_width_px)
        else:
            self.learned_lane_width_px = (
                self.lane_width_alpha * float(lane_width_px)
                + (1.0 - self.lane_width_alpha) * self.learned_lane_width_px
            )

    def _lane_center_jump_is_plausible(self, raw_lane_center_x):
        if self.last_lane_center_x is None or self.max_center_jump_px <= 0:
            return True

        return abs(raw_lane_center_x - self.last_lane_center_x) <= self.max_center_jump_px

    def _draw_fit(self, fit, roi_h, roi_w, debug_img, roi_y_start, color):
        if fit is None:
            return

        plot_y = np.linspace(0, roi_h - 1, roi_h).astype(np.int32)
        plot_x = np.clip(fit(plot_y), 0, roi_w - 1).astype(np.int32)
        points = np.column_stack((plot_x, plot_y + roi_y_start))
        cv2.polylines(debug_img, [points], False, color, 2)

    def _draw_center_path(self, lane, roi_h, roi_w, debug_img, roi_y_start):
        plot_y = np.linspace(0, roi_h - 1, roi_h).astype(np.int32)
        lane_width = lane["lane_width_px"]

        if lane["left_fit"] is not None and lane["right_fit"] is not None:
            center_x = (lane["left_fit"](plot_y) + lane["right_fit"](plot_y)) / 2.0
        elif lane["left_fit"] is not None:
            center_x = lane["left_fit"](plot_y) + lane_width / 2.0
        elif lane["right_fit"] is not None:
            center_x = lane["right_fit"](plot_y) - lane_width / 2.0
        else:
            return

        center_x = np.clip(center_x, 0, roi_w - 1).astype(np.int32)
        points = np.column_stack((center_x, plot_y + roi_y_start))
        cv2.polylines(debug_img, [points], False, (255, 0, 255), 2)

    def _build_lane_from_fits(
        self,
        left_fit,
        right_fit,
        roi_h,
        roi_w,
        debug_img,
        roi_y_start,
        left_pixels,
        right_pixels,
        source,
    ):
        lookahead_y = int(np.clip(roi_h * self.lookahead_y_frac, 0, roi_h - 1))
        left_boundary_x = int(np.clip(left_fit(lookahead_y), 0, roi_w - 1))
        right_boundary_x = int(np.clip(right_fit(lookahead_y), 0, roi_w - 1))

        lane_width_px = right_boundary_x - left_boundary_x
        if not self._lane_width_is_plausible(lane_width_px, roi_w):
            return None

        raw_lane_center_x = int((left_boundary_x + right_boundary_x) // 2)
        if not self._lane_center_jump_is_plausible(raw_lane_center_x):
            return None

        self._update_lane_width(lane_width_px, roi_w)

        absolute_lookahead_y = roi_y_start + lookahead_y
        self._draw_fit(left_fit, roi_h, roi_w, debug_img, roi_y_start, (0, 255, 255))
        self._draw_fit(right_fit, roi_h, roi_w, debug_img, roi_y_start, (0, 255, 255))
        cv2.circle(debug_img, (left_boundary_x, absolute_lookahead_y), 5, (0, 255, 255), -1)
        cv2.circle(debug_img, (right_boundary_x, absolute_lookahead_y), 5, (0, 255, 255), -1)
        cv2.line(debug_img, (0, absolute_lookahead_y), (roi_w - 1, absolute_lookahead_y), (255, 255, 0), 1)

        lane = {
            "left_boundary_x": left_boundary_x,
            "right_boundary_x": right_boundary_x,
            "raw_lane_center_x": raw_lane_center_x,
            "lookahead_y": absolute_lookahead_y,
            "lane_width_px": float(lane_width_px),
            "left_pixels": left_pixels,
            "right_pixels": right_pixels,
            "left_fit": left_fit,
            "right_fit": right_fit,
            "source": source,
            "tracking_mode": "both",
            "confidence": "high",
        }
        self._draw_center_path(lane, roi_h, roi_w, debug_img, roi_y_start)
        return lane

    def _build_lane_from_one_side(
        self,
        side,
        fit,
        roi_h,
        roi_w,
        debug_img,
        roi_y_start,
        visible_pixels,
        source,
    ):
        if not self.allow_one_side_fallback:
            return None

        lane_width_px = self._get_lane_width_px(roi_w)
        if not self._lane_width_is_plausible(lane_width_px, roi_w):
            return None

        lookahead_y = int(np.clip(roi_h * self.lookahead_y_frac, 0, roi_h - 1))
        visible_boundary_x = int(np.clip(fit(lookahead_y), 0, roi_w - 1))

        if side == "left":
            left_boundary_x = visible_boundary_x
            right_boundary_x = None
            raw_lane_center_x = visible_boundary_x + lane_width_px / 2.0
            left_fit = fit
            right_fit = None
            left_pixels = visible_pixels
            right_pixels = 0
        else:
            left_boundary_x = None
            right_boundary_x = visible_boundary_x
            raw_lane_center_x = visible_boundary_x - lane_width_px / 2.0
            left_fit = None
            right_fit = fit
            left_pixels = 0
            right_pixels = visible_pixels

        raw_lane_center_x = int(np.clip(raw_lane_center_x, 0, roi_w - 1))
        if not self._lane_center_jump_is_plausible(raw_lane_center_x):
            return None

        absolute_lookahead_y = roi_y_start + lookahead_y
        self._draw_fit(fit, roi_h, roi_w, debug_img, roi_y_start, (0, 255, 255))
        cv2.circle(debug_img, (visible_boundary_x, absolute_lookahead_y), 5, (0, 255, 255), -1)
        cv2.line(debug_img, (0, absolute_lookahead_y), (roi_w - 1, absolute_lookahead_y), (255, 255, 0), 1)

        lane = {
            "left_boundary_x": left_boundary_x,
            "right_boundary_x": right_boundary_x,
            "raw_lane_center_x": raw_lane_center_x,
            "lookahead_y": absolute_lookahead_y,
            "lane_width_px": float(lane_width_px),
            "left_pixels": left_pixels,
            "right_pixels": right_pixels,
            "left_fit": left_fit,
            "right_fit": right_fit,
            "source": f"{source}_{side}_only",
            "tracking_mode": f"{side}_only",
            "confidence": "medium",
        }
        self._draw_center_path(lane, roi_h, roi_w, debug_img, roi_y_start)
        return lane

    def _build_lane_from_pixel_groups(
        self,
        left_y,
        left_x,
        right_y,
        right_x,
        roi_h,
        roi_w,
        debug_img,
        roi_y_start,
        source,
    ):
        left_fit = self._fit_x_by_y(left_y, left_x)
        right_fit = self._fit_x_by_y(right_y, right_x)
        left_count = len(left_x)
        right_count = len(right_x)

        left_strong = left_fit is not None and left_count >= self.min_lane_pixels
        right_strong = right_fit is not None and right_count >= self.min_lane_pixels

        if left_strong and right_strong:
            lane = self._build_lane_from_fits(
                left_fit,
                right_fit,
                roi_h,
                roi_w,
                debug_img,
                roi_y_start,
                left_count,
                right_count,
                source,
            )
            if lane is not None:
                return lane

        if left_fit is not None and left_count >= self.min_one_side_pixels and not right_strong:
            return self._build_lane_from_one_side(
                "left",
                left_fit,
                roi_h,
                roi_w,
                debug_img,
                roi_y_start,
                left_count,
                source,
            )

        if right_fit is not None and right_count >= self.min_one_side_pixels and not left_strong:
            return self._build_lane_from_one_side(
                "right",
                right_fit,
                roi_h,
                roi_w,
                debug_img,
                roi_y_start,
                right_count,
                source,
            )

        return None

    # ============================================================
    # Lane search
    # ============================================================

    def _find_lane_near_previous_fit(self, mask, debug_img, roi_y_start):
        if self.last_left_fit is None and self.last_right_fit is None:
            return None

        roi_h, roi_w = mask.shape[:2]
        nonzero_y, nonzero_x = mask.nonzero()
        if len(nonzero_x) == 0:
            return None

        empty = np.array([], dtype=np.intp)
        left_inds = empty
        right_inds = empty

        if self.last_left_fit is not None:
            left_expected_x = self.last_left_fit(nonzero_y)
            left_inds = np.where(np.abs(nonzero_x - left_expected_x) <= self.previous_fit_margin)[0]

        if self.last_right_fit is not None:
            right_expected_x = self.last_right_fit(nonzero_y)
            right_inds = np.where(np.abs(nonzero_x - right_expected_x) <= self.previous_fit_margin)[0]

        return self._build_lane_from_pixel_groups(
            nonzero_y[left_inds],
            nonzero_x[left_inds],
            nonzero_y[right_inds],
            nonzero_x[right_inds],
            roi_h,
            roi_w,
            debug_img,
            roi_y_start,
            "previous",
        )

    def _find_lane_with_sliding_windows(self, mask, debug_img, roi_y_start):
        roi_h, roi_w = mask.shape[:2]
        image_center_x = roi_w // 2

        histogram_start = int(roi_h * 0.55)
        histogram = np.sum(mask[histogram_start:, :] > 0, axis=0)
        left_hist = histogram[:image_center_x]
        right_hist = histogram[image_center_x:]

        left_current = int(np.argmax(left_hist)) if left_hist.size and left_hist.max() > 0 else None
        right_current = (
            int(np.argmax(right_hist) + image_center_x)
            if right_hist.size and right_hist.max() > 0
            else None
        )

        if left_current is None and right_current is None:
            return None

        nonzero_y, nonzero_x = mask.nonzero()
        window_height = max(1, roi_h // self.num_windows)
        empty = np.array([], dtype=np.intp)
        left_lane_inds = []
        right_lane_inds = []

        for window_idx in range(self.num_windows):
            win_y_low = roi_h - (window_idx + 1) * window_height
            win_y_high = roi_h - window_idx * window_height
            if window_idx == self.num_windows - 1:
                win_y_low = 0

            good_left = empty
            if left_current is not None:
                left_x_low = max(0, left_current - self.window_margin)
                left_x_high = min(roi_w, left_current + self.window_margin)
                cv2.rectangle(
                    debug_img,
                    (left_x_low, roi_y_start + win_y_low),
                    (left_x_high, roi_y_start + win_y_high),
                    (0, 255, 255),
                    1,
                )
                good_left = (
                    (nonzero_y >= win_y_low)
                    & (nonzero_y < win_y_high)
                    & (nonzero_x >= left_x_low)
                    & (nonzero_x < left_x_high)
                ).nonzero()[0]

            good_right = empty
            if right_current is not None:
                right_x_low = max(0, right_current - self.window_margin)
                right_x_high = min(roi_w, right_current + self.window_margin)
                cv2.rectangle(
                    debug_img,
                    (right_x_low, roi_y_start + win_y_low),
                    (right_x_high, roi_y_start + win_y_high),
                    (0, 255, 255),
                    1,
                )
                good_right = (
                    (nonzero_y >= win_y_low)
                    & (nonzero_y < win_y_high)
                    & (nonzero_x >= right_x_low)
                    & (nonzero_x < right_x_high)
                ).nonzero()[0]

            left_lane_inds.append(good_left)
            right_lane_inds.append(good_right)

            if len(good_left) > self.min_window_pixels:
                left_current = int(np.mean(nonzero_x[good_left]))
            if len(good_right) > self.min_window_pixels:
                right_current = int(np.mean(nonzero_x[good_right]))

        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else empty
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else empty

        return self._build_lane_from_pixel_groups(
            nonzero_y[left_lane_inds],
            nonzero_x[left_lane_inds],
            nonzero_y[right_lane_inds],
            nonzero_x[right_lane_inds],
            roi_h,
            roi_w,
            debug_img,
            roi_y_start,
            "windows",
        )

    # ============================================================
    # Target stabilization, steering, and run loop
    # ============================================================

    def _median_filter_target(self, target_x):
        window = max(1, int(self.target_median_window))
        if window <= 1:
            return int(target_x)

        self.target_history.append(int(target_x))
        if len(self.target_history) > window:
            self.target_history = self.target_history[-window:]

        return int(np.median(np.array(self.target_history, dtype=np.float32)))

    def _limit_target_step(self, target_x, tracking_mode):
        if self.smoothed_lane_center_x is None:
            return int(target_x)

        max_step = (
            self.fallback_max_target_step_px
            if tracking_mode != "both"
            else self.max_target_step_px
        )
        if max_step is None or max_step <= 0:
            return int(target_x)

        previous = float(self.smoothed_lane_center_x)
        return int(max(previous - max_step, min(previous + max_step, float(target_x))))

    def _smooth_target(self, raw_target_x, tracking_mode):
        filtered = self._median_filter_target(raw_target_x)
        limited = self._limit_target_step(filtered, tracking_mode)
        alpha = self.smoothing_alpha if tracking_mode == "both" else self.fallback_smoothing_alpha

        if self.smoothed_lane_center_x is None:
            self.smoothed_lane_center_x = limited
        else:
            self.smoothed_lane_center_x = (
                alpha * limited + (1.0 - alpha) * self.smoothed_lane_center_x
            )

        return int(self.smoothed_lane_center_x), filtered, limited

    def _compute_throttle(self, steering, lane):
        throttle = self.test_throttle

        if self.throttle_slowdown > 0 and self.max_steering > 0:
            steering_ratio = min(abs(steering) / self.max_steering, 1.0)
            throttle *= max(0.0, 1.0 - steering_ratio * self.throttle_slowdown)
            throttle = max(self.min_throttle, throttle)

        if lane["tracking_mode"] != "both":
            throttle = min(throttle, self.one_side_throttle)

        return throttle

    def _remember_lane(self, lane, steering):
        self.last_lane_center_x = lane["raw_lane_center_x"]
        if lane["left_fit"] is not None:
            self.last_left_fit = lane["left_fit"]
        if lane["right_fit"] is not None:
            self.last_right_fit = lane["right_fit"]

        self.last_steering = steering
        self.missed_frames = 0

    def _clear_lane_memory(self):
        self.smoothed_lane_center_x = None
        self.target_history = []
        self.last_lane_center_x = None
        self.last_left_fit = None
        self.last_right_fit = None
        self.last_steering = 0.0

    def run(self, img):
        if img is None:
            return 0.0, 0.0, None

        control_img = self._birdseye_transform(img)
        debug_img = np.copy(control_img)
        h, w = control_img.shape[:2]

        image_center_x = int(np.clip((w // 2) + self.image_center_offset_px, 0, w - 1))
        roi_y_start = int(np.clip(h * self.roi_y_start_frac, 0, h - 1))
        roi_y_end = int(np.clip(h * self.roi_y_end_frac, roi_y_start + 1, h))
        roi = control_img[roi_y_start:roi_y_end, :]
        roi_h = roi.shape[0]

        mask = self._make_lane_mask(roi)
        debug_roi = debug_img[roi_y_start:roi_y_end, :]
        debug_roi[mask > 0] = self.lane_mask_color

        cv2.line(debug_img, (image_center_x, 0), (image_center_x, h), (255, 0, 0), 2)
        cv2.rectangle(debug_img, (0, roi_y_start), (w - 1, roi_y_end - 1), (0, 255, 255), 1)

        lane = self._find_lane_near_previous_fit(mask, debug_img, roi_y_start)
        if lane is None:
            lane = self._find_lane_with_sliding_windows(mask, debug_img, roi_y_start)

        if lane is not None:
            raw_lane_center_x = lane["raw_lane_center_x"]
            lane_center_x, filtered_x, limited_x = self._smooth_target(
                raw_lane_center_x,
                lane["tracking_mode"],
            )

            error = lane_center_x - image_center_x
            steering_error = -error if self.invert_steering else error
            steering = self.steering_gain * steering_error + self.steering_bias
            steering = max(min(steering, self.max_steering), -self.max_steering)

            throttle = self._compute_throttle(steering, lane)
            self._remember_lane(lane, steering)

            cv2.line(debug_img, (raw_lane_center_x, roi_y_start), (raw_lane_center_x, roi_y_end), (255, 255, 0), 1)
            cv2.line(debug_img, (lane_center_x, roi_y_start), (lane_center_x, roi_y_end), (255, 0, 255), 2)
            cv2.circle(debug_img, (lane_center_x, lane["lookahead_y"]), 7, (255, 0, 255), -1)

            text1 = (
                f"bird_sw {lane['source']} {lane['tracking_mode']} "
                f"raw:{raw_lane_center_x} filt:{filtered_x} lim:{limited_x} "
                f"tgt:{lane_center_x}"
            )
            text2 = (
                f"err:{error} steer:{steering:.2f} thr:{throttle:.2f} "
                f"w:{lane['lane_width_px']:.1f} px:{lane['left_pixels']}/{lane['right_pixels']}"
            )
            cv2.putText(debug_img, text1, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(debug_img, text2, (5, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            return steering, throttle, debug_img

        self.missed_frames += 1
        if self.last_lane_center_x is not None and self.missed_frames <= self.max_missed_frames:
            steering = self.last_steering
            throttle = self.dropout_throttle
            status = f"bird_sw missing - holding steer {self.missed_frames}/{self.max_missed_frames}"
        else:
            self._clear_lane_memory()
            steering = 0.0
            throttle = 0.0
            status = "bird_sw missing lane boundary - stopped"

        cv2.putText(debug_img, status, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        return steering, throttle, debug_img
