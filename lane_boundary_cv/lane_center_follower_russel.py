import cv2
import numpy as np


class LaneCenterFollower:
    """
    DonkeyCar computer-vision pilot that steers between two lane boundaries.

    The part receives `cam/image_array`, detects lane-colored boundary pixels,
    fits left and right boundary curves, estimates the lane center at a
    lookahead row, and returns pilot steering, pilot throttle, and a debug image.
    If color detection cannot build a valid lane, an optional Canny/Hough
    fallback can look for plausible edge-based boundaries before stopping.
    """

    def __init__(self, pid, cfg):
        """
        Initialize controller state and load tuning values from DonkeyCar config.

        `pid` is accepted because the DonkeyCar cv_control template passes one
        in, but this controller currently uses its own proportional steering
        gain instead of the PID object.
        """
        self.pid_st = pid

        # HSV threshold for the boundary tape color. Defaults are tuned for
        # white lane boundaries; override these in myconfig.py for blue tape.
        self.lane_hsv_low = np.array(
            getattr(cfg, "LANE_HSV_LOW", (0, 0, 170)),
            dtype=np.uint8,
        )
        self.lane_hsv_high = np.array(
            getattr(cfg, "LANE_HSV_HIGH", (180, 70, 255)),
            dtype=np.uint8,
        )
        self.lane_mask_color = tuple(getattr(cfg, "LANE_MASK_COLOR", (0, 255, 0)))

        # Smooth the lane center estimate so the magenta line jitters less.
        self.smoothed_lane_center_x = None
        self.smoothing_alpha = 0.25
        self.last_lane_center_x = None
        self.last_left_fit = None
        self.last_right_fit = None
        self.last_steering = 0.0
        self.missed_frames = 0

        # Sliding-window settings. Larger LOOKAHEAD means closer to the car.
        self.roi_y_start_frac = getattr(cfg, "LANE_ROI_Y_START_FRAC", 0.50)
        self.num_windows = max(1, int(getattr(cfg, "LANE_SW_WINDOWS", 6)))
        self.window_margin = getattr(cfg, "LANE_SW_MARGIN", 25)
        self.min_window_pixels = getattr(cfg, "LANE_SW_MIN_WINDOW_PIXELS", 5)
        self.min_lane_pixels = getattr(cfg, "LANE_SW_MIN_LANE_PIXELS", 30)
        self.lookahead_y_frac = getattr(cfg, "LANE_LOOKAHEAD_Y_FRAC", 0.75)
        self.min_lane_width_px = getattr(cfg, "LANE_MIN_WIDTH_PX", 25)
        self.max_lane_width_px = getattr(cfg, "LANE_MAX_WIDTH_PX", 260)
        self.max_center_jump_px = getattr(cfg, "LANE_MAX_CENTER_JUMP_PX", 55)
        self.previous_fit_margin = getattr(
            cfg,
            "LANE_PREVIOUS_FIT_MARGIN",
            self.window_margin + 10,
        )

        # Blob filtering removes tiny false positives before lane tracking.
        self.min_blob_area = getattr(cfg, "LANE_MIN_BLOB_AREA", 20)
        self.max_blob_area = getattr(cfg, "LANE_MAX_BLOB_AREA", 0)
        self.min_blob_width = getattr(cfg, "LANE_MIN_BLOB_WIDTH", 0)
        self.min_blob_height = getattr(cfg, "LANE_MIN_BLOB_HEIGHT", 0)

        # Edge fallback. This runs only after the color-mask searches fail.
        self.use_hough_fallback = getattr(cfg, "LANE_USE_HOUGH_FALLBACK", False)
        self.canny_low = getattr(cfg, "LANE_CANNY_LOW", 50)
        self.canny_high = getattr(cfg, "LANE_CANNY_HIGH", 150)
        self.hough_threshold = getattr(cfg, "LANE_HOUGH_THRESHOLD", 15)
        self.hough_min_line_length = getattr(
            cfg,
            "LANE_HOUGH_MIN_LINE_LENGTH",
            25,
        )
        self.hough_max_line_gap = getattr(cfg, "LANE_HOUGH_MAX_LINE_GAP", 20)
        self.hough_min_abs_dy = getattr(cfg, "LANE_HOUGH_MIN_ABS_DY", 12)
        self.hough_previous_fit_margin = getattr(
            cfg,
            "LANE_HOUGH_PREVIOUS_FIT_MARGIN",
            self.previous_fit_margin + 15,
        )
        self.hough_min_segments_per_side = getattr(
            cfg,
            "LANE_HOUGH_MIN_SEGMENTS_PER_SIDE",
            1,
        )

        # Short dropouts can hold steering, but default to zero throttle.
        self.max_missed_frames = getattr(cfg, "LANE_MAX_MISSED_FRAMES", 2)
        self.dropout_throttle = getattr(cfg, "LANE_DROPOUT_THROTTLE", 0.0)

        # Steering settings.
        # Confirmed on this car:
        # magenta left of red -> wheels turn left
        # magenta right of red -> wheels turn right
        self.steering_gain = 0.05
        self.max_steering = 0.40

        # Tiny throttle test.
        # Only used when both lane boundaries are detected.
        self.test_throttle = getattr(cfg, "LANE_TEST_THROTTLE", 0.45)
        self.min_throttle = getattr(cfg, "LANE_THROTTLE_MIN", 0.20)
        self.throttle_slowdown = getattr(cfg, "LANE_THROTTLE_SLOWDOWN", 0.60)

    def _make_lane_mask(self, roi):
        """
        Convert the region of interest into a cleaned binary lane-color mask.

        Pixels inside `LANE_HSV_LOW` and `LANE_HSV_HIGH` become white in the
        mask. Everything else becomes black. Morphology and blob filtering then
        remove small noise before sliding-window tracking uses the result.
        """
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lane_hsv_low, self.lane_hsv_high)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = self._filter_lane_blobs(mask)

        return mask

    def _filter_lane_blobs(self, mask):
        """
        Remove HSV detections that are too small or otherwise implausible.

        This keeps tiny glare/noise blobs from becoming inputs to the lane
        tracker. Set the blob thresholds in config to zero to disable each
        corresponding filter.
        """
        if (
            self.min_blob_area <= 0
            and self.max_blob_area <= 0
            and self.min_blob_width <= 0
            and self.min_blob_height <= 0
        ):
            return mask

        contour_result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        contours = contour_result[-2]
        filtered = np.zeros_like(mask)

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)

            if area < self.min_blob_area:
                continue
            if self.max_blob_area > 0 and area > self.max_blob_area:
                continue
            if width < self.min_blob_width:
                continue
            if height < self.min_blob_height:
                continue

            cv2.drawContours(filtered, [contour], -1, 255, thickness=cv2.FILLED)

        return filtered

    def _fit_x_by_y(self, ys, xs):
        """
        Fit a function that predicts boundary x-position from image y-position.

        Lane boundaries are mostly vertical in the image, so this fits x as a
        function of y. It uses a quadratic when there are enough distinct rows,
        otherwise a line.
        """
        unique_ys = np.unique(ys)
        degree = 2 if len(unique_ys) >= 3 else 1
        coeffs = np.polyfit(ys, xs, degree)
        return np.poly1d(coeffs)

    def _fit_line_x_by_y(self, ys, xs):
        """
        Fit a straight boundary x-position from image y-position.
        """
        coeffs = np.polyfit(ys, xs, 1)
        return np.poly1d(coeffs)

    def _lane_center_jump_is_plausible(self, raw_lane_center_x):
        """
        Reject sudden lane-center jumps that are too large to trust.

        This catches frames where a false positive briefly pulls the detected
        lane center far away from the previous good lane center.
        """
        if self.last_lane_center_x is None or self.max_center_jump_px <= 0:
            return True

        return abs(raw_lane_center_x - self.last_lane_center_x) <= self.max_center_jump_px

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
        """
        Validate fitted left/right boundaries and build a lane result dictionary.

        This method computes boundary positions at the lookahead row, checks lane
        width and lane-center jump limits, draws the fitted curves and lookahead
        markers on the debug image, and returns all values needed by `run()`.
        """
        lookahead_y = int(np.clip(roi_h * self.lookahead_y_frac, 0, roi_h - 1))
        left_boundary_x = int(np.clip(left_fit(lookahead_y), 0, roi_w - 1))
        right_boundary_x = int(np.clip(right_fit(lookahead_y), 0, roi_w - 1))

        lane_width_px = right_boundary_x - left_boundary_x
        if lane_width_px < self.min_lane_width_px:
            return None
        if self.max_lane_width_px > 0 and lane_width_px > self.max_lane_width_px:
            return None

        raw_lane_center_x = (left_boundary_x + right_boundary_x) // 2
        if not self._lane_center_jump_is_plausible(raw_lane_center_x):
            return None

        plot_y = np.linspace(0, roi_h - 1, roi_h).astype(np.int32)
        left_plot_x = np.clip(left_fit(plot_y), 0, roi_w - 1).astype(np.int32)
        right_plot_x = np.clip(right_fit(plot_y), 0, roi_w - 1).astype(np.int32)

        left_points = np.column_stack((left_plot_x, plot_y + roi_y_start))
        right_points = np.column_stack((right_plot_x, plot_y + roi_y_start))

        cv2.polylines(debug_img, [left_points], False, (0, 255, 255), 2)
        cv2.polylines(debug_img, [right_points], False, (0, 255, 255), 2)

        absolute_lookahead_y = roi_y_start + lookahead_y
        cv2.circle(
            debug_img,
            (left_boundary_x, absolute_lookahead_y),
            5,
            (0, 255, 255),
            -1,
        )
        cv2.circle(
            debug_img,
            (right_boundary_x, absolute_lookahead_y),
            5,
            (0, 255, 255),
            -1,
        )
        cv2.line(
            debug_img,
            (0, absolute_lookahead_y),
            (roi_w - 1, absolute_lookahead_y),
            (255, 255, 0),
            1,
        )

        return {
            "left_boundary_x": left_boundary_x,
            "right_boundary_x": right_boundary_x,
            "raw_lane_center_x": raw_lane_center_x,
            "lookahead_y": absolute_lookahead_y,
            "lane_width_px": lane_width_px,
            "left_pixels": left_pixels,
            "right_pixels": right_pixels,
            "left_fit": left_fit,
            "right_fit": right_fit,
            "source": source,
        }

    def _find_lane_with_sliding_windows(self, mask, debug_img, roi_y_start):
        """
        Find left and right lane boundaries with a fresh sliding-window search.

        The search starts from histogram peaks in the lower mask: one peak on
        the left half and one on the right half. It then moves stacked windows
        upward, recentering each side when enough lane pixels are found.
        """
        roi_h, roi_w = mask.shape[:2]
        image_center_x = roi_w // 2

        histogram_start = int(roi_h * 0.55)
        histogram = np.sum(mask[histogram_start:, :] > 0, axis=0)

        left_hist = histogram[:image_center_x]
        right_hist = histogram[image_center_x:]

        if left_hist.max() == 0 or right_hist.max() == 0:
            return None

        left_current = int(np.argmax(left_hist))
        right_current = int(np.argmax(right_hist) + image_center_x)

        nonzero_y, nonzero_x = mask.nonzero()
        window_height = max(1, roi_h // self.num_windows)

        left_lane_inds = []
        right_lane_inds = []

        for window_idx in range(self.num_windows):
            win_y_low = roi_h - (window_idx + 1) * window_height
            win_y_high = roi_h - window_idx * window_height

            if window_idx == self.num_windows - 1:
                win_y_low = 0

            left_x_low = max(0, left_current - self.window_margin)
            left_x_high = min(roi_w, left_current + self.window_margin)
            right_x_low = max(0, right_current - self.window_margin)
            right_x_high = min(roi_w, right_current + self.window_margin)

            cv2.rectangle(
                debug_img,
                (left_x_low, roi_y_start + win_y_low),
                (left_x_high, roi_y_start + win_y_high),
                (0, 255, 255),
                1,
            )
            cv2.rectangle(
                debug_img,
                (right_x_low, roi_y_start + win_y_low),
                (right_x_high, roi_y_start + win_y_high),
                (0, 255, 255),
                1,
            )

            good_left = (
                (nonzero_y >= win_y_low)
                & (nonzero_y < win_y_high)
                & (nonzero_x >= left_x_low)
                & (nonzero_x < left_x_high)
            ).nonzero()[0]
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

        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else []
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else []

        if (
            len(left_lane_inds) < self.min_lane_pixels
            or len(right_lane_inds) < self.min_lane_pixels
        ):
            return None

        left_x = nonzero_x[left_lane_inds]
        left_y = nonzero_y[left_lane_inds]
        right_x = nonzero_x[right_lane_inds]
        right_y = nonzero_y[right_lane_inds]

        if len(np.unique(left_y)) < 2 or len(np.unique(right_y)) < 2:
            return None

        left_fit = self._fit_x_by_y(left_y, left_x)
        right_fit = self._fit_x_by_y(right_y, right_x)

        return self._build_lane_from_fits(
            left_fit,
            right_fit,
            roi_h,
            roi_w,
            debug_img,
            roi_y_start,
            len(left_lane_inds),
            len(right_lane_inds),
            "windows",
        )

    def _find_lane_near_previous_fit(self, mask, debug_img, roi_y_start):
        """
        Recover a lane by searching near the previous good boundary curves.

        This fallback is used when the fresh sliding-window search fails. It
        accepts mask pixels that are close to the last left/right polynomial
        fits, then refits the boundaries from those nearby pixels.
        """
        if self.last_left_fit is None or self.last_right_fit is None:
            return None

        roi_h, roi_w = mask.shape[:2]
        nonzero_y, nonzero_x = mask.nonzero()

        if len(nonzero_x) == 0:
            return None

        left_expected_x = self.last_left_fit(nonzero_y)
        right_expected_x = self.last_right_fit(nonzero_y)

        left_lane_inds = np.where(
            np.abs(nonzero_x - left_expected_x) <= self.previous_fit_margin
        )[0]
        right_lane_inds = np.where(
            np.abs(nonzero_x - right_expected_x) <= self.previous_fit_margin
        )[0]

        if (
            len(left_lane_inds) < self.min_lane_pixels
            or len(right_lane_inds) < self.min_lane_pixels
        ):
            return None

        left_x = nonzero_x[left_lane_inds]
        left_y = nonzero_y[left_lane_inds]
        right_x = nonzero_x[right_lane_inds]
        right_y = nonzero_y[right_lane_inds]

        if len(np.unique(left_y)) < 2 or len(np.unique(right_y)) < 2:
            return None

        left_fit = self._fit_x_by_y(left_y, left_x)
        right_fit = self._fit_x_by_y(right_y, right_x)

        return self._build_lane_from_fits(
            left_fit,
            right_fit,
            roi_h,
            roi_w,
            debug_img,
            roi_y_start,
            len(left_lane_inds),
            len(right_lane_inds),
            "previous",
        )

    def _make_edge_mask(self, roi):
        """
        Build a Canny edge mask for the ROI.

        This mask is used only by the Hough fallback after color-based lane
        searches fail. It deliberately stays separate from the HSV mask because
        raw edges include lots of non-lane structure.
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        return cv2.Canny(blurred, self.canny_low, self.canny_high)

    def _classify_hough_segment(self, x1, y1, x2, y2, roi_w, roi_h):
        """
        Decide whether a Hough segment belongs to the left or right boundary.
        """
        dy = y2 - y1
        dx = x2 - x1

        if abs(dy) < self.hough_min_abs_dy:
            return None

        slope = dx / float(dy)
        intercept = x1 - slope * y1
        mid_y = (y1 + y2) * 0.5
        mid_x = slope * mid_y + intercept

        if (
            self.last_left_fit is not None
            and self.last_right_fit is not None
            and self.hough_previous_fit_margin > 0
        ):
            left_expected_x = float(self.last_left_fit(mid_y))
            right_expected_x = float(self.last_right_fit(mid_y))
            left_distance = abs(mid_x - left_expected_x)
            right_distance = abs(mid_x - right_expected_x)

            if min(left_distance, right_distance) <= self.hough_previous_fit_margin:
                return "left" if left_distance <= right_distance else "right"

            return None

        image_center_x = roi_w // 2
        bottom_x = slope * (roi_h - 1) + intercept

        if bottom_x < image_center_x:
            return "left"
        if bottom_x > image_center_x:
            return "right"

        return None

    def _append_hough_segment_points(self, target_xs, target_ys, x1, y1, x2, y2):
        """
        Add sampled points from a Hough segment so longer segments carry weight.
        """
        length = int(np.hypot(x2 - x1, y2 - y1))
        num_points = max(2, length // 5)

        target_xs.extend(np.linspace(x1, x2, num_points))
        target_ys.extend(np.linspace(y1, y2, num_points))

    def _find_lane_with_hough_edges(self, roi, debug_img, roi_y_start):
        """
        Find lane boundaries from Canny edges and Hough line segments.

        This fallback is intentionally run only after color-mask tracking fails.
        It is useful when washed-out tape still has a visible floor edge, but it
        is more easily fooled than HSV by floor seams, shadows, and baseboards.
        """
        if not self.use_hough_fallback:
            return None

        edge_mask = self._make_edge_mask(roi)
        roi_h, roi_w = edge_mask.shape[:2]

        lines = cv2.HoughLinesP(
            edge_mask,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap,
        )

        if lines is None:
            return None

        left_xs = []
        left_ys = []
        right_xs = []
        right_ys = []
        left_segments = 0
        right_segments = 0

        for line in lines.reshape(-1, 4):
            x1, y1, x2, y2 = [int(value) for value in line]
            side = self._classify_hough_segment(x1, y1, x2, y2, roi_w, roi_h)

            if side == "left":
                left_segments += 1
                self._append_hough_segment_points(left_xs, left_ys, x1, y1, x2, y2)
                cv2.line(
                    debug_img,
                    (x1, y1 + roi_y_start),
                    (x2, y2 + roi_y_start),
                    (255, 128, 0),
                    2,
                )
            elif side == "right":
                right_segments += 1
                self._append_hough_segment_points(right_xs, right_ys, x1, y1, x2, y2)
                cv2.line(
                    debug_img,
                    (x1, y1 + roi_y_start),
                    (x2, y2 + roi_y_start),
                    (255, 128, 0),
                    2,
                )

        if (
            left_segments < self.hough_min_segments_per_side
            or right_segments < self.hough_min_segments_per_side
        ):
            return None

        left_x = np.array(left_xs, dtype=np.float32)
        left_y = np.array(left_ys, dtype=np.float32)
        right_x = np.array(right_xs, dtype=np.float32)
        right_y = np.array(right_ys, dtype=np.float32)

        if len(np.unique(left_y)) < 2 or len(np.unique(right_y)) < 2:
            return None

        left_fit = self._fit_line_x_by_y(left_y, left_x)
        right_fit = self._fit_line_x_by_y(right_y, right_x)

        return self._build_lane_from_fits(
            left_fit,
            right_fit,
            roi_h,
            roi_w,
            debug_img,
            roi_y_start,
            len(left_x),
            len(right_x),
            "hough",
        )

    def _compute_throttle(self, steering):
        """
        Reduce throttle as steering demand increases.

        Straight-ish driving gets up to `LANE_TEST_THROTTLE`. Hard steering
        reduces throttle toward `LANE_THROTTLE_MIN`, controlled by
        `LANE_THROTTLE_SLOWDOWN`.
        """
        if self.max_steering <= 0:
            return self.test_throttle

        steering_ratio = min(abs(steering) / self.max_steering, 1.0)
        slowdown = steering_ratio * self.throttle_slowdown
        throttle = self.test_throttle * max(0.0, 1.0 - slowdown)
        throttle = max(self.min_throttle, throttle)

        return min(self.test_throttle, throttle)

    def _remember_lane(self, lane, steering):
        """
        Store the latest trusted lane for smoothing and dropout recovery.

        The saved polynomial fits are used by `_find_lane_near_previous_fit()`,
        and the saved steering can be held briefly when detection drops out.
        """
        self.last_lane_center_x = lane["raw_lane_center_x"]
        self.last_left_fit = lane["left_fit"]
        self.last_right_fit = lane["right_fit"]
        self.last_steering = steering
        self.missed_frames = 0

    def _clear_lane_memory(self):
        """
        Forget the previous lane after detection has been lost too long.

        Clearing this state prevents stale lane geometry from influencing future
        detections after the car has genuinely lost the boundaries.
        """
        self.smoothed_lane_center_x = None
        self.last_lane_center_x = None
        self.last_left_fit = None
        self.last_right_fit = None
        self.last_steering = 0.0

    def run(self, img):
        """
        Process one camera frame and return DonkeyCar pilot outputs.

        Returns `(steering, throttle, debug_img)`. If both lane boundaries are
        detected, the method steers toward the lane center and draws the overlay.
        If detection fails briefly, it can hold the last steering with dropout
        throttle. If detection stays lost, it stops and clears lane memory.
        """
        if img is None:
            return 0.0, 0.0, None

        # Default outputs.
        # If detection fails, the car stops.
        steering = 0.0
        throttle = 0.0

        debug_img = np.copy(img)
        h, w = debug_img.shape[:2]

        image_center_x = w // 2

        # Only look at the lower part of the image.
        roi_y_start = int(np.clip(h * self.roi_y_start_frac, 0, h - 1))
        roi = img[roi_y_start:h, :]

        mask = self._make_lane_mask(roi)

        # Draw detected lane-boundary pixels.
        debug_roi = debug_img[roi_y_start:h, :]
        debug_roi[mask > 0] = self.lane_mask_color

        # Draw image center line in red.
        cv2.line(
            debug_img,
            (image_center_x, 0),
            (image_center_x, h),
            (255, 0, 0),
            2,
        )

        lane = self._find_lane_with_sliding_windows(mask, debug_img, roi_y_start)
        if lane is None:
            lane = self._find_lane_near_previous_fit(mask, debug_img, roi_y_start)
        if lane is None:
            lane = self._find_lane_with_hough_edges(roi, debug_img, roi_y_start)

        if lane is not None:
            raw_lane_center_x = lane["raw_lane_center_x"]
            # Exponential smoothing to reduce frame-to-frame jitter.
            if self.smoothed_lane_center_x is None:
                self.smoothed_lane_center_x = raw_lane_center_x
            else:
                self.smoothed_lane_center_x = (
                    self.smoothing_alpha * raw_lane_center_x
                    + (1.0 - self.smoothing_alpha) * self.smoothed_lane_center_x
                )

            lane_center_x = int(self.smoothed_lane_center_x)
            error = lane_center_x - image_center_x

            # Convert lane-center error into steering.
            steering = self.steering_gain * error
            steering = max(min(steering, self.max_steering), -self.max_steering)

            # Slow down as steering demand rises.
            throttle = self._compute_throttle(steering)
            self._remember_lane(lane, steering)

            # Draw raw lane center as a thin yellow-ish guide.
            cv2.line(
                debug_img,
                (raw_lane_center_x, roi_y_start),
                (raw_lane_center_x, h),
                (255, 255, 0),
                1,
            )

            # Draw smoothed lane center in magenta.
            cv2.line(
                debug_img,
                (lane_center_x, roi_y_start),
                (lane_center_x, h),
                (255, 0, 255),
                2,
            )

            cv2.putText(
                debug_img,
                (
                    f"error: {error} steer: {steering:.2f} thr: {throttle:.2f} "
                    f"px: {lane['left_pixels']}/{lane['right_pixels']} "
                    f"w: {lane['lane_width_px']} src: {lane['source']}"
                ),
                (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:
            self.missed_frames += 1

            if (
                self.last_lane_center_x is not None
                and self.missed_frames <= self.max_missed_frames
            ):
                steering = self.last_steering
                throttle = self.dropout_throttle
                status_text = (
                    f"missing lane - holding steer "
                    f"{self.missed_frames}/{self.max_missed_frames}"
                )
            else:
                self._clear_lane_memory()
                status_text = "missing lane boundary - stopped"

            cv2.putText(
                debug_img,
                status_text,
                (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        return steering, throttle, debug_img
