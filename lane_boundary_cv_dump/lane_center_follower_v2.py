import cv2
import numpy as np


class LaneCenterFollower:
    def __init__(self, pid, cfg):
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

        # Sliding-window settings. Larger LOOKAHEAD means closer to the car.
        self.roi_y_start_frac = getattr(cfg, "LANE_ROI_Y_START_FRAC", 0.50)
        self.num_windows = getattr(cfg, "LANE_SW_WINDOWS", 6)
        self.window_margin = getattr(cfg, "LANE_SW_MARGIN", 25)
        self.min_window_pixels = getattr(cfg, "LANE_SW_MIN_WINDOW_PIXELS", 5)
        self.min_lane_pixels = getattr(cfg, "LANE_SW_MIN_LANE_PIXELS", 30)
        self.lookahead_y_frac = getattr(cfg, "LANE_LOOKAHEAD_Y_FRAC", 0.75)
        self.min_lane_width_px = getattr(cfg, "LANE_MIN_WIDTH_PX", 25)

        # Steering settings.
        # Confirmed on this car:
        # magenta left of red -> wheels turn left
        # magenta right of red -> wheels turn right
        self.steering_gain = 0.05
        self.max_steering = 0.40

        # Tiny throttle test.
        # Only used when both lane boundaries are detected.
        self.test_throttle = 0.45

    def _make_lane_mask(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lane_hsv_low, self.lane_hsv_high)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def _fit_x_by_y(self, ys, xs):
        unique_ys = np.unique(ys)
        degree = 2 if len(unique_ys) >= 3 else 1
        coeffs = np.polyfit(ys, xs, degree)
        return np.poly1d(coeffs)

    def _find_lane_with_sliding_windows(self, mask, debug_img, roi_y_start):
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

        lookahead_y = int(roi_h * self.lookahead_y_frac)
        left_boundary_x = int(np.clip(left_fit(lookahead_y), 0, roi_w - 1))
        right_boundary_x = int(np.clip(right_fit(lookahead_y), 0, roi_w - 1))

        lane_width_px = right_boundary_x - left_boundary_x
        if lane_width_px < self.min_lane_width_px:
            return None

        raw_lane_center_x = (left_boundary_x + right_boundary_x) // 2

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
            "left_pixels": len(left_lane_inds),
            "right_pixels": len(right_lane_inds),
        }

    def run(self, img):
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
        roi_y_start = int(h * self.roi_y_start_frac)
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

            # Tiny forward motion only when both boundaries are visible.
            throttle = self.test_throttle

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
                    f"px: {lane['left_pixels']}/{lane['right_pixels']}"
                ),
                (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        else:
            # If one side disappears, reset smoothing so stale lane center
            # does not hang around once detection comes back.
            self.smoothed_lane_center_x = None

            cv2.putText(
                debug_img,
                "missing lane boundary - stopped",
                (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        return steering, throttle, debug_img
