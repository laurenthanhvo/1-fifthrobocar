import cv2
import numpy as np


class LaneCenterFollower:
    def __init__(self, pid, cfg):
        self.pid_st = pid

        # Smooth the lane center estimate so the magenta line jitters less.
        self.smoothed_lane_center_x = None
        self.smoothing_alpha = 0.25

        # Minimum number of white pixels needed on each side.
        self.min_pixels = 20

        # Steering settings.
        # Confirmed on this car:
        # magenta left of red -> wheels turn left
        # magenta right of red -> wheels turn right
        self.steering_gain = 0.05
        self.max_steering = 0.40

        # Tiny throttle test.
        # Only used when both lane boundaries are detected.
        self.test_throttle = 0.45

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

        # Only look at the lower half of the image.
        roi_y_start = h // 2
        roi = img[roi_y_start:h, :]

        # Convert to HSV for white thresholding.
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        # White-ish pixels: low saturation, high value.
        lower_white = np.array([0, 0, 170])
        upper_white = np.array([180, 70, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Clean up noise.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Draw detected white pixels in green.
        debug_roi = debug_img[roi_y_start:h, :]
        debug_roi[mask > 0] = [0, 255, 0]

        # Draw image center line in red.
        cv2.line(
            debug_img,
            (image_center_x, 0),
            (image_center_x, h),
            (255, 0, 0),
            2,
        )

        # Split the mask into left and right halves.
        left_mask = mask[:, :image_center_x]
        right_mask = mask[:, image_center_x:]

        _, left_xs = np.where(left_mask > 0)
        _, right_xs = np.where(right_mask > 0)

        if len(left_xs) > self.min_pixels and len(right_xs) > self.min_pixels:
            left_boundary_x = int(np.mean(left_xs))
            right_boundary_x = int(np.mean(right_xs) + image_center_x)

            raw_lane_center_x = (left_boundary_x + right_boundary_x) // 2

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

            # Draw detected left boundary estimate in cyan.
            cv2.line(
                debug_img,
                (left_boundary_x, roi_y_start),
                (left_boundary_x, h),
                (0, 255, 255),
                2,
            )

            # Draw detected right boundary estimate in cyan.
            cv2.line(
                debug_img,
                (right_boundary_x, roi_y_start),
                (right_boundary_x, h),
                (0, 255, 255),
                2,
            )

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
                f"error: {error} steer: {steering:.2f} thr: {throttle:.2f}",
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