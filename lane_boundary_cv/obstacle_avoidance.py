import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Configuration defaults — override these in your myconfig.py
# ---------------------------------------------------------------------------
# OBSTACLE_SCAN_Y         = 40    # top of the detection ROI (pixels from top)
# OBSTACLE_SCAN_HEIGHT    = 80    # height of the ROI in pixels
# OBSTACLE_DEPTH_THRESH   = 800   # stop if anything is closer than this (mm)
# OBSTACLE_WARN_THRESH    = 1200  # begin slowing down beyond stop threshold (mm)
# OBSTACLE_MIN_AREA       = 1500  # minimum contour area (px²) to count as obstacle (RGB fallback)
# OBSTACLE_STOP_THROTTLE  = 0.0   # throttle override when obstacle is too close
# OBSTACLE_SLOW_FACTOR    = 0.5   # multiply normal throttle by this when warning zone
# OBSTACLE_STEER_BIAS     = 0.35  # steering nudge applied when avoiding (+ = right, - = left)
# OBSTACLE_OVERLAY        = True  # draw debug overlay on the output image
# ---------------------------------------------------------------------------


class ObstacleDetector:
    """
    Donkeycar part — detects obstacles using OAK depth data (primary)
    or RGB contour detection (fallback when depth is unavailable).

    Inputs  (vehicle memory keys):
        cam/image_array     — RGB image from the camera
        cam/depth_array     — uint16 depth map in mm from OAK stereo (optional)

    Outputs (vehicle memory keys):
        obstacle/detected   — bool, True when an obstacle is in the ROI
        obstacle/distance   — float, estimated distance in mm (0 = unknown)
        obstacle/position   — str, 'left' | 'center' | 'right' | 'none'
        cv/obstacle_image   — annotated image for the web UI
    """

    def __init__(self, cfg):
        # ROI for scanning — a horizontal band in front of the car
        self.scan_y       = getattr(cfg, 'OBSTACLE_SCAN_Y',        40)
        self.scan_height  = getattr(cfg, 'OBSTACLE_SCAN_HEIGHT',   80)

        # Depth thresholds (mm)
        self.depth_stop   = getattr(cfg, 'OBSTACLE_DEPTH_THRESH',  800)
        self.depth_warn   = getattr(cfg, 'OBSTACLE_WARN_THRESH',  1200)

        # RGB fallback contour minimum area
        self.min_area     = getattr(cfg, 'OBSTACLE_MIN_AREA',     1500)

        self.overlay      = getattr(cfg, 'OBSTACLE_OVERLAY',       True)

        # Internal state
        self.detected     = False
        self.distance     = 0.0
        self.position     = 'none'   # 'left' | 'center' | 'right' | 'none'
        self._confirm_frames  = getattr(cfg, 'OBSTACLE_CONFIRM_FRAMES', 3)

        # debounce state - 
        self._confirm_frames = getattr(cfg, 'OBSTACLE_CONFIRM_FRAMES', 3)
        self._clear_frames   = getattr(cfg, 'OBSTACLE_CLEAR_FRAMES',   5)
        self._yes_count      = 0
        self._no_count       = 0
        self._latched        = False

        self._cfg = cfg

    # ------------------------------------------------------------------
    # Primary method: depth-based detection
    # ------------------------------------------------------------------
    def _detect_depth(self, depth_img, img_w):
        """
        Slice the ROI from the depth map and find the closest point.
        Returns (detected: bool, distance: float, position: str).
        """
        roi = depth_img[self.scan_y : self.scan_y + self.scan_height, :]

        # Zero values mean 'no reading' in OAK depth maps — mask them out
        valid = roi[roi > 0]
        if valid.size == 0:
            return False, 0.0, 'none'

        min_dist = float(np.min(valid))

        if min_dist > self.depth_warn:
            return False, min_dist, 'none'

        # Find where the close pixels are to get lateral position
        close_mask = (roi > 0) & (roi < self.depth_warn)
        cols = np.where(close_mask)[1]
        if cols.size == 0:
            return False, min_dist, 'none'

        center_col = float(np.mean(cols))
        third = img_w / 3.0
        if center_col < third:
            pos = 'left'
        elif center_col < 2 * third:
            pos = 'center'
        else:
            pos = 'right'

        detected = min_dist < self.depth_stop
        return detected, min_dist, pos

    # ------------------------------------------------------------------
    # Fallback: RGB contour-based detection
    # ------------------------------------------------------------------
    def _detect_rgb(self, cam_img):
        """
        Very lightweight RGB fallback: looks for large blobs in the ROI
        that don't match typical road/track colors.
        Returns (detected: bool, distance: float, position: str).
        """
        roi = cam_img[self.scan_y : self.scan_y + self.scan_height, :]
        img_w = cam_img.shape[1]

        gray   = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges  = cv2.Canny(blurred, 50, 150)
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(edges, kernel, iterations=2)
       
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        lane_low  = np.array(getattr(self._cfg, 'LANE_HSV_LOW',  (90,  50,  50)), dtype=np.uint8)
        lane_high = np.array(getattr(self._cfg, 'LANE_HSV_HIGH', (130, 255, 255)), dtype=np.uint8)
        lane_mask = cv2.inRange(hsv_roi, lane_low, lane_high)
        dilated[lane_mask > 0] = 0

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_area = 0
        best_cx   = img_w / 2

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area and area > best_area:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    best_cx   = M['m10'] / M['m00']
                    best_area = area

        if best_area == 0:
            return False, 0.0, 'none'

        # Distance is a rough proxy — larger contour ≈ closer object
        # Normalize: at min_area the object is ~far; at 10×min_area it's ~close
        proximity = min(best_area / (10 * self.min_area), 1.0)
        approx_dist = self.depth_warn * (1.0 - proximity)

        third = img_w / 3.0
        if best_cx < third:
            pos = 'left'
        elif best_cx < 2 * third:
            pos = 'center'
        else:
            pos = 'right'

        detected = best_area > (3 * self.min_area)
        return detected, approx_dist, pos

    # ------------------------------------------------------------------
    # Overlay / telemetry
    # ------------------------------------------------------------------
    def _draw_overlay(self, cam_img, detected, distance, position):
        img = np.copy(cam_img)
        img_h, img_w = img.shape[:2]

        # Draw the scan ROI rectangle
        color = (0, 0, 255) if detected else (0, 255, 0)
        cv2.rectangle(
            img,
            (0, self.scan_y),
            (img_w, self.scan_y + self.scan_height),
            color, 2
        )

        # Status text
        lines = [
            f"OBSTACLE: {'YES' if detected else 'NO'}",
            f"DIST: {distance:.0f}mm" if distance > 0 else "DIST: N/A",
            f"POS: {position.upper()}",
        ]
        y = 15
        for line in lines:
            cv2.putText(img, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 0), 3)
            cv2.putText(img, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 255), 1)
            y += 16

        return img

    # ------------------------------------------------------------------
    # Donkeycar part interface
    # ------------------------------------------------------------------
    def run(self, cam_img, overlay_img=None, depth_img=None):
      if cam_img is None:
          return False, 0.0, 'none', overlay_img

      img_w = cam_img.shape[1]

      if depth_img is not None:
          detected, distance, position = self._detect_depth(depth_img, img_w)
      else:
          # detected, distance, position = self._detect_rgb(cam_img)
          detect_img = overlay_img if overlay_img is not None else cam_img
          detected, distance, position = self._detect_rgb(detect_img)

      # Debounce: require N consecutive detections to latch ON,
      # and M consecutive misses to latch OFF
      if detected:
          self._yes_count += 1
          self._no_count   = 0
          if self._yes_count >= self._confirm_frames:
              self._latched = True
      else:
          self._no_count  += 1
          self._yes_count  = 0
          if self._no_count >= self._clear_frames:
              self._latched = False

      detected = self._latched  # use debounced state for everything below

      self.detected = detected
      self.distance = distance
      self.position = position

      if overlay_img is not None and self.overlay:
          out_img = self._draw_overlay(np.copy(overlay_img), detected, distance, position)
      else:
          out_img = overlay_img  # never fall back to raw cam_img

      return detected, distance, position, out_img


# ---------------------------------------------------------------------------

class ObstacleAvoider:
    """
    Donkeycar part — sits downstream of the lane follower and overrides
    steering / throttle when an obstacle is detected.

    Inputs  (vehicle memory keys):
        pilot/steering      — steering from the lane follower (-1..1)
        pilot/throttle      — throttle from the lane follower (-1..1)
        obstacle/detected   — bool from ObstacleDetector
        obstacle/distance   — float (mm) from ObstacleDetector
        obstacle/position   — str from ObstacleDetector

    Outputs (vehicle memory keys):
        pilot/steering      — (possibly overridden) steering
        pilot/throttle      — (possibly overridden) throttle
    """

    def __init__(self, cfg):
        self.depth_stop    = getattr(cfg, 'OBSTACLE_DEPTH_THRESH',  800)
        self.depth_warn    = getattr(cfg, 'OBSTACLE_WARN_THRESH',  1200)
        self.stop_throttle = getattr(cfg, 'OBSTACLE_STOP_THROTTLE', 0.0)
        self.slow_factor   = getattr(cfg, 'OBSTACLE_SLOW_FACTOR',   0.5)
        self.steer_bias    = getattr(cfg, 'OBSTACLE_STEER_BIAS',    0.35)

    def run(self, steering, throttle, detected, distance, position):
        """
        Returns: (steering, throttle)  — overridden values if necessary.
        """
        if steering is None:
            steering = 0.0
        if throttle is None:
            throttle = 0.0

        if not detected:
            # No obstacle in stop zone, but check warning zone
            if 0 < distance < self.depth_warn:
                # Slow down proportionally
                warn_fraction = max(0.0, (distance - self.depth_stop) /
                                    (self.depth_warn - self.depth_stop))
                throttle = throttle * (self.slow_factor + (1 - self.slow_factor) * warn_fraction)
            return steering, throttle

        # --- Obstacle is in the STOP zone ---
        if distance > 0 and distance < self.depth_stop:
            logger.warning(f"Obstacle STOP: {distance:.0f}mm at {position}")

            # Hard stop
            throttle = self.stop_throttle

            # Steer away from the obstacle
            if position == 'left':
                steering = min(1.0, steering + self.steer_bias)   # nudge right
            elif position == 'right':
                steering = max(-1.0, steering - self.steer_bias)  # nudge left
            # 'center' → keep current steering (or add small random bias if stuck)

        else:
            # Detected but just entering warn zone
            logger.info(f"Obstacle WARN: {distance:.0f}mm at {position}")
            throttle = throttle * self.slow_factor
            if position == 'left':
                steering = min(1.0, steering + self.steer_bias * 0.5)
            elif position == 'right':
                steering = max(-1.0, steering - self.steer_bias * 0.5)

        return steering, throttle


# ---------------------------------------------------------------------------
# Convenience: combined part if you want detector + avoider in one object
# ---------------------------------------------------------------------------

class ObstacleAvoidancePilot:
    """
    Single combined part that wraps both ObstacleDetector and ObstacleAvoider.
    Useful if you prefer a simpler vehicle loop setup.

    Inputs:
        cam/image_array, cam/depth_array (optional),
        pilot/steering, pilot/throttle

    Outputs:
        pilot/steering, pilot/throttle, cv/obstacle_image
    """

    def __init__(self, cfg):
        self.detector = ObstacleDetector(cfg)
        self.avoider  = ObstacleAvoider(cfg)

    def run(self, cam_img, depth_img, steering, throttle):
        detected, distance, position, out_img = self.detector.run(cam_img, depth_img)
        steering, throttle = self.avoider.run(steering, throttle, detected, distance, position)
        return steering, throttle, out_img