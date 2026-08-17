#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool

from cv_bridge import CvBridge


# ============================================================
# PARAMETERS
# ============================================================

MIN_LINE_LENGTH = 150
MIN_ELONGATION = 4.0

ADAPTIVE_BLOCK = 41
ADAPTIVE_C = 12

# ROI
ROI_TOP = 0.0
ROI_BOTTOM = 1.0
ROI_LEFT = 0.0
ROI_RIGHT = 1.0


class LaneDetectionNode(Node):

    def __init__(self):

        super().__init__('lane_detection_node')

        # ----------------------------------------------------
        # CV Bridge
        # ----------------------------------------------------

        self.bridge = CvBridge()

        # ----------------------------------------------------
        # Subscriber
        # ----------------------------------------------------

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.lane_error_pub = self.create_publisher(
            Float32,
            '/lane/error',
            10
        )

        self.lane_detected_pub = self.create_publisher(
            Bool,
            '/lane/detected',
            10
        )

        self.left_x_pub = self.create_publisher(
            Float32,
            '/lane/left_x',
            10
        )

        self.right_x_pub = self.create_publisher(
            Float32,
            '/lane/right_x',
            10
        )

        # ----------------------------------------------------
        # Visualization
        # ----------------------------------------------------

        self.show_debug = True

        if self.show_debug:

            cv2.namedWindow(
                'Lane Detection',
                cv2.WINDOW_NORMAL
            )

            cv2.namedWindow(
                'Lane Mask',
                cv2.WINDOW_NORMAL
            )

        self.get_logger().info(
            'Lane Detection Node started.'
        )

        self.get_logger().info(
            'Subscribing to /camera/image_raw'
        )

    # ========================================================
    # LANE DETECTION
    # ========================================================

    def detect_lanes(
        self,
        cv_image,
        min_length=MIN_LINE_LENGTH,
        min_elongation=MIN_ELONGATION
    ):

        h, w = cv_image.shape[:2]

        # ----------------------------------------------------
        # ROI
        # ----------------------------------------------------

        y0 = int(ROI_TOP * h)
        y1 = int(ROI_BOTTOM * h)

        x0 = int(ROI_LEFT * w)
        x1 = int(ROI_RIGHT * w)

        roi = cv_image[y0:y1, x0:x1]

        rh, rw = roi.shape[:2]

        # ----------------------------------------------------
        # BLACK TAPE MASK
        # ----------------------------------------------------

        gray = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2GRAY
        )

        blur = cv2.GaussianBlur(
            gray,
            (5, 5),
            0
        )

        mask = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            ADAPTIVE_BLOCK,
            ADAPTIVE_C
        )

        # ----------------------------------------------------
        # MORPHOLOGY
        # ----------------------------------------------------

        kernel = np.ones(
            (3, 3),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1
        )

        mask = cv2.dilate(
            mask,
            kernel,
            iterations=1
        )

        # ----------------------------------------------------
        # CONTOURS
        # ----------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []

        for c in contours:

            if len(c) < 5:
                continue

            rect = cv2.minAreaRect(c)

            (_, _), (bw, bh), _ = rect

            long_side = max(bw, bh)

            short_side = max(
                min(bw, bh),
                1.0
            )

            elongation = (
                long_side /
                short_side
            )

            if (
                long_side >= min_length
                and
                elongation >= min_elongation
            ):

                candidates.append(
                    (long_side, c)
                )

        # ----------------------------------------------------
        # SELECT TWO LONGEST OBJECTS
        # ----------------------------------------------------

        candidates.sort(
            key=lambda x: x[0],
            reverse=True
        )

        top_two = [
            c for _, c in candidates[:2]
        ]

        # ----------------------------------------------------
        # FIT LINE
        # ----------------------------------------------------

        def fit_full_line(pts):

            if len(pts) < 2:
                return None

            line = cv2.fitLine(
                pts,
                cv2.DIST_L2,
                0,
                0.01,
                0.01
            )

            vx, vy, px, py = line.flatten()

            if abs(vy) < 1e-6:
                return None

            # Top of image

            top_x = (
                px +
                (0 - py) *
                (vx / vy)
            )

            # Bottom of image

            bot_x = (
                px +
                (rh - 1 - py) *
                (vx / vy)
            )

            return (
                int(top_x) + x0,
                y0,

                int(bot_x) + x0,
                y0 + rh - 1
            )

        # Fit lines

        lines = [
            fit_full_line(c)
            for c in top_two
        ]

        lines = [
            line
            for line in lines
            if line is not None
        ]

        # ----------------------------------------------------
        # SORT LEFT → RIGHT
        # ----------------------------------------------------

        lines.sort(
            key=lambda line: line[2]
        )

        # ----------------------------------------------------
        # LEFT / RIGHT / CENTER
        # ----------------------------------------------------

        left_line = None
        right_line = None
        center_line = None

        if len(lines) == 2:

            left_line = lines[0]
            right_line = lines[1]

            top_cx = (
                left_line[0] +
                right_line[0]
            ) // 2

            bot_cx = (
                left_line[2] +
                right_line[2]
            ) // 2

            center_line = (
                top_cx,
                left_line[1],
                bot_cx,
                left_line[3]
            )

        return (
            mask,
            left_line,
            right_line,
            center_line
        )

    # ========================================================
    # IMAGE CALLBACK
    # ========================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'CvBridge error: {e}'
            )

            return

        # ----------------------------------------------------
        # DETECT LANES
        # ----------------------------------------------------

        (
            mask,
            left_line,
            right_line,
            center_line
        ) = self.detect_lanes(frame)

        frame_cx = frame.shape[1] // 2

        # ----------------------------------------------------
        # OUTPUT IMAGE
        # ----------------------------------------------------

        result = frame.copy()

        # ----------------------------------------------------
        # DRAW LEFT LINE
        # ----------------------------------------------------

        if left_line is not None:

            cv2.line(
                result,
                (left_line[0], left_line[1]),
                (left_line[2], left_line[3]),
                (0, 255, 0),
                4
            )

        # ----------------------------------------------------
        # DRAW RIGHT LINE
        # ----------------------------------------------------

        if right_line is not None:

            cv2.line(
                result,
                (right_line[0], right_line[1]),
                (right_line[2], right_line[3]),
                (255, 0, 0),
                4
            )

        # ----------------------------------------------------
        # LANE DETECTED
        # ----------------------------------------------------

        detected = False

        if center_line is not None:

            detected = True

            # Draw center

            cv2.line(
                result,
                (center_line[0], center_line[1]),
                (center_line[2], center_line[3]),
                (0, 0, 255),
                4
            )

            # ------------------------------------------------
            # LANE ERROR
            # ------------------------------------------------

            lane_cx = center_line[2]

            offset_px = (
                lane_cx -
                frame_cx
            )

            # Publish lane error

            error_msg = Float32()

            error_msg.data = float(
                offset_px
            )

            self.lane_error_pub.publish(
                error_msg
            )

            # Publish left/right positions

            left_msg = Float32()

            left_msg.data = float(
                left_line[2]
            )

            self.left_x_pub.publish(
                left_msg
            )

            right_msg = Float32()

            right_msg.data = float(
                right_line[2]
            )

            self.right_x_pub.publish(
                right_msg
            )

            # Display error

            cv2.putText(
                result,
                f'Lane Error: {offset_px:+.1f} px',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )

        else:

            cv2.putText(
                result,
                'LANE NOT DETECTED',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )

        # ----------------------------------------------------
        # PUBLISH DETECTION STATUS
        # ----------------------------------------------------

        detected_msg = Bool()

        detected_msg.data = detected

        self.lane_detected_pub.publish(
            detected_msg
        )

        # ----------------------------------------------------
        # CAMERA CENTER
        # ----------------------------------------------------

        cv2.line(
            result,
            (frame_cx, 0),
            (frame_cx, result.shape[0]),
            (255, 255, 255),
            1
        )

        # ----------------------------------------------------
        # DEBUG WINDOWS
        # ----------------------------------------------------

        if self.show_debug:

            cv2.imshow(
                'Lane Detection',
                result
            )

            cv2.imshow(
                'Lane Mask',
                mask
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):

                self.get_logger().info(
                    'Closing debug windows.'
                )

                self.show_debug = False

                cv2.destroyAllWindows()

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def destroy_node(self):

        if self.show_debug:

            cv2.destroyAllWindows()

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = LaneDetectionNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()