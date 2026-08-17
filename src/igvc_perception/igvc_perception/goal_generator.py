#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


# ============================================================
# PARAMETERS
# ============================================================

# Desired distance ahead of the robot.
LOOKAHEAD_DISTANCE = 2.0

# Minimum distance to try if the desired goal is not yet mapped.
MIN_LOOKAHEAD_DISTANCE = 0.5

# Step used when searching backwards for a valid goal.
LOOKAHEAD_STEP = 0.2

# Lateral conversion from lane error to meters.
PIXELS_TO_METERS = 0.004

# Safety clamp for lateral movement.
MAX_LATERAL_OFFSET = 1.0

# Minimum movement of the goal before resending.
GOAL_UPDATE_MIN_DIST = 0.3

ROBOT_FRAME = 'base_link'
GLOBAL_FRAME = 'map'

# Occupancy grid thresholds.
OCCUPIED_THRESHOLD = 50
UNKNOWN_VALUE = -1


class GoalGeneratorNode(Node):

    def __init__(self):
        super().__init__('goal_generator_node')

        # --------------------------------------------------------
        # State
        # --------------------------------------------------------

        self.lane_error = 0.0
        self.lane_detected = False
        self.last_goal_xy = None

        self.current_map = None

        # True while Nav2 is processing a goal.
        self.goal_active = False

        # --------------------------------------------------------
        # TF
        # --------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # --------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------

        self.create_subscription(
            Float32,
            '/lane/error',
            self.error_cb,
            10
        )

        self.create_subscription(
            Bool,
            '/lane/detected',
            self.detected_cb,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb,
            10
        )

        # --------------------------------------------------------
        # Nav2 action client
        # --------------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Run at 2 Hz.
        self.timer = self.create_timer(
            0.5,
            self.update_goal
        )

        self.get_logger().info(
            'Goal Generator Node started.'
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def error_cb(self, msg):
        self.lane_error = msg.data

    def detected_cb(self, msg):
        self.lane_detected = msg.data

    def map_cb(self, msg):
        self.current_map = msg

    # ============================================================
    # MAP VALIDITY
    # ============================================================

    def is_goal_valid(self, x, y):
        """
        Check whether a map-frame coordinate is:

        1. Inside the current occupancy grid.
        2. Known.
        3. Not occupied.

        Returns:

            (True, 'free')
            (False, 'outside map bounds')
            (False, 'map cell is unknown')
            (False, 'map cell is occupied')
        """

        if self.current_map is None:
            return False, 'no map'

        grid = self.current_map
        info = grid.info

        resolution = info.resolution

        # Convert world coordinates to map cell coordinates.
        map_x = int(
            (x - info.origin.position.x) / resolution
        )

        map_y = int(
            (y - info.origin.position.y) / resolution
        )

        # --------------------------------------------------------
        # Bounds check
        # --------------------------------------------------------

        if (
            map_x < 0
            or map_x >= info.width
            or map_y < 0
            or map_y >= info.height
        ):
            return False, 'outside map bounds'

        # --------------------------------------------------------
        # Occupancy check
        # --------------------------------------------------------

        index = map_y * info.width + map_x

        cell_value = grid.data[index]

        if cell_value == UNKNOWN_VALUE:
            return False, 'map cell is unknown'

        if cell_value >= OCCUPIED_THRESHOLD:
            return False, 'map cell is occupied'

        return True, 'free'

    # ============================================================
    # CREATE LOCAL GOAL
    # ============================================================

    def create_local_goal(self, lookahead_distance):
        """
        Create a goal in base_link coordinates.

        The lane error determines the lateral offset.

        x = forward
        y = left
        """

        # Flip the sign here if the robot steers in the wrong direction.
        lateral_offset = max(
            -MAX_LATERAL_OFFSET,
            min(
                MAX_LATERAL_OFFSET,
                -self.lane_error * PIXELS_TO_METERS
            )
        )

        local_goal = PoseStamped()

        local_goal.header.frame_id = ROBOT_FRAME
        # local_goal.header.stamp = self.get_clock().now().to_msg()
        local_goal.header.stamp = Time(nanoseconds=0).to_msg()
        local_goal.pose.position.x = lookahead_distance
        local_goal.pose.position.y = lateral_offset
        local_goal.pose.position.z = 0.0

        # No specific orientation is required for the candidate.
        local_goal.pose.orientation.x = 0.0
        local_goal.pose.orientation.y = 0.0
        local_goal.pose.orientation.z = 0.0
        local_goal.pose.orientation.w = 1.0

        return local_goal

    # ============================================================
    # FIND VALID GOAL
    # ============================================================

    def find_valid_goal(self):
        """
        Try the desired lookahead first.

        If it is outside the map or unknown, progressively reduce
        the lookahead distance until a valid free cell is found.

        Example:

            2.0 m  -> unknown
            1.8 m  -> unknown
            1.6 m  -> unknown
            1.4 m  -> free

        Then 1.4 m is returned.

        Returns:

            PoseStamped, reason

        or

            None, reason
        """

        distance = LOOKAHEAD_DISTANCE

        while distance >= MIN_LOOKAHEAD_DISTANCE:

            local_goal = self.create_local_goal(distance)

            try:
                global_goal = self.tf_buffer.transform(
                    local_goal,
                    GLOBAL_FRAME,
                    timeout=Duration(seconds=0.2)
                )

            except Exception as e:

                self.get_logger().warn(
                    f'TF transform failed: {e}',
                    throttle_duration_sec=5.0
                )

                return None, 'tf failure'

            gx = global_goal.pose.position.x
            gy = global_goal.pose.position.y

            valid, reason = self.is_goal_valid(gx, gy)

            if valid:

                return global_goal, 'free'

            # Log rejected candidates.
            self.get_logger().debug(
                f'Candidate ({gx:.2f}, {gy:.2f}) at '
                f'{distance:.2f} m rejected: {reason}'
            )

            distance -= LOOKAHEAD_STEP

        return None, 'no valid goal found'

    # ============================================================
    # GOAL UPDATE
    # ============================================================

    def update_goal(self):

        # --------------------------------------------------------
        # No lane
        # --------------------------------------------------------

        if not self.lane_detected:
            return

        # --------------------------------------------------------
        # Don't send another goal while Nav2 is working
        # --------------------------------------------------------

        if self.goal_active:
            return

        # --------------------------------------------------------
        # Map unavailable
        # --------------------------------------------------------

        if self.current_map is None:

            self.get_logger().warn(
                'No /map received yet — cannot validate candidate goals.',
                throttle_duration_sec=5.0
            )

            return

        # --------------------------------------------------------
        # Nav2 unavailable
        # --------------------------------------------------------

        if not self.nav_client.server_is_ready():

            self.get_logger().warn(
                'navigate_to_pose action server not available yet.',
                throttle_duration_sec=5.0
            )

            return

        # --------------------------------------------------------
        # Find valid goal
        # --------------------------------------------------------

        global_goal, reason = self.find_valid_goal()

        if global_goal is None:

            self.get_logger().warn(
                'No valid known/free goal found between '
                f'{MIN_LOOKAHEAD_DISTANCE:.1f} m and '
                f'{LOOKAHEAD_DISTANCE:.1f} m ahead.',
                throttle_duration_sec=2.0
            )

            return

        # --------------------------------------------------------
        # Extract coordinates
        # --------------------------------------------------------

        gx = global_goal.pose.position.x
        gy = global_goal.pose.position.y

        # --------------------------------------------------------
        # Prevent unnecessary goal updates
        # --------------------------------------------------------

        if self.last_goal_xy is not None:

            dx = gx - self.last_goal_xy[0]
            dy = gy - self.last_goal_xy[1]

            distance_from_previous_goal = math.hypot(
                dx,
                dy
            )

            if distance_from_previous_goal < GOAL_UPDATE_MIN_DIST:
                return

        # --------------------------------------------------------
        # Send goal
        # --------------------------------------------------------

        self.send_goal(global_goal)

        self.last_goal_xy = (gx, gy)

    # ============================================================
    # SEND GOAL
    # ============================================================

    def send_goal(self, pose_stamped):

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = pose_stamped

        self.goal_active = True

        send_future = self.nav_client.send_goal_async(
            goal_msg
        )

        send_future.add_done_callback(
            self.goal_response_callback
        )

        self.get_logger().info(
            f'Valid goal sent: '
            f'({pose_stamped.pose.position.x:.2f}, '
            f'{pose_stamped.pose.position.y:.2f})'
        )

    # ============================================================
    # NAV2 GOAL RESPONSE
    # ============================================================

    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Failed to send goal to Nav2: {e}'
            )

            self.goal_active = False

            return

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Goal was rejected by Nav2.'
            )

            self.goal_active = False

            return

        self.get_logger().info(
            'Goal accepted by Nav2.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.goal_result_callback
        )

    # ============================================================
    # NAV2 RESULT
    # ============================================================

    def goal_result_callback(self, future):

        try:

            result = future.result()

            status = result.status

            self.get_logger().info(
                f'Navigation goal finished with status: {status}'
            )

        except Exception as e:

            self.get_logger().warn(
                f'Failed to get navigation result: {e}'
            )

        # Allow another goal to be generated.
        self.goal_active = False


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = GoalGeneratorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()