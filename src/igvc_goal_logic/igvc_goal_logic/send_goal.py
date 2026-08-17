#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal_pose')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self, x, y):
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: x={x}, y={y}')
        future = self._client.send_goal_async(goal_msg,
                                              feedback_callback=self.feedback_callback)

        future.add_done_callback(self.goal_response_callback)
    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished: {result}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SendGoal()
    node.send(x=2.0, y=1.0)
    rclpy.spin(node)


if __name__ == '__main__':
    main()