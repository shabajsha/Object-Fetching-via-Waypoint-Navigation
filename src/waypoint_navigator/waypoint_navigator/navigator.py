import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0)
        ]

        self.current_goal_index = 0
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        x, y = self.waypoints[self.current_goal_index]
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def result_callback(self, future):
        self.get_logger().info('Goal reached!')
        
        # Simulate pickup verification
        self.get_logger().info('Checking for object...')
        self.get_logger().info('Object confirmed!')

        self.current_goal_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()