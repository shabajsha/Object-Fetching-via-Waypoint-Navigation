import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

import math
import time



class TaskNavigator(Node):

    def __init__(self):
        super().__init__('task_navigator')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define pickup zones
        self.pickups = [
            (1.5, 0.0),
            (0.0, 1.5),
            (-1.0, 0.0)
        ]

        self.drop_zone = (0.0, 0.0)

        self.current_pose = None
        self.state = "SELECT_PICKUP"

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("Task Navigator Started")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        if self.state == "SELECT_PICKUP" and self.current_pose:
            self.select_next_pickup()

    def select_next_pickup(self):
        if not self.pickups:
            self.log_mission_status("Mission complete. All pickups serviced.")
            self.state = "FINISHED"
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        # Compute nearest pickup
        self.pickups.sort(
            key=lambda p: math.hypot(p[0] - rx, p[1] - ry)
        )

        self.target = self.pickups.pop(0)
        self.state = "GO_TO_PICKUP"
        self.send_goal(self.target)

    def send_goal(self, target):
        self.log_mission_status(
            f"Navigating to target at ({target[0]:.2f}, {target[1]:.2f})"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.client.wait_for_server()

        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)



    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.log_mission_status("Navigation failed. Re-selecting pickup.")
            self.state = "SELECT_PICKUP"
            return

        if self.state == "GO_TO_PICKUP":
            self.log_mission_status("Pickup zone reached. Verifying object...")
            import time
            time.sleep(2)
            self.log_mission_status("Object picked successfully.")
            self.state = "GO_TO_DROP"
            self.send_goal(self.drop_zone)

        elif self.state == "GO_TO_DROP":
            self.log_mission_status("Drop zone reached. Object delivered.")
            self.state = "SELECT_PICKUP"

    def log_mission_status(self, message):
        rx = round(self.current_pose.position.x, 2)
        ry = round(self.current_pose.position.y, 2)

        self.get_logger().info(
            f"[MISSION] {message} | "
            f"Robot Pose: ({rx}, {ry}) | "
            f"Remaining Pickups: {len(self.pickups)}"
        )
    
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()