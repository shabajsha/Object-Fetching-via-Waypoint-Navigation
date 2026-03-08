import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
import math


class TaskNavigator(Node):

    def __init__(self):
        super().__init__('task_navigator')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Updated pickup locations from RViz clicked_point
        self.pickups = [
            (-0.89, -5.74),  # pickup 1
            (0.06, 9.22),    # pickup 2
            (6.74, 0.39)     # pickup 3
        ]
        
        # Updated drop zone
        self.drop_zone = (-1.01, 2.76)

        self.current_pose = None
        self.target = None
        self.target_box = None
        self.state = "WAIT_FOR_POSE"
        self.approach_distance = 0.85

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.5, self.mission_loop)

        # Wait for Nav2
        self.get_logger().info("Waiting for Nav2 action server...")
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Nav2 not available!")
            raise RuntimeError("Nav2 not ready")
        
        self.get_logger().info("Task Navigator Started with updated waypoints")
        self.get_logger().info(f"Pickups: {self.pickups}")
        self.get_logger().info(f"Drop zone: {self.drop_zone}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def mission_loop(self):
        if self.current_pose is None:
            return

        if self.state == "WAIT_FOR_POSE":
            self.state = "SELECT_PICKUP"
            self.select_next_pickup()

    def compute_approach_point(self, box_xy):
        bx, by = box_xy
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        dx = bx - rx
        dy = by - ry
        d = math.hypot(dx, dy)

        if d < 1e-3:
            return (bx - self.approach_distance, by)

        ux = dx / d
        uy = dy / d
        return (bx - ux * self.approach_distance, by - uy * self.approach_distance)

    def select_next_pickup(self):
        if self.state != "SELECT_PICKUP" or self.current_pose is None:
            return

        if not self.pickups:
            self.log_mission_status("Mission complete. All pickups serviced.")
            self.state = "FINISHED"
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        self.pickups.sort(key=lambda p: math.hypot(p[0] - rx, p[1] - ry))

        self.target_box = self.pickups.pop(0)
        self.target = self.compute_approach_point(self.target_box)
        self.state = "GO_TO_PICKUP"
        self.send_goal(self.target)

    def send_goal(self, target):
        self.log_mission_status(
            f"Navigating to target ({target[0]:.2f}, {target[1]:.2f})"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.log_mission_status("Goal rejected")
            self.state = "SELECT_PICKUP"
            self.select_next_pickup()
            return

        self.log_mission_status("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.log_mission_status("Navigation failed. Selecting next pickup.")
            self.state = "SELECT_PICKUP"
            self.select_next_pickup()
            return

        if self.state == "GO_TO_PICKUP":
            self.log_mission_status("Pickup reached. Object picked.")
            self.state = "GO_TO_DROP"
            self.send_goal(self.drop_zone)

        elif self.state == "GO_TO_DROP":
            self.log_mission_status("Drop zone reached. Object delivered.")
            self.state = "SELECT_PICKUP"
            self.select_next_pickup()

    def log_mission_status(self, message):
        if self.current_pose:
            rx = round(self.current_pose.position.x, 2)
            ry = round(self.current_pose.position.y, 2)
            self.get_logger().info(
                f"[MISSION] {message} | Pose=({rx}, {ry}) | Remaining={len(self.pickups)}"
            )
        else:
            self.get_logger().info(f"[MISSION] {message}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()