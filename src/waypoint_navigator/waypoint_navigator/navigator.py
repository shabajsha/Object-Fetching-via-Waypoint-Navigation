import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
import math
import threading
import time


class TaskNavigator(Node):
    def __init__(self):
        super().__init__('task_navigator')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Define pickup jobs (location + how many boxes at that location)
        self.pickup_jobs = [
            {"xy": (5.374265193939209, -0.33860430121421814), "count": 1},
            {"xy": (-0.869442343711853, -5.6549482345581055), "count": 1},
            {"xy": (-8.57079792022705, -3.57749342918396), "count": 1},
        ]

        self.drop_zone = (-2.811201333999634, 3.1657333374023438)
        self.current_pose = None
        self.state = "WAIT_FOR_POSE"
        self.current_job_idx = None
        self.path_costs = {}
        self.computing_paths = False
        self.goal_handle = None
        
        # Early completion distance threshold
        self.early_completion_distance = 0.6

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.5, self.mission_loop)
        self.create_timer(0.2, self.check_early_completion)  # Check proximity frequently

        self.get_logger().info("Waiting for Nav2 action server...")
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("Nav2 action server not available")

        self.get_logger().info("Navigator started with early completion at 0.6m")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def mission_loop(self):
        if self.current_pose is None:
            return

        if self.state == "WAIT_FOR_POSE":
            self.state = "SELECT_PICKUP"
            self.select_next_pickup_async()

    def check_early_completion(self):
        """Check if robot is within 0.6m of target, if so complete navigation"""
        if self.current_pose is None:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        # Check pickup completion
        if self.state == "GO_TO_PICKUP" and self.current_job_idx is not None:
            target_xy = self.pickup_jobs[self.current_job_idx]["xy"]
            distance = math.hypot(rx - target_xy[0], ry - target_xy[1])

            if distance < self.early_completion_distance:
                self.get_logger().info(
                    f"Early completion triggered for pickup! Distance: {distance:.2f}m (threshold: {self.early_completion_distance}m)"
                )
                # Cancel goal and move to drop
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.goal_handle = None

                self.pickup_jobs[self.current_job_idx]["count"] -= 1
                remaining = self.pickup_jobs[self.current_job_idx]["count"]
                self.get_logger().info(f"Box picked early! Remaining at this location: {remaining}")

                self.state = "GO_TO_DROP"
                self.send_goal(self.drop_zone)
                return

        # Check drop completion
        if self.state == "GO_TO_DROP":
            distance = math.hypot(rx - self.drop_zone[0], ry - self.drop_zone[1])

            if distance < self.early_completion_distance:
                self.get_logger().info(
                    f"Early completion triggered for drop! Distance: {distance:.2f}m (threshold: {self.early_completion_distance}m)"
                )
                # Cancel goal and select next pickup
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.goal_handle = None

                self.get_logger().info("Box dropped early! Selecting next pickup...")
                self.state = "SELECT_PICKUP"
                self.select_next_pickup_async()
                return

    def compute_path_length_blocking(self, target_xy):
        """
        Blocking call to compute path length.
        Should be called in a thread, not in main loop.
        """
        if self.current_pose is None:
            return float('inf')

        goal = ComputePathToPose.Goal()
        goal.start = PoseStamped()
        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose = self.current_pose

        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = target_xy[0]
        goal.goal.pose.position.y = target_xy[1]
        goal.goal.pose.orientation.w = 1.0

        goal.use_start = True
        goal.planner_id = "GridBased"

        try:
            self.get_logger().info(f"[PATH THREAD] Computing path to {target_xy}...")

            if not self.path_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(f"[PATH THREAD] Planner unavailable for {target_xy}")
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
                euclidean = math.hypot(target_xy[0] - rx, target_xy[1] - ry)
                self.path_costs[target_xy] = euclidean
                return euclidean

            send_future = self.path_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn(f"[PATH THREAD] Path goal rejected for {target_xy}")
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
                euclidean = math.hypot(target_xy[0] - rx, target_xy[1] - ry)
                self.path_costs[target_xy] = euclidean
                return euclidean

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)

            result = result_future.result()
            if result is None or result.result is None:
                self.get_logger().warn(f"[PATH THREAD] No result for {target_xy}")
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
                euclidean = math.hypot(target_xy[0] - rx, target_xy[1] - ry)
                self.path_costs[target_xy] = euclidean
                return euclidean

            path = result.result.path.poses
            if len(path) < 2:
                self.get_logger().warn(f"[PATH THREAD] Path too short for {target_xy}")
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
                euclidean = math.hypot(target_xy[0] - rx, target_xy[1] - ry)
                self.path_costs[target_xy] = euclidean
                return euclidean

            path_length = 0.0
            for i in range(len(path) - 1):
                p1 = path[i].pose.position
                p2 = path[i + 1].pose.position
                path_length += math.hypot(p2.x - p1.x, p2.y - p1.y)

            self.get_logger().info(f"[PATH THREAD] Path to {target_xy}: {path_length:.2f}m")
            self.path_costs[target_xy] = path_length
            return path_length

        except Exception as e:
            self.get_logger().error(f"[PATH THREAD] Exception: {e}")
            rx = self.current_pose.position.x
            ry = self.current_pose.position.y
            euclidean = math.hypot(target_xy[0] - rx, target_xy[1] - ry)
            self.path_costs[target_xy] = euclidean
            return euclidean

    def select_next_pickup_async(self):
        """Start background thread to compute paths and select pickup"""
        if self.computing_paths:
            return

        thread = threading.Thread(target=self._select_pickup_thread, daemon=True)
        thread.start()

    def _select_pickup_thread(self):
        """Run in background thread - compute all paths, then select"""
        self.computing_paths = True
        self.path_costs.clear()

        active = [(i, job) for i, job in enumerate(self.pickup_jobs) if job["count"] > 0]

        if not active:
            self.get_logger().info("Mission complete. All pickups delivered.")
            self.state = "FINISHED"
            self.computing_paths = False
            return

        self.get_logger().info(f"[PATH THREAD] Computing paths to {len(active)} pickups...")

        for i, job in active:
            target_xy = job["xy"]
            self.compute_path_length_blocking(target_xy)
            time.sleep(0.1)

        best_idx = None
        best_cost = float('inf')

        for i, job in active:
            target_xy = job["xy"]
            cost = self.path_costs.get(target_xy, float('inf'))
            self.get_logger().info(f"  Pickup {i} at {target_xy}: cost = {cost:.2f}m")

            if cost < best_cost:
                best_cost = cost
                best_idx = i

        if best_idx is None:
            self.get_logger().error("Failed to select pickup!")
            self.computing_paths = False
            return

        self.current_job_idx = best_idx
        target = self.pickup_jobs[best_idx]["xy"]
        self.get_logger().info(
            f"[PATH SELECT] Selected pickup {best_idx} at {target} (cost: {best_cost:.2f}m)"
        )

        self.state = "GO_TO_PICKUP"
        self.send_goal(target)
        self.computing_paths = False

    def send_goal(self, target_xy):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_xy[0]
        goal_msg.pose.pose.position.y = target_xy[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending navigation goal: ({target_xy[0]:.2f}, {target_xy[1]:.2f})")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected")
            if self.state == "GO_TO_PICKUP":
                self.state = "SELECT_PICKUP"
                self.select_next_pickup_async()
            elif self.state == "GO_TO_DROP":
                self.send_goal(self.drop_zone)
            return

        self.get_logger().info("Navigation goal accepted")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Navigation failed (status={status})")
            if self.state == "GO_TO_PICKUP":
                self.get_logger().info("Retrying next pickup...")
                self.state = "SELECT_PICKUP"
                self.select_next_pickup_async()
            elif self.state == "GO_TO_DROP":
                self.get_logger().info("Retrying drop zone...")
                self.send_goal(self.drop_zone)
            return

        if self.state == "GO_TO_PICKUP":
            self.get_logger().info("Pickup location reached!")
            self.pickup_jobs[self.current_job_idx]["count"] -= 1
            remaining = self.pickup_jobs[self.current_job_idx]["count"]
            self.get_logger().info(f"Box picked. Remaining at this location: {remaining}")

            self.state = "GO_TO_DROP"
            self.send_goal(self.drop_zone)

        elif self.state == "GO_TO_DROP":
            self.get_logger().info("Drop zone reached!")
            self.state = "SELECT_PICKUP"
            self.select_next_pickup_async()


def main(args=None):
    rclpy.init(args=args)
    node = TaskNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()