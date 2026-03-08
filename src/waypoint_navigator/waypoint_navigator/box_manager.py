import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import math
import time


class BoxManager(Node):

    def __init__(self):
        super().__init__('box_manager')

        # Updated pickup locations with 4th point added
        self.pickup_locations = [
            (5.37, -0.34),    # pickup 1
            (-0.87, -5.65),   # pickup 2
            (-0.87, -5.65),   # pickup 3 (duplicate - will stack at same location)
            (-8.57, -3.58)    # pickup 4
        ]

        # Updated drop zone
        self.drop_zone = (-2.81, 3.17)

        self.pickup_trigger_radius = 1.05
        self.drop_trigger_radius = 0.75

        self.active_boxes = []
        self.delivered_count = 0
        self.delivered_boxes = []
        self.current_pose = None

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Marker publisher for labels
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.5, self.check_proximity)
        self.create_timer(2.0, self.publish_markers)
        self.create_timer(60.0, self.cleanup_delivered_boxes)

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete service...')

        self.spawn_initial_boxes()
        self.get_logger().info("Box Manager Started with updated locations")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def spawn_initial_boxes(self):
        for idx, location in enumerate(self.pickup_locations):
            box_name = f"cardboard_box_{idx}"
            # Stack boxes 2 and 3 at same location with height offset
            if idx == 0 or idx == 3:
                z_offset = 0.10
            elif idx == 1:
                z_offset = 0.10
            else:  # idx == 2
                z_offset = 0.41
            
            self.spawn_box(box_name, location[0], location[1], z_offset, static_model=True)
            self.active_boxes.append({'name': box_name, 'location': location, 'picked': False})
            self.get_logger().info(f"Spawned {box_name} at {location}")

    def spawn_box(self, name, x, y, z, static_model=True):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.get_box_sdf(static_model=static_model)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        request.initial_pose = pose
        request.reference_frame = "world"

        future = self.spawn_client.call_async(request)
        return future

    def delete_box(self, name):
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_client.call_async(request)
        return future

    def check_proximity(self):
        if self.current_pose is None:
            return

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        for box in self.active_boxes:
            if not box['picked']:
                bx, by = box['location']
                distance = math.hypot(rx - bx, ry - by)
                if distance < self.pickup_trigger_radius:
                    self.get_logger().info(f"Robot near {box['name']}, removing box")
                    self.delete_box(box['name'])
                    box['picked'] = True

        drop_distance = math.hypot(rx - self.drop_zone[0], ry - self.drop_zone[1])
        picked_boxes = [b for b in self.active_boxes if b['picked']]
        if drop_distance < self.drop_trigger_radius and len(picked_boxes) > self.delivered_count:
            self.spawn_stacked_boxes()

    def spawn_stacked_boxes(self):
        box_name = f"delivered_box_{self.delivered_count}"
        z_height = 0.10 + (self.delivered_count * 0.29)
        self.spawn_box(box_name, self.drop_zone[0], self.drop_zone[1], z_height, static_model=True)
        
        self.delivered_boxes.append({
            'name': box_name,
            'spawn_time': time.time()
        })
        
        self.delivered_count += 1
        self.get_logger().info(f"Delivered box {self.delivered_count} at drop zone")

    def cleanup_delivered_boxes(self):
        """Remove delivered boxes after 60 seconds"""
        current_time = time.time()
        boxes_to_remove = []
        
        for box in self.delivered_boxes:
            if current_time - box['spawn_time'] > 60.0:
                self.get_logger().info(f"Cleaning up {box['name']} (>60s old)")
                self.delete_box(box['name'])
                boxes_to_remove.append(box)
        
        for box in boxes_to_remove:
            self.delivered_boxes.remove(box)

    def publish_markers(self):
        """Publish text markers for pickup spots and drop zone"""
        marker_array = MarkerArray()
        
        # Pickup markers (avoid duplicate markers at same location)
        unique_locations = {}
        for idx, location in enumerate(self.pickup_locations):
            loc_key = location
            if loc_key not in unique_locations:
                unique_locations[loc_key] = []
            unique_locations[loc_key].append(idx + 1)
        
        marker_id = 0
        for location, pickup_nums in unique_locations.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = location[0]
            marker.pose.position.y = location[1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            if len(pickup_nums) > 1:
                marker.text = f"Pickups {','.join(map(str, pickup_nums))}"
            else:
                marker.text = f"Pickup {pickup_nums[0]}"
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        # Drop zone marker
        drop_marker = Marker()
        drop_marker.header.frame_id = "map"
        drop_marker.header.stamp = self.get_clock().now().to_msg()
        drop_marker.ns = "waypoints"
        drop_marker.id = 100
        drop_marker.type = Marker.TEXT_VIEW_FACING
        drop_marker.action = Marker.ADD
        
        drop_marker.pose.position.x = self.drop_zone[0]
        drop_marker.pose.position.y = self.drop_zone[1]
        drop_marker.pose.position.z = 0.5
        drop_marker.pose.orientation.w = 1.0
        
        drop_marker.scale.z = 0.3
        drop_marker.color.r = 1.0
        drop_marker.color.g = 0.0
        drop_marker.color.b = 0.0
        drop_marker.color.a = 1.0
        
        drop_marker.text = "DROP ZONE"
        marker_array.markers.append(drop_marker)
        
        self.marker_pub.publish(marker_array)

    def get_box_sdf(self, static_model=True):
        static_str = "true" if static_model else "false"
        return f"""
        <?xml version='1.0'?>
        <sdf version='1.6'>
          <model name='cardboard_box'>
            <static>{static_str}</static>
            <link name='link'>
              <collision name='collision'>
                <geometry><box><size>0.28 0.28 0.28</size></box></geometry>
              </collision>
              <visual name='visual'>
                <geometry><box><size>0.28 0.28 0.28</size></box></geometry>
                <material>
                  <ambient>0.8 0.6 0.4 1</ambient>
                  <diffuse>0.8 0.6 0.4 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """


def main(args=None):
    rclpy.init(args=args)
    node = BoxManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()