# Importing the ROS 2 Python client library
import rclpy  
from rclpy.node import Node  # Base class for defining a ROS 2 node
from rclpy.time import Time  # Used for handling ROS 2 timestamps

# Importing message types used for communication in ROS 2
from sensor_msgs.msg import LaserScan, PointCloud2  # LaserScan for LiDAR data, PointCloud2 for 3D point clouds
from nav_msgs.msg import OccupancyGrid, MapMetaData  # OccupancyGrid for map representation, MapMetaData for metadata
from std_msgs.msg import Header  # Standard header message for timestamping and frame information
from geometry_msgs.msg import Pose, TransformStamped  # Pose for position and orientation, TransformStamped for coordinate transformations

# Importing libraries for LiDAR scan conversion
from laser_geometry import LaserProjection  # Converts LaserScan messages into 3D PointCloud2 format
import sensor_msgs_py.point_cloud2 as pc2  # Utilities for working with PointCloud2 messages in Python

# Importing libraries for handling coordinate transformations
from tf2_ros import Buffer, TransformListener  # Buffer for storing transformations, TransformListener for receiving TF data
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud  # Applies transformations to point clouds

from ament_index_python.packages import get_package_share_directory

# Importing NumPy for numerical computations and array handling
import numpy as np  

# Importing OpenCV for image processing (used to save maps as PGM images)
import cv2  

# Importing os for handling file system operations (creating folders, saving files)
import os  

import csv

class MapBuilder:
    def __init__(self, resolution=0.1, size=500, folder="maps"):
        self.resolution = resolution  # 10 cm per pixel
        self.size = size  # pixels (map will be size x size)
        self.map = 205 * np.ones((size, size), dtype=np.uint8)  # Initialize as unknown (gray)
        self.confidence_map = np.zeros((size, size), dtype=np.uint8)  # Track confidence (0-100)
        self.folder = folder  
        os.makedirs(self.folder, exist_ok=True)
        

    def world_to_map(self, x, y):
        """ Convert real-world (meters) coordinates to map indices """
        mx = int(self.size / 2 + x / self.resolution)
        my = int(self.size / 2 + y / self.resolution)
        return mx, my

    def add_scan(self, scan_data):
        """ Process LiDAR scan data into the occupancy grid """
        for x, y in scan_data:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.size and 0 <= my < self.size:
                # Increase confidence by 25% each hit, max 100%
                self.confidence_map[my, mx] = min(self.confidence_map[my, mx] + 25, 100)

                # Convert confidence to an occupancy probability (0 = free, 100 = occupied)
                self.map[my, mx] = int(205 - (self.confidence_map[my, mx] * 2.05))  # Scale 100 → 0, 0 → 205


    def save_map(self, filename="map.pgm"):
        """ Save map as a PGM file """
        filepath = os.path.join(self.folder, filename)
        cv2.imwrite(filepath, self.map)
        print(f"Map saved at {filepath}")

    def to_occupancy_grid(self, stamp):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"

        grid.info = MapMetaData()
        grid.info.resolution = self.resolution
        grid.info.width = self.size
        grid.info.height = self.size
        grid.info.origin = Pose()
        grid.info.origin.position.x = - (self.size / 2) * self.resolution
        grid.info.origin.position.y = - (self.size / 2) * self.resolution
        grid.info.origin.position.z = 0.0
        
        grid.data = (self.confidence_map - 205).astype(np.int8).flatten().tolist()
        return grid

    

class LidarMapBuilder(Node):
    def __init__(self):
        super().__init__('lidar_map_builder')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Create map_trigger subscriber to trigger the map creation
        self.trigger_subscriber = self.create_subscription(bool,'/map_trigger',self.map_trigger_callback,10)

        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_map', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/accumulated_pointcloud', 10)
        self.map_builder = MapBuilder()
        self.scan_count = 0
        self.scan_freq = 20  # Number of scans before saving map
        self.nth_scan = 5  # Accumulate every Nth scan
        self.d_filter = 0.40  # Distance filter for LiDAR points

        # TF Buffer and Listener for correcting odometry drift
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.proj = LaserProjection()
        self.accumulated_points = []  # List to store accumulated LiDAR points

        #workspace viz
        self.ws_file= os.path.join(get_package_share_directory('mission_control'), 'workspaces', 'workspace_1.tsv')
         
        self.vertices = self.read_tsv(self.ws_file)

    def map_trigger_callback(self, msg):
        """ Callback function that is called when a message is received on '/map_trigger' """
        self.get_logger().info('Received map trigger message.')

        # Directly call the map builder to create the occupancy grid map using the message timestamp
        occupancy_grid = self.map_builder.to_occupancy_grid(msg.stamp)
        self.grid_publisher.publish(occupancy_grid)
        self.get_logger().info('Published occupancy grid map after trigger.')


    def scan_callback(self, msg):
        
        self.scan_count += 1  

        # Only process every Nth scan
        if self.scan_count % self.nth_scan != 0:
            return  
        
        stamp = msg.header.stamp
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        # Convert angles to degrees for easier filtering
        # Convert angles to degrees for easier filtering
        angles_degrees = np.degrees(angles)

        # Create a mask for points behind the robot
        behind_robot = (angles_degrees < -120) | (angles_degrees > 120)
        
        ranges[behind_robot & (ranges < self.d_filter)] = np.inf


        if len(ranges) == 0:
            self.get_logger().warn("All scan points filtered out!")
            return  # Exit early if no valid points remain

        # Modify the original msg with the filtered values
        msg.ranges = ranges.tolist()
        #msg.angle_min = np.min(angles)  # Update min angle
        #msg.angle_max = np.max(angles)  # Update max angle


        # Define frames and get the transform
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id
        time = Time.from_msg(stamp)

        # Transform scan data to correct odometry drift
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel, 
                from_frame_rel,
                time,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Convert LaserScan to PointCloud2
            cloud = self.proj.projectLaser(msg)
              # Transform the point cloud
            transformed_cloud = do_transform_cloud(cloud, transform)
            
            transformed_points = np.array([
                [p[0], p[1]] for p in pc2.read_points(transformed_cloud, field_names=("x", "y"), skip_nans=True)
            ], dtype=np.float32)

            transformed_filtered_points = transformed_points

            # Aggregate and publish every Nth transformed scan as a PointCloud2
            
            point_cloud_points = pc2.read_points(transformed_cloud, field_names=("x", "y","z"), skip_nans=True)
            for point in point_cloud_points:
                self.accumulated_points.append([point[0], point[1], point[2]])


            # Add every Nth transformed scan to the accumulated point cloud
            
            #self.accumulated_points.extend(transformed_points)

            self.publish_accumulated_pointcloud(msg.header)

            # Update the map using the accumulated point cloud
            self.map_builder.add_scan(transformed_filtered_points)

        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")

        #Publish the occupancy grid map every scan_freq scans
        if self.scan_count % self.scan_freq == 0:  # Publish every scan frequency count
            occupancy_grid = self.map_builder.to_occupancy_grid(stamp)
            self.publisher.publish(occupancy_grid)
            self.get_logger().info("Published occupancy grid map.")

        if self.scan_count >= self.scan_freq:  # Save every scan frequency count
            self.map_builder.save_map()
            self.scan_count = 0  

    def publish_accumulated_pointcloud(self, header):
        """ Publishes the accumulated LiDAR point cloud """
        if len(self.accumulated_points) == 0:
            return
        
        header.frame_id = "map"  # Ensure the point cloud is in the map frame
        point_cloud_msg = pc2.create_cloud_xyz32(header, np.array(self.accumulated_points, dtype=np.float32))
        self.pointcloud_publisher.publish(point_cloud_msg)
        self.get_logger().info(f"Published accumulated point cloud with {len(self.accumulated_points)} points")
        #Read vertices from tvs file

    def read_tsv(self, file_path):
        vertices = []
        with open(file_path, mode="r") as file:
            reader = csv.reader(file, delimiter="\t")
            next(reader)  # Skip header row
            for row in reader:
                x, y = float(row[0]) / 100, float(row[1]) / 100
                vertices.append((x, y))
            self.update_workspace_boundary(vertices)
            self.get_logger().info(f"Added workspace boundary with {len(vertices)} vertices")
    
        return vertices
        
    def update_workspace_boundary(self, vertices):
        """ Mark the grid cells along the boundary of the workspace """
        for i in range(len(vertices)):
            # Get two consecutive points to form an edge
            p1 = vertices[i]
            p2 = vertices[(i + 1) % len(vertices)]  # Wrap around to the first point

            # Convert the world coordinates (p1, p2) to map indices
            mx1, my1 = self.map_builder.world_to_map(p1[0], p1[1])
            mx2, my2 = self.map_builder.world_to_map(p2[0], p2[1])

            # Mark the cells along the edge (this is a simplified line segment marking)
            self.draw_line_on_map(mx1, my1, mx2, my2)
    def draw_line_on_map(self, x1, y1, x2, y2):
        """ Draw a line between two points (x1, y1) and (x2, y2) on the occupancy grid """
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            # Mark the current point as occupied
            self.map_builder.map[y1, x1] = 0  # Occupied
            self.map_builder.confidence_map[y1, x1] = 100  # Max confidence

            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

def main():
    rclpy.init()
    node = LidarMapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
