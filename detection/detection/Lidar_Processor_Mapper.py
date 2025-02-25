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

#Importing library for handling marker
from visualization_msgs.msg import Marker 


# Importing NumPy for numerical computations and array handling
import numpy as np  

# Importing OpenCV for image processing (used to save maps as PGM images)
import cv2  

# Importing os for handling file system operations (creating folders, saving files)
import os  


class MapBuilder:
    def __init__(self, resolution=0.1, size=200, folder="maps"):
        self.resolution = resolution  
        self.size = size  
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
        for x, y, _ in scan_data:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.size and 0 <= my < self.size:
                self.confidence_map[my, mx] = min(self.confidence_map[my, mx] + 25, 100)
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


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/nth_simple_pointcloud', 10)

        self.corrected_subscriber = self.create_subscription(PointCloud2, '/nth_corrected_pointcloud', self.corrected_callback, 10)

        self.accumulated_publisher = self.create_publisher(PointCloud2, '/uncorrected_accumulated_pointcloud', 10)

        self.final_map_subscriber = self.create_subscription(PointCloud2, '/corrected_accumulated_pointcloud', self.final_map_callback, 10)

        self.grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_map', 10)

        self.marker_subscriber = self.create_subscription(Marker, '/visualization_marker', self.marker_callback, 10)
        
        # TF Buffer and Listener for correcting odometry drift
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_builder = MapBuilder()
        self.proj = LaserProjection()
        self.scan_count = 0
        self.nth_scan = 5
        self.accumulated_points = []
        self.scan_freq = 50
        self.d_filter= 0.4
    
    def scan_callback(self, msg):

        self.scan_count += 1
        if self.scan_count % self.nth_scan != 0:
            return
        
        stamp = msg.header.stamp
        ranges = np.array(msg.ranges)  # Get the range data from the scan message
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))  # Calculate the angles
        
        # Create a mask for points behind the robot and filter them out
        angles_degrees = np.degrees(angles)
        behind_robot = (angles_degrees < -120) | (angles_degrees > 120)
        ranges[behind_robot & (ranges < self.d_filter)] = np.inf  # Set filtered points to infinity

        # If there are no valid points, return early
        if len(ranges) == 0:
            self.get_logger().warn("All scan points filtered out!")
            return
        
        # Modify the LaserScan message with the filtered ranges
        msg.ranges = ranges.tolist()

        # Get the TF transform for correcting odometry drift
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id
        time = Time.from_msg(stamp)

        try:
            # Lookup transform to 'map' frame
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel, 
                from_frame_rel,
                time,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Convert the LaserScan message to PointCloud2
            cloud = self.proj.projectLaser(msg)
            # Transform the point cloud to the 'map' frame
            transformed_cloud = do_transform_cloud(cloud, transform)
            # Publish the accumulated point cloud
            self.pointcloud_publisher.publish(transformed_cloud)
            self.get_logger().info(f'Published nth scan as PointCloud2')

        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")
        

    def corrected_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.accumulated_points.extend(points)
        
        if self.scan_count % self.scan_freq == 0:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = "map"
            
            self.get_logger().info(f'accumulated points type:{np.dtype(self.accumulated_points[0])}')
            points_array = np.array([(p[0], p[1], p[2]) for p in self.accumulated_points], dtype=np.float32)
            accumulated_cloud = pc2.create_cloud_xyz32(header, points_array)

            self.accumulated_publisher.publish(accumulated_cloud)
            self.get_logger().info('Published accumulated PointCloud2')
            self.accumulated_points = []  # Reset accumulation

    def final_map_callback(self, msg):
        self.get_logger().info('Received corrected accumulated point cloud for map building')
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        #reset accumulated points to icp corrected ones
        self.accumulated_points.extend(points)
        
        # Here, integrate the corrected cloud into an occupancy grid map
        # Update the map with the new points (you may call add_scan or process ICP here)
        self.map_builder.add_scan(points)

        #Publish the occupancy grid map every scan_freq scans
        occupancy_grid = self.map_builder.to_occupancy_grid(msg.header.stamp)
        self.grid_publisher.publish(occupancy_grid)
        self.get_logger().info("Published occupancy grid map.")
        #Store occupancy map pgm
        self.map_builder.save_map()
        self.scan_count = 0  
    
    def marker_callback(self, msg):
        """ Callback to handle visualization marker messages and update the map """
        if msg.type == Marker.POINTS:  # Assuming it's a shape with multiple vertices (outline)
            vertices = [(p.x, p.y) for p in msg.points]  # List of vertices (x, y) in world coordinates
            self.update_workspace_boundary(vertices)

            # Optionally, you can fill the inside of the polygon
            # self.fill_inside_polygon(vertices)

            self.get_logger().info(f"Added workspace boundary with {len(msg.points)} vertices")
    
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
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
