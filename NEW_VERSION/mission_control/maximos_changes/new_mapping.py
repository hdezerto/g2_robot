import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node  
from rclpy.time import Time  
from sensor_msgs.msg import LaserScan, PointCloud2  
from nav_msgs.msg import OccupancyGrid, MapMetaData  
from std_msgs.msg import Header, Bool  
from geometry_msgs.msg import Pose ,PolygonStamped
from laser_geometry import LaserProjection  
import sensor_msgs_py.point_cloud2 as pc2  
from tf2_ros import Buffer, TransformListener  
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud  
from ament_index_python.packages import get_package_share_directory  
import numpy as np  
import cv2 
import csv 
import os  
 
from shapely.geometry import Point, Polygon  

#internal imports
from mission_control_utils import create_polygon

# -------- Tunable Parameters --------
WORKSPACE_FILE_PATH = os.path.join(get_package_share_directory('mission_control'), 'workspaces', 'workspace_1.tsv')  # Path to the workspace file
RESOLUTION = 0.05  # Grid cell size in meters per cell
EXPANSION_RADIUS = 2  # Number of cells to expand occupied areas (for better visualization)
SCAN_THRESHOLD = 5  # Number of scans to skip before processing new data
SCAN_FREQUENCY = 10  # Frequency at which the map is published and saved
NTH_SCAN = 5  # Process every Nth scan
DISTANCE_FILTER = 0.40  # Minimum distance filter for LiDAR points to avoid noise
MAX_CONFIDENCE = 100  # Maximum confidence value
CONFIDENCE_STEP = 25  # Step increase in confidence per scan (25%, 50%, 75%, 100%)
MAP_FOLDER = "maps"  # Directory where generated maps will be stored
# ------------------------------------

'''
What is needed for exploration controller to work:
 Why are we using a detected callback inside the exploration loop?, detected objects can simply cause a callback in LidarMapBuilder to update the occupancy map (and publish a new version)
I have made this a simplified mapper that runs as an indendent node. No direct calls from mission controller. Mission controller can simple subscribe to the right topics.
If we need 
'''

class MapBuilder:
    """
    Handles the construction of the occupancy grid map based on LiDAR scans.
    It maintains both the occupancy map and confidence levels.
    """
    def __init__(self, resolution=RESOLUTION, folder=MAP_FOLDER):
        self.resolution = resolution  # Size of each grid cell in meters
        self.folder = folder  # Directory for saving maps
        os.makedirs(self.folder, exist_ok=True)  # Ensure the map directory exists

        # Initialize default map size, dynamically updated later
        self.size_x = 500  
        self.size_y = 500
        self.map = None  # Main occupancy map
        self.confidence_map = None  # Confidence levels per cell

    def initialize_map(self, min_x, max_x, min_y, max_y):
        """ Initializes the occupancy grid based on the workspace size. """
        self.size_x = int((max_x - min_x) / self.resolution) + 10  # Add buffer
        self.size_y = int((max_y - min_y) / self.resolution) + 10
        self.origin_x = min_x
        self.origin_y = min_y

        # Initialize maps: Occupancy is unknown (-1), confidence starts at 0
        self.map = -1 * np.ones((self.size_y, self.size_x), dtype=np.int8)  # Using -1 for unknown cells
        self.confidence_map = np.zeros((self.size_y, self.size_x), dtype=np.uint8)  # Confidence map [0-100]

    def world_to_map(self, x, y):
        """ Converts real-world coordinates (meters) to map indices. """
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def add_scan(self, scan_data):
        """ Processes new LiDAR scan data into the occupancy grid. """
        for x, y in scan_data:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.size_x and 0 <= my < self.size_y:
                # Increase confidence, capped at MAX_CONFIDENCE
                self.confidence_map[my, mx] = min(self.confidence_map[my, mx] + CONFIDENCE_STEP, MAX_CONFIDENCE)
                
                # Convert confidence to occupancy: Higher confidence = higher occupancy probability
                # For example: confidence of 100 -> map value = 100 (fully occupied), confidence of 50 -> map value = 50 (partially occupied)
                self.map[my, mx] = int(self.confidence_map[my, mx])  # Directly using confidence as occupancy

    def save_map(self, filename="map.pgm"):
        """ Saves the map as a grayscale PGM file. """
        filepath = os.path.join(self.folder, filename)
        grayscale_map = self.map_to_grayscale(self.map)
        cv2.imwrite(filepath, grayscale_map)
        print(f"Map saved at {filepath}")

    def to_occupancy_grid(self, stamp):
        """ Converts the internal map to an ROS OccupancyGrid message. """
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"

        grid.info = MapMetaData()
        grid.info.resolution = self.resolution
        grid.info.width = self.size_x
        grid.info.height = self.size_y
        grid.info.origin = Pose()
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y

        # Ensure that the map is in the proper ROS format (0 = free, 100 = occupied, -1 = unknown)
        ros_map = self.map.flatten().tolist()
        
        # Convert confidence to occupancy: If confidence is > 0, set map value to 100 (occupied)
        # If it's still -1, keep as unknown
        #ros_map = [100 if value > 0 else value for value in ros_map]
        
        grid.data = ros_map
        return grid

    def map_to_grayscale(self, occupancy_map):
        """ Converts the occupancy map in range [-1, 100] to grayscale [0, 255]. """
        # Rescale occupancy values to be in the range [0, 255] (ignore -1 for unknown cells)
        return np.clip((occupancy_map + 1) * (255 / 101), 0, 255).astype(np.uint8)


class LidarMapBuilder(Node):
    """
    ROS Node that subscribes to LiDAR scans, processes them, and publishes
    an occupancy grid map.
    """
    def __init__(self):
        super().__init__('lidar_map_builder')


        # Define a shared QoS profile for latched publishers
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.trigger_subscriber = self.create_subscription(Bool, '/map_trigger', self.map_trigger_callback, 10)
        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_map', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/accumulated_pointcloud', 10)
        
        # Publisher for the workspace (latched publisher)
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', latched_qos)

        self.map_builder = MapBuilder()
        self.scan_count = 0  # Counter for scan frequency control

        # TF buffer for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.proj = LaserProjection()
        self.accumulated_points = []

        # Load workspace boundaries from TSV file
        self.vertices = self.read_tsv(WORKSPACE_FILE_PATH)
        if self.vertices:
            min_x = min(p[0] for p in self.vertices)
            max_x = max(p[0] for p in self.vertices)
            min_y = min(p[1] for p in self.vertices)
            max_y = max(p[1] for p in self.vertices)
            self.map_builder.initialize_map(min_x, max_x, min_y, max_y)

    def map_trigger_callback(self, msg):
        """ Publishes the map when triggered by an external signal. """
        self.get_logger().info('Received map trigger message.')
        occupancy_grid = self.map_builder.to_occupancy_grid(msg.stamp)
        self.publisher.publish(occupancy_grid)

    def is_point_inside_workspace(self, x, y):
        """ Checks if a point is within the defined workspace. """
        if not self.vertices:
            return True  # No workspace limits
        polygon = Polygon(self.vertices)
        return polygon.contains(Point(x, y))

    '''Working scan callback from the previous version'''
    def scan_callback(self, msg):
        """ Processes incoming LiDAR scans and updates the occupancy grid. """
        self.scan_count += 1
        if self.scan_count % SCAN_THRESHOLD != 0:
            return  # Skip processing for every Nth scan
        
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
        time = rclpy.Time.from_msg(stamp)

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
  
            # Extract (x, y) points
            transformed_points = [
                (p[0], p[1]) for p in pc2.read_points(transformed_cloud, field_names=("x", "y"), skip_nans=True)
            ]

            # Filter points to keep only those inside the workspace
            filtered_points = [(x, y) for x, y in transformed_points if self.is_point_inside_workspace(x, y)]
            self.map_builder.add_scan(filtered_points)

        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")

        if self.scan_count % SCAN_FREQUENCY == 0:
            occupancy_grid = self.map_builder.to_occupancy_grid(msg.header.stamp)
            self.publisher.publish(occupancy_grid)

        if self.scan_count >= SCAN_FREQUENCY:
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
        self.publish_workspace()
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

    def publish_workspace(self, file_path=None):
        if len(self.vertices)!=0:
            coordinates = self.vertices
        else:
            coordinates = self.read_tsv(self.WORKSPACE_FILE_PATH)
        polygon = create_polygon(coordinates)
        polygon.header.stamp = self.get_clock().now().to_msg()
        self.pointcloud_publisher(polygon)

def main():
    rclpy.init()
    node = LidarMapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

