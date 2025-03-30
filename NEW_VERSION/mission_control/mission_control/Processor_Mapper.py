# Importing the ROS 2 Python client library
import rclpy  
from rclpy.node import Node  # Base class for defining a ROS 2 node
from rclpy.time import Time  # Used for handling ROS 2 timestamps
from rclpy.qos import QoSProfile, DurabilityPolicy


# Importing message types used for communication in ROS 2
from sensor_msgs.msg import LaserScan, PointCloud2  # LaserScan for LiDAR data, PointCloud2 for 3D point clouds
from nav_msgs.msg import OccupancyGrid, MapMetaData  # OccupancyGrid for map representation, MapMetaData for metadata
from std_msgs.msg import Header,Bool  # Standard header message for timestamping and frame information
from geometry_msgs.msg import Pose ,PolygonStamped, Point32

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

import csv

# Importing os for handling file system operations (creating folders, saving files)
import os  

from ament_index_python.packages import get_package_share_directory  

from detection_interfaces.msg import DetectionMsg

from shapely import Point,Polygon as ShapelyPolygon



# -------- Tunable Parameters --------
#TODO ensure right file path is used
#WORKSPACE_FILE_PATH = os.path.join(get_package_share_directory('mission_control'), 'workspaces', 'workspace_2.tsv')  # Path to the workspace file
WORKSPACE_FILE_PATH = os.path.join(
    os.getenv("HOME"), "test_ws", "src", "g2_robot", "detection", "resource", "workspace_2.tsv"
)
RESOLUTION = 0.05  # Grid cell size in meters per cell
EXPANSION_RADIUS = 1  # Number of cells to expand occupied areas (for better visualization)
SCAN_THRESHOLD = 5  # Number of scans to skip before processing new data
SCAN_FREQUENCY = 20  # Frequency at which the map is published and saved
NTH_SCAN = 5  # Process every Nth scan
DISTANCE_FILTER = 0.40  # Minimum distance filter for LiDAR points to avoid noise
MAX_CONFIDENCE = 100  # Maximum confidence value
CONFIDENCE_STEP = 100  # Step increase in confidence per scan (25%, 50%, 75%, 100%)
MAP_FOLDER = "maps"  # Directory where generated maps will be stored
# ------------------------------------

#TODO
#CHECK world to map coordinates


def create_polygon(coordinates):
    polygon = PolygonStamped()
    polygon.header = Header()
    polygon.header.frame_id = "map"
    for coord in coordinates:
        point = Point32(x=coord[0], y=coord[1], z=0.0)
        polygon.polygon.points.append(point)
    return polygon


class MapBuilder:
    def __init__(self, resolution=RESOLUTION, folder=MAP_FOLDER):
        self.resolution = resolution  
        self.folder = folder  
        os.makedirs(self.folder, exist_ok=True)
        
        
         # Initialize default map size, dynamically updated later
        self.size_x = 500  
        self.size_y = 500
        self.origin_x = 0
        self.origin_y = 0
        self.map = None  # Main occupancy map
        self.confidence_map = None  # Confidence levels per cell
        
        #self.map = 205 * np.ones((size, size), dtype=np.uint8)  # Initialize as unknown (gray)
        #self.confidence_map = np.zeros((size, size), dtype=np.uint8)  # Track confidence (0-100)

    def initialize_map(self, min_x, max_x, min_y, max_y):
        """ Initializes the occupancy grid based on the workspace size. """
        self.size_x = int((max_x - min_x) / self.resolution)+1# Add buffer
        self.size_y = int((max_y - min_y) / self.resolution)+1
        self.origin_x = min_x
        self.origin_y = min_y

        # Initialize maps: Occupancy is unknown (-1), confidence starts at 0
        self.map = 0 * np.ones((self.size_y, self.size_x), dtype=np.int8)  # Using -1 for unknown, 0 for known cells
        self.confidence_map = np.zeros((self.size_y, self.size_x), dtype=np.uint8)  # Confidence map [0-100]


    def world_to_map(self, x, y):
        """ Converts real-world coordinates (meters) to map indices. """
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my
    
    def map_to_world(self, mx, my):
        """ Converts real-world coordinates (meters) to map indices. """
        x = mx*self.resolution + self.origin_x
        y = my*self.resolution + self.origin_y
        return x, y


    def add_scan(self, scan_data):
        """ Processes new LiDAR scan data, updates the occupancy grid, and applies dilation. """
        if not scan_data:
            return

        map_points = []  # Store map indices for dilation

        for x, y in scan_data:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.size_x and 0 <= my < self.size_y:
                # Increase confidence, capped at MAX_CONFIDENCE
                self.confidence_map[my, mx] = min(self.confidence_map[my, mx] + CONFIDENCE_STEP, MAX_CONFIDENCE)
                self.map[my, mx] = int(self.confidence_map[my, mx])  # Directly use confidence as occupancy
                map_points.append((x, y))  # Store world coordinates for dilation

        # Apply dilation to the entire scan
        self.apply_dilation(map_points)        

    def efficient_add_scan(self, scan_data):
        """Processes new LiDAR scan data, updates the occupancy grid, and applies dilation using NumPy."""
        if not scan_data:
            return

        # Convert world coordinates to map indices
        scan_data = np.array(scan_data)  # Convert list to NumPy array for vectorized operations
        mx, my = self.world_to_map(scan_data[:, 0], scan_data[:, 1])  # Vectorized coordinate conversion

        # Filter valid indices that are within map bounds
        valid_mask = (0 <= mx) & (mx < self.size_x) & (0 <= my) & (my < self.size_y)
        mx, my = mx[valid_mask], my[valid_mask]  # Keep only valid indices

        # Update confidence map using NumPy's indexing
        self.confidence_map[my, mx] = np.minimum(self.confidence_map[my, mx] + CONFIDENCE_STEP, MAX_CONFIDENCE)

        # Update occupancy grid based on confidence values
        self.map[my, mx] = self.confidence_map[my, mx]  # Directly assign confidence to occupancy grid

        # Convert back to world coordinates if apply_dilation needs them
        affected_points = [(x, y) for x, y in zip(scan_data[valid_mask, 0], scan_data[valid_mask, 1])]

        # Apply dilation on affected points
        self.apply_dilation(affected_points)


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
        grid.info.origin.position.z = 0.0
        
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

    
    def apply_dilation(self, points, reverse=False):
        """ Applies dilation or reverse dilation to the given set of points in the occupancy map. """
        if not points:
            return

        # Create a binary mask of the same size as the map
        mask = np.zeros_like(self.map, dtype=np.uint8)

        # Set the provided points as occupied in the mask
        for x, y in points:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.size_x and 0 <= my < self.size_y:
                mask[my, mx] = 255  # Mark occupied pixels


        # Define the dilation kernel (3x3 kernel for expanding by 1 cell)
        kernel = np.ones((3, 3), np.uint8)

        # Define the dilation kernel (11x11 kernel for expanding by 5 cells)
        kernel = np.ones((11, 11), np.uint8)

        # Apply either dilation or reverse dilation (erosion) based on the reverse flag
        if reverse:
            # Erode (reverse dilation) the mask to shrink the effect
            processed_mask = cv2.erode(mask, kernel, iterations=1)
        else:
            # Dilate the mask to expand the effect
            processed_mask = cv2.dilate(mask, kernel, iterations=1)

        # Use NumPy boolean indexing for map updates based on the processed mask
        if reverse:
            # Reverse dilation: shrink affected cells back to unknown
            dilated_cells = processed_mask > 0
            self.map[dilated_cells] = np.where(self.map[dilated_cells] == 50, -1, self.map[dilated_cells])  # Reset to unknown
            self.confidence_map[dilated_cells] = np.where(self.map[dilated_cells] == -1, 0, self.confidence_map[dilated_cells])  # Reset confidence to 0
        else:
            # Regular dilation: expand affected cells to 50% occupancy
            dilated_cells = processed_mask > 0
            self.map[dilated_cells] = np.where(self.map[dilated_cells] != 100, 50, self.map[dilated_cells])  # Mark dilated areas with 50% occupancy
            self.confidence_map[dilated_cells] = np.where(self.map[dilated_cells] != 100, 50, self.confidence_map[dilated_cells])  # Set confidence to 50


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Define a shared QoS profile for latched publishers
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
         #Subs to necesary topics

        #Mapping item callbacks
        #Basic Lidar callback
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        #ICP corrected single cloud callback, should be used to create an accumulated cloud
        self.corrected_subscriber = self.create_subscription(PointCloud2, '/nth_corrected_pointcloud', self.corrected_callback, 10)
        #ICP corrected accumulated cloud callback, used to trigger map generation based on udated accumulated cloud
        self.final_map_subscriber = self.create_subscription(PointCloud2, '/corrected_accumulated_pointcloud', self.final_map_callback, 10)
        #Object addition and removal callbacks
        self.addition_subscriber = self.create_subscription(DetectionMsg, '/add_object', self.add_obj_callback, 10)
        self.removal_subscriber = self.create_subscription(DetectionMsg, '/remove_object', self.remove_obj_callback, 10)


        #self.marker_subscriber = self.create_subscription(Marker, '/visualization_marker', self.marker_callback, 10)
        # Create map_trigger subscriber to trigger the map creation
        self.trigger_subscriber = self.create_subscription(Bool, '/map_trigger', self.map_trigger_callback, 10)

        #occupancy gridmap publisher
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_map', 10)
        #accumulated and simple pointcloud publishers
        self.accumulated_publisher = self.create_publisher(PointCloud2, '/uncorrected_accumulated_pointcloud', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/nth_simple_pointcloud', 10)
        # Publisher for the workspace (latched publisher)
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', latched_qos)
        


        # TF Buffer and Listener for correcting odometry drift
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_builder = MapBuilder()
        self.proj = LaserProjection()
        self.accumulated_points = []

        self.scan_count = 0
        self.nth_scan = SCAN_THRESHOLD
        self.scan_freq = SCAN_FREQUENCY
        self.d_filter= DISTANCE_FILTER
        self.ld_filter = 6
        self.First_time=True

    
        # Load workspace boundaries from TSV file
        self.vertices = self.read_tsv(WORKSPACE_FILE_PATH)
        self.publish_workspace()


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

        # Apply distance-based filtering
        bad_points= ranges > self.ld_filter  # Only keep points within the maximum range
        ranges[bad_points]= np.inf

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

            #TODO Ensure the bellow properly adds points
            if self.First_time:
                self.get_logger().info(f'Published 1st scan as PointCloud2')

                # Read points from the message
                points = list(pc2.read_points(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True))

                # Voxelize the point cloud before adding to the map
                #Voxelize will also filter out points out of the workspace
                voxel_size = 0.1  # Adjust voxel size for desired resolution
                voxelized_points = self.voxelize(points, voxel_size)

                # Reset accumulated points to ICP-corrected ones
                self.map_builder.add_scan(voxelized_points)

                # Publish the occupancy grid map every scan_freq scans
                occupancy_grid = self.map_builder.to_occupancy_grid(msg.header.stamp)
                self.grid_publisher.publish(occupancy_grid)
                self.get_logger().info("Published initial occupancy grid map.")

                # Store occupancy map as PGM
                self.map_builder.save_map()
                
                self.First_time = False
            else:
                self.First_time = False
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
        # Filter points to keep only those inside the workspace
        filtered_points = [(x, y) for x, y, _ in points if self.is_point_inside_workspace(x, y)]
        self.map_builder.add_scan(filtered_points)

        #Publish the occupancy grid map every scan_freq scans
        occupancy_grid = self.map_builder.to_occupancy_grid(msg.header.stamp)
        self.grid_publisher.publish(occupancy_grid)
        self.get_logger().info("Published occupancy grid map.")
        #Store occupancy map pgm
        self.map_builder.save_map()
        self.scan_count = 0  


    def voxelize(self, points, voxel_size=0.1):
        """ Reduce point cloud density by grouping points into 2D grid cells and filtering by workspace. """
        voxels = {}
        for x, y, z in points:
            vx, vy = int(x / voxel_size), int(y / voxel_size)

            # Ensure the point is inside the workspace before adding
            if not self.is_point_inside_workspace(x, y):
                continue  # Skip points outside the workspace

            if (vx, vy) not in voxels:
                voxels[(vx, vy)] = (x, y)  # Store only one point per voxel

        return list(voxels.values())

    def add_obj_callback(self, msg):
        """ Adds a point to the occupancy map and applies dilation. """
        x, y = msg.x, msg.y  # Get the x, y coordinates of the object

        # Convert the world coordinates to map coordinates
        mx, my = self.map_builder.world_to_map(x, y)

        # Ensure the point is within the map bounds
        if 0 <= mx < self.map_builder.size_x and 0 <= my < self.map_builder.size_y:
            # Increase the confidence of the cell to simulate adding the object
            self.map_builder.confidence_map[my, mx] = min(self.map_builder.confidence_map[my, mx] + CONFIDENCE_STEP, MAX_CONFIDENCE)
            self.map_builder.map[my, mx] = int(self.map_builder.confidence_map[my, mx])  # Set occupancy level

            # Apply dilation on the newly added point
            self.map_builder.apply_dilation([(x, y)])
            self.map_make_pub(msg.stamp)

    def remove_obj_callback(self, msg):
        """ Removes a point from the occupancy map and applies reverse dilation. """
        x, y = msg.x, msg.y  # Get the x, y coordinates of the object

        # Convert the world coordinates to map coordinates
        mx, my = self.map_builder.world_to_map(x, y)

        # Ensure the point is within the map bounds
        if 0 <= mx < self.map_builder.size_x and 0 <= my < self.map_builder.size_y:
            # If the confidence is greater than 0, reduce the confidence value
            if self.map_builder.confidence_map[my, mx] > 0:
                self.map_builder.confidence_map[my, mx] = max(self.map_builder.confidence_map[my, mx] - CONFIDENCE_STEP, 0)
                self.map_builder.map[my, mx] = int(self.map_builder.confidence_map[my, mx])

            # Apply reverse dilation only if the cell is at 50% occupancy
            if self.map_builder.map[my, mx] == 50:
                self.map_builder.apply_reverse_dilation([(x, y)],reverse=True)  # Reverse dilation on the removed point
            self.map_make_pub(msg.stamp)
    
    def map_trigger_callback(self, msg):
        """ Publishes the map when triggered by an external signal. """
        self.get_logger().info('Received map trigger message.')
        self.map_make_pub(self,msg.header.stamp)
    
    def map_make_pub(self,stamp):
        """ Publishes the map when triggered by an external signal. """
        self.get_logger().info('Generate and publish map.')

        # Here, integrate the corrected cloud into an occupancy grid map
        # Update the map with the new points (you may call add_scan or process ICP here)
        # Filter points to keep only those inside the workspace
        points=self.accumulated_points
        #ensure only points inside workspace are used
        filtered_points = [(x, y) for x, y, _ in points if self.is_point_inside_workspace(x, y)]

        self.map_builder.add_scan(filtered_points)

        occupancy_grid = self.map_builder.to_occupancy_grid(stamp)
        self.grid_publisher.publish(occupancy_grid)
        self.get_logger().info('Published occupancy grid map after trigger.')
        self.map_builder.save_map()


    def is_point_inside_workspace(self, x, y):
        """ Checks if a point is within the defined workspace. """
        if not self.vertices:
            return True  # No workspace limits
        polygon = ShapelyPolygon(self.vertices)
        return polygon.contains(Point(x, y))

    def marker_callback(self, msg):
        """ Callback to handle visualization marker messages and update the map """
        if msg.type == Marker.POINTS:  # Assuming it's a shape with multiple vertices (outline)
            vertices = [(p.x, p.y) for p in msg.points]  # List of vertices (x, y) in world coordinates
            self.update_workspace_boundary(vertices)

            # Optionally, you can fill the inside of the polygon
            # self.fill_inside_polygon(vertices)

            self.get_logger().info(f"Added workspace boundary with {len(msg.points)} vertices")

    def read_tsv(self, file_path):
        vertices = []
        with open(file_path, mode="r") as file:
            reader = csv.reader(file, delimiter="\t")
            next(reader)  # Skip header row
            for row in reader:
                x, y = float(row[0]) / 100, float(row[1]) / 100
                vertices.append((x, y))
            if vertices:
                min_x = min(p[0] for p in vertices)
                max_x = max(p[0] for p in vertices)
                min_y = min(p[1] for p in vertices)
                max_y = max(p[1] for p in vertices)
            self.map_builder.initialize_map(min_x, max_x, min_y, max_y)
            self.update_workspace_boundary(vertices)
            self.get_logger().info(f"Added workspace boundary with {len(vertices)} vertices")
        
        return vertices
        
    def update_workspace_boundary(self, vertices):
        """ Mark the grid cells along the boundary of the workspace and outside"""
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
        """ Draw a line between two points (x1, y1) and (x2, y2) on the occupancy grid, ensure everything else is 0 """
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy


        while True:
            # Mark the current point as occupied
            self.map_builder.map[y1, x1] = 100  # Occupied
            
            try:
                self.map_builder.map[y1+1, x1+1] = 50  # Dial
            except:
                self.map_builder.map[y1-1, x1-1] = 50  # Dial

            self.map_builder.confidence_map[y1, x1] = 100  # Max confidence
            try:
                self.map_builder.confidence_map[y1+1, x1+1] = 50  # Dial
            except:
                self.map_builder.confidence_map[y1-1, x1-1] = 50  # Dial

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
        self.workspace_publisher.publish(polygon)

        #iterate through map and remove free points outside of boundary
        # Create a meshgrid for all combinations of x, y
        X, Y = np.meshgrid(np.linspace(0, self.map_builder.size_x - 1, self.map_builder.size_x), 
                           np.linspace(0, self.map_builder.size_y - 1, self.map_builder.size_y))

        # Flatten the meshgrid to 1D arrays
        X_flat = X.flatten()
        Y_flat = Y.flatten()

        # Create an empty mask with the same shape as the map
        mask = np.zeros((self.map_builder.size_y, self.map_builder.size_x), dtype=bool)

        # Iterate over all indices and check if they are inside the boundary
        for x, y in zip(X_flat, Y_flat):
            # Convert map index to world coordinates
            real_x, real_y = self.map_builder.map_to_world(x, y)

            # Check if the world coordinates are inside the boundary
            if not(self.is_point_inside_workspace(real_x, real_y)):
                mask[int(y), int(x)] = True  # Mark the corresponding cell as valid
        
        self.map_builder.map[mask]=100
        self.map_builder.confidence_map[mask]=100


def main():
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
