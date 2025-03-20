import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from mission_control_utils import read_workspace
import numpy as np
import tf2_ros
import tf_transformations
import copy



# -------- Tunable parameters --------
RESOLUTION = 0.05  # Grid cell size [m/cell]
LIDAR_MIN_RANGE = 0.4  # Minimum range to consider a valid measurement [m]
SCAN_THRESHOLD = 5  # Number of scans to skip before processing 
# ------------------------------------



# ----------------- External functions -----------------

def initialize_occupancy_grid(resolution=RESOLUTION):
    coordinates = read_workspace()
    
    # Determine the bounds of the workspace
    min_x = min(coord[0] for coord in coordinates)
    min_y = min(coord[1] for coord in coordinates)
    max_x = max(coord[0] for coord in coordinates)
    max_y = max(coord[1] for coord in coordinates)
    
    # Set the origin of the occupancy grid to the minimum coordinates
    origin_x = min_x
    origin_y = min_y
    
    # Calculate the width and height of the occupancy grid
    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1
    
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info.resolution = resolution  # [meters per cell]
    occupancy_grid.info.width = width  # Calculated width [cells]
    occupancy_grid.info.height = height  # Calculated height [cells]
    occupancy_grid.info.origin.position.x = origin_x
    occupancy_grid.info.origin.position.y = origin_y
    occupancy_grid.info.origin.position.z = 0.0
    occupancy_grid.info.origin.orientation.w = 1.0 # No rotation
    occupancy_grid.data = [-1] * (occupancy_grid.info.width * occupancy_grid.info.height)  # Initialize with unknown values
    
    # Mark the lines connecting the vertices as occupied space
    for i in range(len(coordinates)):
        x1, y1 = coordinates[i]
        x2, y2 = coordinates[(i + 1) % len(coordinates)] # The modulo operator allows to connect the last vertex with the first one
        mark_line_as_occupied(occupancy_grid, x1, y1, x2, y2) # Mark the line as occupied space
    
    # Set the cells inside the workspace as free cells.
    # (0,0) in map coordinates is the starting point since it is always inside the workspace
    mark_free_cells(occupancy_grid, 0, 0)

    return occupancy_grid





# ----------------- Internal functions (auxiliary) -----------------
def mark_line_as_occupied(occupancy_grid, x1, y1, x2, y2):
    # Bresenham's line algorithm to mark the line as occupied
    def bresenham(x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    # Convert coordinates to grid indices
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    resolution = occupancy_grid.info.resolution

    grid_x1 = int((x1 - origin_x) / resolution)
    grid_y1 = int((y1 - origin_y) / resolution)
    grid_x2 = int((x2 - origin_x) / resolution)
    grid_y2 = int((y2 - origin_y) / resolution)
    
    # Get the points on the line
    line_points = bresenham(grid_x1, grid_y1, grid_x2, grid_y2)
    
    # Mark the points as occupied in the occupancy grid
    width = occupancy_grid.info.width
    data = occupancy_grid.data 
    for (grid_x, grid_y) in line_points:
        index = grid_y * width + grid_x
        data[index] = 100  # Mark as occupied space


def mark_free_cells(occupancy_grid, start_x, start_y):
    # Convert start coordinates to grid indices
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    resolution = occupancy_grid.info.resolution

    grid_x = int((start_x - origin_x) / resolution)
    grid_y = int((start_y - origin_y) / resolution)
    
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = occupancy_grid.data
    
    # Flood fill algorithm to mark free cells
    stack = [(grid_x, grid_y)]
    while stack:
        x, y = stack.pop()
        index = y * width + x
        if data[index] == -1:  # If the cell is unknown
            data[index] = 0  # Mark as free space
            # Add neighboring cells to the stack
            if x > 0:
                stack.append((x - 1, y))
            if x < width - 1:
                stack.append((x + 1, y))
            if y > 0:
                stack.append((x, y - 1))
            if y < height - 1:
                stack.append((x, y + 1))




# ----------------- OccupancyGridMap class -----------------

class OccupancyGridMap(Node):
    def __init__(self):
        super().__init__('OccupancyGridMap_node')
        self.map_publisher = self.create_publisher(OccupancyGrid, '/lidar_occupancy_grid', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize the clean occupancy grid with the workspace file
        # This grid will not be modified, only copied to reset the map when needed (due to noise), avoiding to repeat computations
        self.clean_occupancy_grid = initialize_occupancy_grid()
        # Make a copy of the clean grid for lidar updates
        self.occupancy_grid = OccupancyGrid()
        self.copy_clean_grid()

        # Publish the occupancy grid (just for first visualization)
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg() # Use the current time
        self.map_publisher.publish(self.occupancy_grid)

        # Add a counter for processing scans
        self.scan_counter = 0


    def copy_clean_grid(self):
        """ Copy the clean occupancy grid to the current occupancy grid """
        self.occupancy_grid = copy.deepcopy(self.clean_occupancy_grid)


    # CHECK THIS FUNCTION
    def scan_callback(self, msg):
        # Increment the scan counter
        self.scan_counter += 1
        # Only process the scan if the counter reaches the threshold
        if self.scan_counter < SCAN_THRESHOLD:
            return
        # Reset the counter after processing
        self.scan_counter = 0

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)
        
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        data = np.array(self.occupancy_grid.data, dtype=np.int8)  # Avoid unnecessary conversions

        # Lookup the transformation from lidar_link to map frame
        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=1.0))
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Could not transform {msg.header.frame_id} to map: {e}')
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'Transformation extrapolation error: {e}')
            return

        # Filter valid range values
        valid_mask = (ranges >= LIDAR_MIN_RANGE) & (ranges <= msg.range_max) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]

        # Precompute angles efficiently
        valid_indices = np.where(valid_mask)[0]  # Get valid indices
        angles = angle_min + valid_indices * angle_increment

        # Compute Lidar points in the Lidar frame
        x_lidar = valid_ranges * np.cos(angles)
        y_lidar = valid_ranges * np.sin(angles)

        # Create a batch of points in homogeneous coordinates
        points = np.vstack((x_lidar, y_lidar, np.zeros_like(x_lidar), np.ones_like(x_lidar)))

        # Convert TF transform to a matrix
        transform_matrix = self.transform_to_matrix(transform.transform)

        # Apply the transformation (efficient batch processing)
        points_map = transform_matrix @ points  

        # Compute grid indices
        grid_x = ((points_map[0] - origin_x) / resolution).astype(int)
        grid_y = ((points_map[1] - origin_y) / resolution).astype(int)

        # Filter valid indices to avoid out-of-bounds errors
        valid_grid_mask = (0 <= grid_x) & (grid_x < width) & (0 <= grid_y) & (grid_y < height)
        grid_x, grid_y = grid_x[valid_grid_mask], grid_y[valid_grid_mask]

        # Convert to flat indices safely
        flat_indices = np.ravel_multi_index((grid_y, grid_x), (height, width))

        # Mark occupied cells only if they are initially free
        for index in flat_indices:
            if data.flat[index] == 0:  # Check if the cell is initially free
                data.flat[index] = 100  # Mark as occupied

        # Publish the updated occupancy grid
        self.occupancy_grid.data = data.flatten().tolist()
        self.occupancy_grid.header.stamp = msg.header.stamp
        self.map_publisher.publish(self.occupancy_grid)


    # CHECK THIS FUNCTION
    def transform_to_matrix(self, transform):
        """ Convert a TransformStamped message to a 4x4 transformation matrix """
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        rotation = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        
        # Get the full 4×4 transformation matrix from the quaternion
        transform_matrix = tf_transformations.quaternion_matrix(rotation)
        
        # Insert the translation into the last column
        transform_matrix[:3, 3] = translation  
        
        return transform_matrix  # Return full 4×4 matrix



# ----------------- Main function -----------------
def main(args=None):
    rclpy.init(args=args)
    mapping_node = OccupancyGridMap()
    mapping_node.get_logger().info('Mapping node has been created.')

    try:
        rclpy.spin(mapping_node)
    except Exception as e:
        mapping_node.get_logger().error(f'An error occurred: {e}')
    finally:
        mapping_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()