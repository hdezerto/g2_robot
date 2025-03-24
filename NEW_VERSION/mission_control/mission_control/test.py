
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path


# from mission_control_utils import compute_path_to_point, dilate_occupied_cells
# from occupancy_grid_map import initialize_occupancy_grid
import time

import csv
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
import time

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import heapq

import numpy as np
from scipy.interpolate import CubicSpline

import os
from ament_index_python.packages import get_package_share_directory

# -------- Tunable parameters --------
WORKSPACE_FILE_PATH = 'workspace_2.tsv'  # Path to the workspace file
# ------------------------------------



EXPANSION_RADIUS = 2

# ------------ External functions ------------

def read_workspace(file_path=WORKSPACE_FILE_PATH):
    coordinates = []
    package_share_directory = get_package_share_directory("mission_control")
    file_path = os.path.join(
        package_share_directory, "resource", "workspace_2.tsv"
    )
    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter='\t')
        next(reader)  # Skip header
        for row in reader:
            x, y = float(row[0]) / 100.0, float(row[1]) / 100.0  # Convert cm to meters
            coordinates.append((x, y))
    return coordinates


def create_polygon(coordinates):
    polygon = PolygonStamped()
    polygon.header = Header()
    polygon.header.frame_id = "map"
    for coord in coordinates:
        point = Point32(x=coord[0], y=coord[1], z=0.0)
        polygon.polygon.points.append(point)
    return polygon


def publish_workspace(publisher, node, file_path=WORKSPACE_FILE_PATH):
    coordinates = read_workspace(file_path)
    polygon = create_polygon(coordinates)
    time.sleep(1)  # Give the publisher time to connect
    polygon.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(polygon)


def dilate_occupied_cells(occupancy_grid, expansion_radius):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = occupancy_grid.data
    
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if data[index] == 100:  # If the cell is occupied
                # Mark the neighboring cells as dilated
                for dy in range(-expansion_radius, expansion_radius + 1):
                    for dx in range(-expansion_radius, expansion_radius + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if data[ny * width + nx] == 0:  # Do not overwrite occupied cells
                                data[ny * width + nx] = 50  # Mark as dilated space


def grid_to_real_coordinates(grid_points, occupancy_grid):
    real_world_points = []
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    resolution = occupancy_grid.info.resolution

    for (x, y) in grid_points:
        real_x = origin_x + x * resolution
        real_y = origin_y + y * resolution
        real_world_points.append((real_x, real_y))

    return real_world_points




def compute_path_to_point(start, goal, exploration_occupancy_grid, get_clock):
    path_points = compute_grid_path(start, goal, exploration_occupancy_grid)

    if not path_points:
        return None

    return create_path_message(path_points, get_clock, exploration_occupancy_grid)



# ------------ Internal functions (auxiliary) ------------

def create_path_message(path_points, get_clock, occupancy_grid):
    path_points = simplify_grid_path(path_points)
    # Convert grid coordinates to real-world coordinates
    path_points = grid_to_real_coordinates(path_points, occupancy_grid)

    # Smooth the path using cubic interpolation
    #path_points = bezier_smooth_path(path_points)

    path = Path()
    path.header.stamp = get_clock().now().to_msg()
    path.header.frame_id = 'map'

    for point in path_points:
        pose = PoseStamped()
        pose.header.stamp = get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0 # Indicates that the orientation of the robot is set to a default, neutral orientation, meaning no rotation 
        path.poses.append(pose)
    
    # Set the orientation of the last waypoint to match the direction of the path
    waypoint_no = len(path.poses)
    print(path.poses[waypoint_no - 1].pose.position.x)

    dx = path.poses[waypoint_no - 1].pose.position.x - path.poses[waypoint_no - 2].pose.position.x
    dy = path.poses[waypoint_no - 1].pose.position.y - path.poses[waypoint_no - 2].pose.position.y
    theta = np.arctan2(dy, dx)
    q = quaternion_from_euler(0, 0, theta)
    path.poses[waypoint_no - 1].pose.orientation.x = q[0]
    path.poses[waypoint_no - 1].pose.orientation.y = q[1]
    path.poses[waypoint_no - 1].pose.orientation.z = q[2]
    path.poses[waypoint_no - 1].pose.orientation.w = q[3]

    return path



# CHECK THIS LATER
def bezier_smooth_path(path_points):
    """Applies cubic interpolation to smooth the path."""
    path_points = np.array(path_points)
    t = np.linspace(0, 1, len(path_points))
    
    x_spline = CubicSpline(t, path_points[:, 0])
    y_spline = CubicSpline(t, path_points[:, 1])

    smooth_t = np.linspace(0, 1, len(path_points) * 10)  # More points for smoothness
    smooth_path = np.stack((x_spline(smooth_t), y_spline(smooth_t)), axis=-1)

    return smooth_path.tolist()





# A* pathfinding algorithm
def compute_grid_path(start, goal, grid):
    diagonal_cost = 1.414 # Cost to move diagonally ~= sqrt(2)
    # Octile distance heuristic
    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (diagonal_cost - 1) * min(dx, dy)
    
    neighbors = [(0, 1),  (1, 0),  (0, -1),  (-1, 0),  # Cardinal directions
                 (1, 1),  (1, -1), (-1, 1), (-1, -1)   # Diagonal directions
                ]
    close_set = set() # Set of visited cells
    came_from = {} # Dictionary to store the path
    gscore = {start: 0} # Cost from start to current cell
    fscore = {start: heuristic(start, goal)} # Estimated cost from start to goal through current cell (using heuristic)
    oheap = [] # Priority queue to store the cells to visit
   

    heapq.heappush(oheap, (fscore[start], start)) # Add the start cell to the queue. The queue is ordered by fscore (lowest first)

    while oheap:
        current = heapq.heappop(oheap)[1] # Get the cell with the lowest fscore

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return [start] + data[::-1] # Return reversed path (start to goal)

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + (diagonal_cost if abs(i) + abs(j) == 2 else 1)

            # Check if the neighbor is not a free cell
            if grid.data[neighbor[1] * grid.info.width + neighbor[0]] != 0:
                continue
            
            # Check if the neighbor has been visited and if the cost to reach it is higher than the current cost
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
            
            # If the cost to reach the neighbor is lower than the current cost or not in the queue, update the path
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False


def simplify_grid_path(path_points):
    if not path_points:
        return []

    simplified_path = [path_points[0]]
    for i in range(1, len(path_points) - 1):
        prev_point = simplified_path[-1] # x1, y1
        curr_point = path_points[i] # x2, y2
        next_point = path_points[i + 1] # x3, y3

        # Check if the current point is redundant (i.e., lies on a straight line)
        # (x3-x2)*(y2-y1) != (y3-y2)*(x2-x1)
        if (next_point[0] - curr_point[0]) * (curr_point[1] - prev_point[1]) != (next_point[1] - curr_point[1]) * (curr_point[0] - prev_point[0]):
            simplified_path.append(curr_point)

    simplified_path.append(path_points[-1])
    return simplified_path


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

#  ------------ TEST PATH PLANNING ------------

class TestComputePathNode(Node):
    def __init__(self):
        super().__init__('test_compute_path_node')

        self.grid_publisher = self.create_publisher(OccupancyGrid, 'test_occupancy_grid', 10)
        self.path_publisher = self.create_publisher(Path, 'test_path', 10)

        self.timer = self.create_timer(3.0, self.timer_callback)

        # Initialize the occupancy grid
        self.occupancy_grid = initialize_occupancy_grid()

        # ----- Add some obstacles -----
        self.mark_square(20, 13, 0)
        self.mark_square(20, 10, 0)
        self.mark_square(20, 4, 0)


        self.mark_square(30, 7, 0)
    

        # Dilate the occupancy grid
        dilate_occupied_cells(self.occupancy_grid, EXPANSION_RADIUS)

        # Define start and goal points (in grid coordinates)
        self.start = (10, 10)
        self.goal = (40, 10)


    def mark_square(self, x, y, size):
        for i in range(-size, size + 1):
            for j in range(-size, size + 1):
                self.occupancy_grid.data[(y + j) * self.occupancy_grid.info.width + (x + i)] = 100

    def timer_callback(self):
        
        # Mark the start and goal points in the occupancy grid
        self.occupancy_grid.data[self.start[1] * self.occupancy_grid.info.width + self.start[0]] = 15
        self.occupancy_grid.data[self.goal[1] * self.occupancy_grid.info.width + self.goal[0]] = 15
        # Publish the occupancy grid (with the start and goal points marked)
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_publisher.publish(self.occupancy_grid)
        # Return the start and goal points to their original values (for the path computation)
        self.occupancy_grid.data[self.start[1] * self.occupancy_grid.info.width + self.start[0]] = 0
        self.occupancy_grid.data[self.goal[1] * self.occupancy_grid.info.width + self.goal[0]] = 0

        # Compute the path
        start_time = time.time() # Measure the time to compute the path
        path = compute_path_to_point(self.start, self.goal, self.occupancy_grid, self.get_clock)
        end_time = time.time()
        computation_time = end_time - start_time

        self.get_logger().info(f'Computed path with {len(path.poses)} waypoints in {computation_time:.4f} seconds')

        if path:
            # Publish the path
            self.path_publisher.publish(path)
            self.get_logger().info('Published path')
        else:
            self.get_logger().info('No path found')



def main(args=None):
    rclpy.init(args=args)
    node = TestComputePathNode()
    node.get_logger().info('Test Compute Path Node has started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
