# """

# Node just for testing code. IGNORE IT !

# """

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path

from geometry_msgs.msg import PoseStamped
import heapq

from geometry_msgs.msg import PolygonStamped

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import time

from rclpy.qos import QoSProfile, DurabilityPolicy

import os

from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header

import csv

from ament_index_python.packages import get_package_share_directory
import os


EXPANSION_RADIUS = 2
RESOLUTION = 0.05 

# Get the shared directory of the mission_control package
package_share_directory = get_package_share_directory('mission_control')

# Construct the path to workspace_1.tsv
WORKSPACE_FILE_PATH = os.path.join(package_share_directory, 'workspaces', 'workspace_1.tsv')

# -----------------------  AUXILIARY FUNCTIONS -----------------------



def publish_workspace(publisher, clock, file_path=None):
    if file_path:
        coordinates = read_workspace(file_path)
    else:
        coordinates = read_workspace() # Default file path
    polygon = create_polygon(coordinates)
    polygon.header.stamp = clock.now().to_msg()
    publisher.publish(polygon)



def read_workspace(file_path=WORKSPACE_FILE_PATH):
    coordinates = []
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


def inflate_occupied_cells(occupancy_grid, expansion_radius=EXPANSION_RADIUS):
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
                            if data[ny * width + nx] == 0:  # Only inflate free cells
                                data[ny * width + nx] = 50  # Mark as dilated space



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


# Flood fill algorithm to mark free cells
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



def compute_path(start, goal, exploration_occupancy_grid, clock):
    path_points = compute_grid_path(start, goal, exploration_occupancy_grid)
    if not path_points:
        return None, None
    
    path = create_path_message(path_points, clock, exploration_occupancy_grid)
    return path_points, path



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


def create_path_message(path_points, clock, occupancy_grid):
    path_points = simplify_grid_path(path_points)
    # Convert grid coordinates to real-world coordinates
    path_points = grid_to_real_coordinates(path_points, occupancy_grid)

    # Smooth the path using cubic interpolation
    #path_points = bezier_smooth_path(path_points)

    path = Path()
    path.header.stamp = clock.now().to_msg()
    path.header.frame_id = 'map'

    for point in path_points:
        pose = PoseStamped()
        pose.header.stamp = clock.now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0 # Indicates that the orientation of the robot is set to a default, neutral orientation, meaning no rotation 
        path.poses.append(pose)

    return path


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


def real_to_grid_coordinates(real_points, occupancy_grid):
    grid_points = []
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    resolution = occupancy_grid.info.resolution

    for (real_x, real_y) in real_points:
        grid_x = int((real_x - origin_x) / resolution)
        grid_y = int((real_y - origin_y) / resolution)
        grid_points.append((grid_x, grid_y))

    return grid_points




#  ------------ TEST PATH PLANNING ------------

class TestComputePathNode(Node):
    def __init__(self):
        super().__init__('test_compute_path_node')

         # Initialize the static transform broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Broadcast the static transform for the map frame
        self.broadcast_map_frame()

        # Define a shared QoS profile for latched publishers
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL


        # Publisher for the workspace (latched publisher)
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', latched_qos)
        publish_workspace(self.workspace_publisher, self.get_clock())

        self.grid_publisher = self.create_publisher(OccupancyGrid, 'test_occupancy_grid', 10)
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        self.timer = self.create_timer(3.0, self.timer_callback)

        # Initialize the occupancy grid
        self.occupancy_grid = initialize_occupancy_grid()

        # ----- Add some obstacles -----
        (x_grid, y_grid) = real_to_grid_coordinates([(0.5, 0)], self.occupancy_grid)[0]
        self.mark_square(x_grid, y_grid, 0)
    
    

        # Dilate the occupancy grid
        inflate_occupied_cells(self.occupancy_grid, EXPANSION_RADIUS)

        # Define start and goal points (in grid coordinates)
        self.start = real_to_grid_coordinates([(0, 0)], self.occupancy_grid)[0]
        self.goal = real_to_grid_coordinates([(1, 0)], self.occupancy_grid)[0]
    
    
    def broadcast_map_frame(self):
        # Create a static transform for the map frame
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'  # Parent frame
        transform.child_frame_id = 'map'    # Child frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.static_tf_broadcaster.sendTransform(transform)


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
        _, path = compute_path(self.start, self.goal, self.occupancy_grid, self.get_clock())
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




