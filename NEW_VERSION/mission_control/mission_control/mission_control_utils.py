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

# -------- Tunable parameters --------
WORKSPACE_FILE_PATH = 'workspace_2.tsv'  # Path to the workspace file
# ------------------------------------



# ------------ External functions ------------

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


