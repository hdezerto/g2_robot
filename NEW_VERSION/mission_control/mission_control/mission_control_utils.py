from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
import time

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import heapq

import numpy as np
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

from occupancy_grid_map import read_workspace, grid_to_real_coordinates



# ------------ External functions ------------

def publish_workspace(publisher, get_clock, file_path=None):
    if file_path:
        coordinates = read_workspace(file_path)
    else:
        coordinates = read_workspace() # Default file path
    polygon = create_polygon(coordinates)
    polygon.header.stamp = get_clock.now().to_msg()
    publisher.publish(polygon)


def compute_path(start, goal, exploration_occupancy_grid, get_clock):
    path_points = compute_grid_path(start, goal, exploration_occupancy_grid)
    if not path_points:
        return None, None
    
    path = create_path_message(path_points, get_clock, exploration_occupancy_grid)
    return path_points, path


def publish_detections_to_rviz(tf_broadcaster, detected_objects, detected_boxes, clock):
    """
    Publishes detected objects and boxes to RViz as TFs with corresponding labels embedded in the child_frame_id.

    Args:
        tf_broadcaster (TransformBroadcaster): The TransformBroadcaster instance for publishing TFs.
        detected_objects (list): List of tuples (x, y, category) for detected objects.
        detected_boxes (list): List of tuples (x, y, theta) for detected boxes.
        current_time (Time): The current ROS2 time to use for the TFs.
    """
    current_time = clock.now().to_msg() # current_time = self.get_clock().now().to_msg()
    # Publish TFs for detected objects
    for idx, (x, y, category) in enumerate(detected_objects):
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "map"
        label = {
            '1': 'C',  # Cube
            '2': 'S',  # Sphere
            '3': 'P'   # Plushie
        }.get(category, '?')  # Default to '?' if category is unknown
        transform.child_frame_id = f"obj_{idx}_{label}"  # Include label in the frame ID
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        tf_broadcaster.sendTransform(transform)

    # Publish TFs for detected boxes
    for idx, (x, y, theta) in enumerate(detected_boxes):
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "map"
        transform.child_frame_id = f"box_{idx}"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        # Convert theta (angle in degrees) to quaternion
        quaternion = quaternion_from_euler(0, 0, np.radians(theta))
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        tf_broadcaster.sendTransform(transform)



# ------------ Internal functions (auxiliary) ------------

def create_polygon(coordinates):
    polygon = PolygonStamped()
    polygon.header = Header()
    polygon.header.frame_id = "map"
    for coord in coordinates:
        point = Point32(x=coord[0], y=coord[1], z=0.0)
        polygon.polygon.points.append(point)
    return polygon


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




