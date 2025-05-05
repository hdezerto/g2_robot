import rclpy
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
import time

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import heapq

import numpy as np
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformException

from .occupancy_grid_map import (read_workspace, grid_to_real_coordinates, real_to_grid_coordinates)

import math

# ------------ External functions ------------

def publish_workspace(publisher, clock, file_path=None):
    if file_path:
        coordinates = read_workspace(file_path)
    else:
        coordinates = read_workspace()  # Default file path
    polygon = create_polygon(coordinates)
    polygon.header.stamp = clock.now().to_msg()
    publisher.publish(polygon)


def compute_path(start, goal, exploration_occupancy_grid, clock):
    start_cell, start_real = start
    goal_cell, goal_real = goal

    path_points = compute_grid_path(start_cell, goal_cell, exploration_occupancy_grid)

    if not path_points:
        return None, None

    path = create_path_message(path_points, start_real, goal_real, clock, exploration_occupancy_grid)

    return path_points, path


def get_current_pose(tf_buffer, logger, occupancy_grid):
    """
    Gets the robot's current pose in both real-world and grid coordinates.

    Args:
        tf_buffer (Buffer): The TF2 buffer instance for looking up transforms.
        node (Node): The ROS2 node instance (required for logging).
        occupancy_grid (OccupancyGrid): The occupancy grid used for converting real-world coordinates to grid coordinates.

    Returns:
        tuple: A tuple containing:
            - real_pose (tuple): The real-world coordinates (x, y, yaw).
            - grid_position (tuple): The grid coordinates (x, y).
    """
    try:
        # Lookup the latest available transform from 'map' to 'base_link'
        transform = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=1.0))

        # Extract translation (x, y) and rotation (yaw)
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi  # Normalize yaw to [-pi, pi)

        # Real-world position
        real_pose = (x, y, yaw)

        # Convert to grid coordinates
        grid_position = real_to_grid_coordinates([(x, y, None)], occupancy_grid)[0]

        return real_pose, grid_position

    except TransformException as e:
        logger.error(f"Failed to get current pose: {e}")
        return None, None


def check_collision(path_planning_grid, grid_path, current_grid_position):
    """
    Check if there is a collision along the grid_path starting from the closest point
    to the current_grid_position.

    Args:
        path_planning_grid (OccupancyGrid): The occupancy grid used for path planning.
        grid_path (list of tuples): The planned path in grid coordinates [(x1, y1), (x2, y2), ...].
        current_grid_position (tuple): The robot's current position in grid coordinates (x, y).

    Returns:
        bool: True if a collision is detected, False otherwise.
    """
    if not grid_path:
        return False  # No path to check

    # Find the closest point on the grid_path to the current position
    # This is to account for the fact that the robot might deviate slightly from the path
    closest_point = min(
        grid_path,
        key=lambda point: (point[0] - current_grid_position[0]) ** 2
        + (point[1] - current_grid_position[1]) ** 2)

    # Get the index of the closest point in the grid_path
    start_index = grid_path.index(closest_point)

    # Check for collisions from the closest point onward
    width = path_planning_grid.info.width
    for x, y in grid_path[start_index:]:
        # Convert (x, y) to the corresponding index in the occupancy grid
        index = y * width + x

        # Check if the cell is not free
        if path_planning_grid.data[index] != 0:
            return True  # Collision detected

    return False  # No collision detected


def publish_detections_to_rviz(tf_broadcaster, detected_objects, detected_boxes, clock):
    """
    Publishes detected objects and boxes to RViz as dynamic TFs with corresponding labels embedded in the child_frame_id.

    Args:
        tf_broadcaster (TransformBroadcaster): The TransformBroadcaster instance for publishing TFs.
        detected_objects (list): List of tuples (x, y, category) for detected objects.
        detected_boxes (list): List of tuples (x, y, theta) for detected boxes.
        clock (Clock): The current ROS2 clock instance.
    """

    current_time = clock.now().to_msg()

    # Publish TFs for detected objects
    for idx, (x, y, category) in enumerate(detected_objects):
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "map"
        # C: cube, S: sphere, P: plushie
        label = {1: "C", 2: "S", 3: "P"}.get(category, "?")  # Default to '?' if category is unknown
        transform.child_frame_id = f"obj_{idx}_{label}"  # Include label in the frame ID
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation

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
    diagonal_cost = 1.414  # Cost to move diagonally ~= sqrt(2)

    # Octile distance heuristic
    def heuristic(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (diagonal_cost - 1) * min(dx, dy)

    neighbors = [
        (0, 1),
        (1, 0),
        (0, -1),
        (-1, 0),  # Cardinal directions
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),  # Diagonal directions
    ]
    close_set = set()  # Set of visited cells
    came_from = {}  # Dictionary to store the path
    gscore = {start: 0}  # Cost from start to current cell
    fscore = {start: heuristic(start, goal)}  # Estimated cost from start to goal through current cell (using heuristic)
    oheap = []  # Priority queue to store the cells to visit

    heapq.heappush(oheap, (fscore[start], start))  # Add the start cell to the queue. The queue is ordered by fscore (lowest first)

    while oheap:
        current = heapq.heappop(oheap)[1]  # Get the cell with the lowest fscore

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            # return a_star_backup(start, goal, grid)  # FOR TESTING - remove when no errors in A* backup
            return [start] + data[::-1]  # Return reversed path (start to goal)

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + (diagonal_cost if abs(i) + abs(j) == 2 else 1)

            # Check if the neighbor is not a free cell
            if grid.data[neighbor[1] * grid.info.width + neighbor[0]] != 0:
                continue

            # Check if the neighbor has been visited and if the cost to reach it is higher than the current cost
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float("inf")):
                continue

            # If the cost to reach the neighbor is lower than the current cost or not in the queue, update the path
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return a_star_backup(start, goal, grid)  # Fallback to backup A* if no path is found


def a_star_backup(start, goal, grid):
    """
    A backup implementation of the A* algorithm for pathfinding.
    Allows the robot to move through occupied cells, though with a high penalty.

    Args:
        start (tuple): The starting grid coordinates (x, y).
        goal (tuple): The goal grid coordinates (x, y).
        grid (OccupancyGrid): The occupancy grid used for pathfinding.

    Returns:
        list: A list of grid coordinates representing the path, or False if no path is found.
    """
    open_list = []
    closed_list = set()

    # Initialize the start node
    start_node = (0, start, None)  # (f, position, parent)
    heapq.heappush(open_list, start_node)

    while open_list:
        # Get the node with the lowest f-score
        current_node = heapq.heappop(open_list)
        current_f, current_position, parent = current_node

        if current_position in closed_list:
            continue

        # Add the current position to the closed list
        closed_list.add(current_position)

        # Check if the goal is reached
        if current_position == goal:
            path = []
            while current_node:
                path.append(current_node[1])
                current_node = current_node[2]  # Traverse back using the parent pointer
            return path[::-1]  # Return the path in reverse order (start to goal)

        # Generate neighbors
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, 1), (0, -1)]:  # Cardinal directions
            nx, ny = current_position[0] + dx, current_position[1] + dy
            if 0 <= nx < grid.info.width and 0 <= ny < grid.info.height:
                if (grid.data[ny * grid.info.width + nx] == 0):  # Check if the cell is free
                    neighbors.append((nx, ny))

        for neighbor in neighbors:
            if neighbor in closed_list:
                continue

            # Calculate g, h, and f scores
            g = current_f + 1
            h = math.sqrt((neighbor[0] - goal[0]) ** 2 + (neighbor[1] - goal[1]) ** 2)
            occupancy = grid.data[neighbor[1] * grid.info.width + neighbor[0]]

            # Add a penalty for occupied cells
            if occupancy > 50 or occupancy < 0:
                g += 1000  # Adjust penalty as needed
            elif occupancy != 0:
                g += 100

            f = g + h
            neighbor_node = (f, neighbor, current_node)

            # Add the neighbor to the open list
            heapq.heappush(open_list, neighbor_node)

    return False  # Return False if no path is found


def create_path_message(grid_path_points, start_real, goal_real, clock, occupancy_grid):
    grid_path_points = simplify_grid_path(grid_path_points, occupancy_grid)  # Simplify the path by removing redundant points
    grid_path_points = grid_path_points[1:-1]  # Remove the start and goal points from the path (the exact ones will be added later)

    # Convert path grid coordinates to real-world coordinates
    real_path_points = grid_to_real_coordinates(grid_path_points, occupancy_grid)

    # Add start_real and goal_real as the first and last points
    real_path_points.insert(0, start_real)
    real_path_points.append(goal_real)

    # Smooth the path using cubic interpolation
    # path_points = bezier_smooth_path(path_points)

    path = Path()
    path.header.stamp = clock.now().to_msg()
    path.header.frame_id = "map"

    # print("Path points: ", real_path_points) # DEBUG

    for x, y, _ in real_path_points:
        pose = PoseStamped()
        pose.header.stamp = clock.now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # Indicates that the orientation of the robot is set to a default, neutral orientation, meaning no rotation
        path.poses.append(pose)

    if (goal_real[2] is None):  # If no yaw is provided for the goal, set it to match the direction of the path
        dx = path.poses[-1].pose.position.x - path.poses[-2].pose.position.x
        dy = path.poses[-1].pose.position.y - path.poses[-2].pose.position.y
        yaw = np.arctan2(dy, dx)
    else:  # Yaw is given, useful for collection
        yaw = goal_real[2]

    q = quaternion_from_euler(0, 0, yaw)
    (path.poses[-1].pose.orientation.x,
     path.poses[-1].pose.orientation.y,
     path.poses[-1].pose.orientation.z,
     path.poses[-1].pose.orientation.w) = q

    return path


def simplify_grid_path(path_points, occupancy_grid):
    if not path_points:
        return []

    simplified_path = [path_points[0]]
    for i in range(1, len(path_points) - 1):
        prev_point = simplified_path[-1]  # x1, y1
        curr_point = path_points[i]  # x2, y2
        next_point = path_points[i + 1]  # x3, y3

        # Check if the current point is redundant (i.e., lies on a straight line)
        # (x3-x2)*(y2-y1) != (y3-y2)*(x2-x1)
        if (next_point[0] - curr_point[0]) * (curr_point[1] - prev_point[1]) != (next_point[1] - curr_point[1]) * (curr_point[0] - prev_point[0]):
            simplified_path.append(curr_point)

    simplified_path.append(path_points[-1])
    return simplify_further(simplified_path, occupancy_grid)


def simplify_further(path_points, occupancy_grid):
    whole_path = path_points
    new_path = whole_path
    j = 0
    try:
        while j < len(whole_path) - 1:
            for i in range(j + 1, len(whole_path)):
                if path_valid(whole_path[j], whole_path[i], occupancy_grid):
                    new_path = whole_path[: j + 1] + whole_path[i:]
                else:
                    break
            whole_path = new_path
            j += 1
    except IndexError:
        pass
    return new_path


def bresenham_line(x0, y0, x1, y1):
    """Bresenham's Line Algorithm — returns list of cells from (x0,y0) to (x1,y1)."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells


def path_valid(start, end, occupancy_grid):
    """
    Check if a straight line between start and end passes through any occupied cells.
    :param occupancy_grid: nav_msgs.msg.OccupancyGrid
    :param start: tuple (x0, y0) grid coordinates
    :param end: tuple (x1, y1) grid coordinates
    :return: True if line is free, False if any cell is occupied
    """
    width = occupancy_grid.info.width
    data = occupancy_grid.data

    if data[start[1] * width + start[0]] != 0 or data[end[1] * width + end[0]] != 0:
        return False

    # Bresenham's line algorithm to get all cells between start and end
    for x, y in bresenham_line(*start, *end):
        idx = y * width + x
        if data[idx] != 0:  # Threshold can be adjusted
            return False
    return True


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
