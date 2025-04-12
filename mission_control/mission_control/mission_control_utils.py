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

from .occupancy_grid_map import read_workspace, grid_to_real_coordinates, real_to_grid_coordinates


# ------------ External functions ------------

def publish_workspace(publisher, clock, file_path=None):
    if file_path:
        coordinates = read_workspace(file_path)
    else:
        coordinates = read_workspace() # Default file path
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
        transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=1.0))

        # Extract translation (x, y) and rotation (yaw)
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Real-world position
        real_pose = (x, y, yaw)

        # Convert to grid coordinates
        grid_position = real_to_grid_coordinates([(x, y)], occupancy_grid)[0]

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
    closest_point = min(grid_path, key=lambda point: (point[0] - current_grid_position[0])**2 + (point[1] - current_grid_position[1])**2)

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
        label = {
            1: 'C',  # Cube
            2: 'S',  # Sphere
            3: 'P'   # Plushie
        }.get(category, '?')  # Default to '?' if category is unknown
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


### ------- Collection ------- ###

def collection_path_planning(
    object_position: tuple,
    current_grid_position: tuple,
    collection_occupancy_grid,
    clock,
) -> tuple[bool, Path]:
    """
    Plan the path to the position for the reobservation of the object.

    1. Compute 12 points around the object with the observation distance. These are the possible positions.
    2. Check if these positions are occupied or don't have a clear view to the object. If so, remove them.
    3. If no possible positions are left, return False.
    4. Compute the path to all possible positions using A* and keep the one with the lowest cost.
    5. Simplify the path and make sure that the orientation of the last waypoint is facing towards the object.
    
    Args:
        object_position (tuple): The (x, y) coordinates of the object to be collected.
        current_grid_position (tuple): The current grid position (x, y) of the robot.
        collection_occupancy_grid: The occupancy grid representing the environment.
        clock: The current time or clock object used for timestamping the path.
    Returns:
        tuple[bool, Path]: A tuple containing a boolean indicating success or failure,
                           and the planned path as a Path message if successful, or None if not.
    """

    # Parameters
    observing_distance = 30
    n = 10 # Number of points on the line between the object and the possible position

    # Init
    object_x, object_y = object_position
    object_grid_position = real_to_grid_coordinates(
        [object_position], collection_occupancy_grid
    )[0]
    possible_positions = []

    # positions are in a circle around the object every 30 degrees
    for i in range(0, 360, 30):
        x = object_x + observing_distance * np.cos(np.radians(i))
        y = object_y + observing_distance * np.sin(np.radians(i))
        possible_positions.append((x, y))
    possible_grid_positions = real_to_grid_coordinates(
        possible_positions, collection_occupancy_grid
    )

    # remove occupied positions or position without a clear view to the object
    for i, grid_position in enumerate(possible_grid_positions):
        if not check_valid_observation_position(collection_occupancy_grid, object_position, possible_positions[i], grid_position):
            possible_positions.pop(i)
            possible_grid_positions.pop(i)
             
    # If no possible positions are left, return False
    if possible_positions:
        return False, None
    
    # Calculate the path to all possible positions and keep the one with the lowest cost
    minimum_cost = float("inf")
    path = []
    final_position = []
    for i, goal_grid_position in enumerate(possible_grid_positions):
        path_points, cost = compute_grid_path(
            current_grid_position,
            goal_grid_position,
            collection_occupancy_grid,
            return_cost=True,
        )
        if cost < minimum_cost:
            minimum_cost = cost
            path = path_points
            final_position = possible_positions[i]
    # if no path to the object is found, return False
    if not path:
        return False, None

    # Create the path message
    path = create_path_message(path_points, clock, collection_occupancy_grid)

    # Correct the final orientation of the robot to face the object
    dx = object_x - final_position[0]
    dy = object_y - final_position[1]
    theta = np.arctan2(dy, dx)
    q = quaternion_from_euler(0, 0, theta)
    path.poses[-1].pose.position.x = final_position[0]
    path.poses[-1].pose.position.y = final_position[1]
    path.poses[-1].pose.orientation.x = q[0]
    path.poses[-1].pose.orientation.y = q[1]
    path.poses[-1].pose.orientation.z = q[2]
    path.poses[-1].pose.orientation.w = q[3]

    return True, path

def check_valid_observation_position(self, collection_occupancy_grid, object_position, possible_position, possible_grid_position, observing_distance=30, n=30):
    """
    Checks whether a given position is valid for observing an object in a grid-based environment.
    1. Check if the possible position is occupied higher than 30%. If yes, remove the possible position.
    2. Check if from the possible positions you have a clear vision of the object.
        Idea:
        - Create n points on the line between the object and the possible position.
        - Convert them to the cell position and check if they are occupied higher than 30%.
        - If any of them is occupied, remove the possible position.
    Args:
        collection_occupancy_grid (OccupancyGrid): The occupancy grid representing the environment. 
            It contains information about which cells are occupied.
        object_position (tuple): The (x, y) coordinates of the object to be observed.
        possible_position (tuple): The (x, y) coordinates of the position to be validated.
        possible_grid_position (tuple): The (x, y) grid cell indices corresponding to the possible position.
    Returns:
        bool: True if the position is valid for observation (not occupied and has a clear line of sight to the object), 
              False otherwise.
    Notes:
        - The function checks if the grid cell corresponding to the possible position is occupied.
        - It also verifies that there is a clear line of sight between the possible position and the object by 
          sampling points along the direct path and ensuring they are not obstructed.
        - The function assumes that the occupancy grid data uses a threshold value (e.g., >30) to indicate obstruction.
    """

    object_x, object_y = object_position
    object_grid_position = real_to_grid_coordinates(
        [object_position], collection_occupancy_grid
    )[0]
    # Check if the position is occupied
    if (
        collection_occupancy_grid.data[
            possible_grid_position[1] * collection_occupancy_grid.info.width
            + possible_grid_position[0]
        ]
        != 0
    ):
        return False
    # Check whether the view is clear
    else:
        # Create a direct path from the position to the object
        direct_path = []
        path_resolution = observing_distance / n

        # Create n points on the line between the object and the possible position
        for j in range(path_resolution, observing_distance, path_resolution):
            x = possible_position[0] + j * np.cos(np.arctan2((object_y - possible_position[1]), (object_x - possible_position[0])))
            y = possible_position[1] + j * np.sin(np.arctan2((object_y - possible_position[1]), (object_x - possible_position[0])))
            direct_path.append((x, y))
        direct_grid_path_init = real_to_grid_coordinates(direct_path, collection_occupancy_grid)
        direct_grid_path = []
        for new_grid_cell in direct_grid_path_init:
            if (not new_grid_cell in direct_grid_path) and (not new_grid_cell == object_grid_position):
                direct_grid_path.append(new_grid_cell)

        for point in direct_grid_path:
            if collection_occupancy_grid.data[
                point[1] * collection_occupancy_grid.info.width + point[0]
            ] > 30:
                return False
        return True  


# CHANGES MADE IN OTHER FUNCTIONS THAN COLLECTION_PATH_PLANNING:
# - compute_grid_path() function was modified to return the cost of the path if return_cost is True.
#
# TODO:
# - Measure parameters observing_distance, pickup_tf_x and pickup_tf_y.



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



def create_path_message(grid_path_points, start_real, goal_real, clock, occupancy_grid):
    grid_path_points = simplify_grid_path(grid_path_points) # Simplify the path by removing redundant points

    grid_path_points = grid_path_points[1:-1] # Remove the start and goal points from the path (the exact ones will be added later)

    # Convert path grid coordinates to real-world coordinates
    real_path_points = grid_to_real_coordinates(grid_path_points, occupancy_grid)

    # Add start_real and goal_real as the first and last points
    real_path_points.insert(0, start_real)
    real_path_points.append(goal_real)

    # Smooth the path using cubic interpolation
    #path_points = bezier_smooth_path(path_points)

    path = Path()
    path.header.stamp = clock.now().to_msg()
    path.header.frame_id = 'map'

    for (x, y) in real_path_points:
        pose = PoseStamped()
        pose.header.stamp = clock.now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 # Indicates that the orientation of the robot is set to a default, neutral orientation, meaning no rotation 
        path.poses.append(pose)

    if goal_real[2] is None: # If no yaw is provided for the goal, set it to match the direction of the path
        dx = path.poses[-1].pose.position.x - path.poses[-2].pose.position.x
        dy = path.poses[-1].pose.position.y - path.poses[-2].pose.position.y
        yaw = np.arctan2(dy, dx)
    else: # Yaw is given, useful for collection
        yaw = goal_real[2]
    
    q = quaternion_from_euler(0, 0, yaw)
    path.poses[-1].pose.orientation.x, path.poses[-1].pose.orientation.y, \
    path.poses[-1].pose.orientation.z, path.poses[-1].pose.orientation.w = q
    
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