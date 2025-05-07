#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PolygonStamped
from .mission_control_utils import (
    publish_workspace,
    compute_path,
    publish_detections_to_rviz,
    get_current_pose,
    check_collision,
)
from .occupancy_grid_map import (
    initialize_occupancy_grid,
    inflate_occupied_cells,
    update_path_planning_grid,
    grid_to_real_coordinates,
    real_to_grid_coordinates,
)
from nav_msgs.msg import OccupancyGrid

from enum import Enum, auto
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from detection_interfaces.msg import DetectionMsg
import os  # To get the current directory

import time
import numpy as np

"""
NOTES (HUGO):
- Fix trapped inside objects
- Fix unable to process more map callbacks and detection at the same time

----- COMMANDS -----:
IN ~/dd2419_ws    rviz2 -d exploration.rviz

--- SSH into the robot ( ssh happy@192.168.128.110 ):
IN ~/dd2419_ws    colcon build --symlink-install
fastdds discovery -i 0 -t 192.168.128.110 -q 42100
ros2 launch g2_robot_launch g2_robot_launch_hardware.xml

ros2 run icp icp_processor
ros2 run mission_control processor_mapper

ros2 run mission_control simple_mapper

ros2 run motion_control motion_control
ros2 run detection detection
IN ~/dd2419_ws    ros2 run mission_control exploration_controller

----- MAP FILE -----:
The map is saved in the directory where the node is run.

"""


# self.get_logger().info('HERE DEBUG!!!')  # DEBUG
# print("-------------------- DEBUG HERE") # DEBUG


# -------- Tunable parameters --------
# EXPLORATION_STEP = 7  # Step size for generating exploration points [cells]
EXPLORATION_STEP = 15  # DEBUGGING
POSITION_THRESHOLD = 0.13  # Threshold for considering two detections as the same [m]
MAP_FILE_NAME = "map_exploration.tsv"  # Name of the map file to save
OBSERVATION_TIME = 3.0  # Time to observe the environment [s]
# ------------------------------------


# ------------------------------- State class -------------------------------
class State(Enum):
    INIT = auto()
    OBSERVING = auto()
    GET_NEXT_EXPLORATION_POINT = auto()
    PLAN_PATH = auto()
    MOVING = auto()
    STUCK_OBSERVING = auto()
    END_EXPLORATION = auto()


# ------------------------------- ExplorationController class -------------------------------
class ExplorationController(Node):

    def run(self):
        while rclpy.ok():
            if self.state == State.OBSERVING:
                self.observing(OBSERVATION_TIME)
            elif self.state == State.GET_NEXT_EXPLORATION_POINT:
                self.get_next_exploration_point()
            elif self.state == State.PLAN_PATH:
                self.plan_path()
            elif self.state == State.MOVING:  # Just process callbacks
                rclpy.spin_once(self)
            elif self.state == State.STUCK_OBSERVING:
                self.stuck_observing(OBSERVATION_TIME)
            elif self.state == State.END_EXPLORATION:
                self.end_exploration()
                break

    # ------------------- Initialization -------------------
    def __init__(self):
        # State to initialize the node
        super().__init__("ExplorationController_node")
        self.state = State.INIT

        # Publishers and subscribers
        latched_qos = QoSProfile(depth=1)  # Define a shared QoS profile for latched publishers
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.workspace_publisher = self.create_publisher(PolygonStamped, "/workspace_polygon", latched_qos)
        self.exploration_grid_publisher = self.create_publisher(OccupancyGrid, "/exploration_occupancy_grid", latched_qos)
        self.detections_subscriber = self.create_subscription(DetectionMsg, "/detections", self.detections_callback, 5)  # CHECK if 5 is not too much here
        self.tf_broadcaster = TransformBroadcaster(self)  # For publishing detected objects/boxes to RViz
        self.mapper_occupancy_grid_subscriber = self.create_subscription(OccupancyGrid, "/mapper_occupancy_grid", self.mapper_occupancy_grid_callback, 1)
        self.planning_grid_publisher = self.create_publisher(OccupancyGrid, "/planning_grid", latched_qos)
        self.path_publisher = self.create_publisher(Path, "/planned_path", 10)
        self.reached_destination_subscriber = self.create_subscription(Bool, "/reached_destination", self.reached_destination_callback, 10)
        self.stop_publisher = self.create_publisher(Bool, "/stop_motion", 10)

        # Initialize TransformListener to get current position of the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)  # spin_thread=True to run the listener in a separate thread

        # Timer to end exploration after a certain time
        self.timer = self.create_timer(300, self.timer_callback)

        # Publish the workspace to RViz
        publish_workspace(self.workspace_publisher, self.get_clock())

        # Initialize exploration grid (workspace file and resolution defined in occupancy_grid_map.py) to:
        # - compute the exploration points
        # - check if the detected objects/boxes are inside the workspace
        self.exploration_occupancy_grid = initialize_occupancy_grid()
        inflate_occupied_cells(self.exploration_occupancy_grid)
        
        #self.exploration_points = self.compute_exploration_points(self.exploration_occupancy_grid, step=EXPLORATION_STEP)
        #self.exploration_points = [(10, 45), (185, 60), (185, 75), (135, 30), (105, 15), (20, 15), (20, 30), (105, 30)] # HARD CODED values (including cabinet)
        #self.exploration_points = [(10, 47), (65, 47), (145, 47), (135, 30), (105, 15), (20, 15), (20, 30), (105, 30)] # HARD CODED values (excluding cabinet)
        #self.exploration_points = [(10,45),(35,26),(59, 26),(50, 45), (185, 60), (190, 60),(190, 75), (185, 60), (135, 30), (105, 15), (20, 15), (20, 30), (105, 30)] # HARD CODED values (including cabinet)

        self.exploration_points = [(10, 45), (190, 55), (185, 78),(189,78), (135, 30), (105, 15), (20, 15), (20, 30), (59, 26),(50, 45)] # HARD CODED values (including cabinet)

        self.mark_exploration_points(self.exploration_occupancy_grid, self.exploration_points)  # Just for DEBUG
        self.publish_exploration_grid()
        # DEBUG:
        # self.get_logger().info(f'Exploration points (grid): {self.exploration_points}')
        # real_world_points = grid_to_real_coordinates(self.exploration_points, self.exploration_occupancy_grid)
        # formatted_real_world_points = [(f"{x:.2f}", f"{y:.2f}") for x, y in real_world_points]
        # self.get_logger().info(f'Exploration points (real world): {formatted_real_world_points}')

        # Initilizate variables for exploration
        self.current_pose = (0.0, 0.0, 0.0)  # Initial position (x, y, yaw) in real world coordinates
        self.current_grid_position = real_to_grid_coordinates([self.current_pose], self.exploration_occupancy_grid)[0]
        self.exploration_point_index = 0
        self.exploration_point = None
        self.detections = []  # Unified list for all detections
        self.detected_objects = []  # List of tuples (x, y, category)
        self.detected_boxes = []  # List of tuples (x, y, theta)
        # Grid where the path will be computed. Obtained by adding the detected objects/boxes to the latest lidar grid.
        self.path_planning_grid = initialize_occupancy_grid()
        inflate_occupied_cells(self.path_planning_grid)

        self.publish_planning_grid()  # Publish the initial grid to RViz
        self.grid_path = []  # Path in grid coordinates
        self.latest_lidar_grid = (initialize_occupancy_grid())  # In case detections_callback is called before the mapper callback OR when testing without lidar
        # Timer to periodically publish detections to RViz
        self.detections_timer = self.create_timer(0.5, self.publish_detections_periodically)

        self.get_logger().info("Waiting 3 sec...")
        time.sleep(3)  # Wait for all nodes to be ready and the TF buffer to populate

        # self.state = State.OBSERVING
        # self.state = State.MOVING # DEBUG detection
        self.state = State.GET_NEXT_EXPLORATION_POINT  # DEBUG motion controller


    # ------------------- STATE FUNCTIONS -------------------
    def observing(self, duration):
        """
        Process callbacks for a specified duration, just for initial observation of the environment (using camera and lidar)
        """
        self.get_logger().info(f"Observing for {duration} seconds...")
        start_time = self.get_clock().now().nanoseconds() / 1e9  # Start time in seconds

        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            rclpy.spin_once(self)

        self.get_logger().info("Finished observing.")
        self.state = State.GET_NEXT_EXPLORATION_POINT # Now it can get the first exploration point


    def stuck_observing(self, duration):
        self.get_logger().info(f"Stuck observing for {duration} seconds...")
        start_time = self.get_clock().now().nanoseconds / 1e9  # Start time in seconds

        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            rclpy.spin_once(self)


    def get_next_exploration_point(self):
        # Get the next exploration point from the list (if it exists)
        if self.exploration_point_index < len(self.exploration_points):
            self.exploration_point = self.exploration_points[self.exploration_point_index]  # Get grid coordinates of the next exploration point
            self.exploration_point_index += 1
            self.state = State.PLAN_PATH
        else:  # No more points to explore
            self.get_logger().info("No more exploration points. Ending exploration.")
            self.state = State.END_EXPLORATION


    # Timer to end exploration after a certain time
    def timer_callback(self):
        # This function is called every 5 minutes to save the map file
        self.state = State.END_EXPLORATION


    def plan_path(self):
        self.update_current_pose()  # Update the current pose of the robot
        start = (self.current_grid_position, self.current_pose)

        # start_yaw_debug = self.current_pose[2] * 180 / np.pi # DEBUG
        # self.get_logger().info(f'----- start_yaw_debug: {start_yaw_debug} degrees')  # DEBUG

        goal = (self.exploration_point, grid_to_real_coordinates([self.exploration_point], self.path_planning_grid)[0])
        self.get_logger().info(f"Start: {start} | Goal: {goal}")  # DEBUG
        self.grid_path, path = compute_path(start, goal, self.path_planning_grid, self.get_clock(), self.get_logger())  # Compute the path in grid coordinates
        if path:  # Path found
            self.path_publisher.publish(path)  # Publish the path to the motion controller and RViz
            self.get_logger().info("Path published. Moving...")
            # --------- JUST TO SEE THE GRID PATH ---------
            self.mark_grid_path(self.exploration_occupancy_grid, self.grid_path)  # DEBUG
            # --------------------------------------
            self.state = State.MOVING
        else:
            self.get_logger().info("No path found. Getting the next exploration point.")
            self.state = State.GET_NEXT_EXPLORATION_POINT


    def detections_callback(self, msg):
        if self.state == State.STUCK_OBSERVING:  # Ignore detections while stuck
            return

        # self.get_logger().info(f'Received detection: {msg.type} (class: {msg.cat}) at ({msg.x}, {msg.y}) with theta {msg.theta}')  # DEBUG
        # Check if the detection is inside the workspace. NOTE: objects that lie on the edge of the workspace are considered outside!
        if not self.is_inside_workspace(msg.x, msg.y):
            self.get_logger().info(f"Detection is outside the workspace. Ignoring it.")
            return
        if self.is_lidar_occupied(msg.x, msg.y):
            self.get_logger().info(f"Detection is inside a lidar occupied cell. Ignoring it.")
            return
        # Update the detections list. It returns true if it's a new detection (for collision check)
        is_new_detection = self.update_detections(msg)
        if is_new_detection:
            # Update path planning grid with the detected object/box and check for collision
            if self.update_path_planning_grid_and_check_collision(self.latest_lidar_grid):
                self.get_logger().info("Collision detected from camera. Recomputing path.")
                self.state = State.PLAN_PATH


    def mapper_occupancy_grid_callback(self, msg):
        # self.get_logger().info('Received new mapper occupancy grid.') # DEBUG
        self.latest_lidar_grid = msg  # Save the latest lidar grid for detections_callback)
        # Update path planning grid with the received lidar occupancy grid and check for collision
        is_collision = self.update_path_planning_grid_and_check_collision(msg)
        if self.state == State.MOVING:
            if is_collision:
                # if self.is_stuck():
                #     self.get_logger().info('Robot is stuck. Waiting for corrected map. MOVING -> STUCK_OBSERVING')
                #     self.state = State.STUCK_OBSERVING
                # else:
                # JULE EDIT - check if new a* algorithm can move robot out of the collision zone
                self.get_logger().info("Collision detected from lidar. Recomputing path.")
                self.state = State.PLAN_PATH

        elif self.state == State.STUCK_OBSERVING:
            if not self.is_stuck():
                self.get_logger().info("Robot is no longer stuck. Recomputing path. STUCK_OBSERVING -> PLAN_PATH")
                self.state = State.PLAN_PATH
            # else it will stay in state STUCK_OBSERVING until the lidar map is corrected


    def reached_destination_callback(self, msg):
        if msg.data:  # msg.data is True if the destination was reached
            self.get_logger().info("Destination reached successfully. Going to the next exploration point.")
            self.state = State.GET_NEXT_EXPLORATION_POINT
        else:
            self.get_logger().info("Failed to reach destination. Going to the next exploration point.")
            self.state = State.GET_NEXT_EXPLORATION_POINT


    def end_exploration(self):
        # Write the map file with detected objects/boxes
        self.write_map_file()
        self.get_logger().info("Exploration completed. Map file saved.")


    # ------------------- UTILS -------------------

    def is_stuck(self):
        # Check if the robot is inside a non-free cell (occupied or inflated) in the path planning grid
        return self.path_planning_grid.data[self.current_grid_position[0] + self.current_grid_position[1] * self.path_planning_grid.info.width] != 0


    def update_path_planning_grid_and_check_collision(self, lidar_occupancy_grid):
        # Update the planning grid with the latest detected objects/boxes
        self.path_planning_grid = update_path_planning_grid(lidar_occupancy_grid, self.detected_objects, self.detected_boxes)
        self.publish_planning_grid()  # Publish the updated grid to RViz
        self.update_current_pose()  # Update the current pose of the robot
        if check_collision(self.path_planning_grid, self.grid_path, self.current_grid_position):
            self.stop_robot()  # Send message to stop the robot
            return True  # Collision detected
        else:
            return False


    def update_current_pose(self):
        # Update the current pose of the robot
        self.current_pose, self.current_grid_position = get_current_pose(self.tf_buffer, self.get_logger(), self.exploration_occupancy_grid)
        if self.current_pose is None:
            self.get_logger().info("Failed to get current pose!")
        # self.get_logger().info(f'Current pose (real): {self.current_pose}  | (grid): {self.current_grid_position}')  # DEBUG


    def update_detections(self, msg):
            # Check if the detection is new or corresponds to an existing one
            detected_list = self.detections  # Unified list for all detections
            for detection in detected_list:
                if ((detection["x"] - msg.x) ** 2 + (detection["y"] - msg.y) ** 2) ** 0.5 < POSITION_THRESHOLD:
                    # Existing detection: update position and add vote
                    detection["x"] = msg.x
                    detection["y"] = msg.y
                    
                    # If the new message is a BOX, update theta and force the winner to BOX
                    if msg.type == "BOX":
                        detection["theta"] = msg.theta
                        detection["votes"].append(msg.type)
                        detection["winner"] = "BOX" # Force winner to BOX
                    # If the new message is an OBJECT
                    else:
                        detection["votes"].append(msg.cat)
                        # Only update the winner if it's not already decided as a BOX
                        if detection["winner"] != "BOX":
                            detection["winner"] = max(set(detection["votes"]), key=detection["votes"].count) # Update winner based on votes
    
                    # self.get_logger().info(f'Updated detection: {detection}')
                    return False  # Existing detection updated
    
            # New detection: add to the list
            new_detection = {
                "x": msg.x,
                "y": msg.y,
                "theta": msg.theta if msg.type == "BOX" else None,
                "votes": ([msg.cat] if msg.type == "OBJECT" else [msg.type]),  # Initialize votes with the category or type
                "winner": (msg.cat if msg.type == "OBJECT" else msg.type)  # Initialize winner with the category or type
            }
            detected_list.append(new_detection)  # detected_list is a list of dictionaries
            # self.get_logger().info(f'Added new detection: {new_detection}')
    
            self.update_detections_lists()  # Update the detected objects and boxes lists
    
            return True  # New detection added


    def update_detections_lists(self):
        """
        Updates the individual lists of detected objects and boxes based on the unified detections list.
        """
        self.detected_objects = []  # Clear the existing list of objects
        self.detected_boxes = []  # Clear the existing list of boxes

        for detection in self.detections:
            if detection["winner"] == "BOX":  # The detection is classified as a box (x, y, theta)
                self.detected_boxes.append((detection["x"], detection["y"], detection["theta"]))
            else:  # The detection is classified as an object (x, y, category)
                self.detected_objects.append((detection["x"], detection["y"], detection["winner"]))


    def is_inside_workspace(self, x, y):
        """
        Check if the given coordinates (x, y) are inside the workspace (including the border).
        """
        # Convert real-world coordinates to grid coordinates
        grid_x, grid_y = real_to_grid_coordinates([(x, y, None)], self.exploration_occupancy_grid)[0]

        # Get the grid dimensions
        width = self.exploration_occupancy_grid.info.width
        height = self.exploration_occupancy_grid.info.height

        # Check if the grid coordinates are within bounds
        if 0 <= grid_x < width and 0 <= grid_y < height:
            # Calculate the index in the grid data
            index = grid_y * width + grid_x
            return self.exploration_occupancy_grid.data[index] != -1  # Return true if not outside the workspace (i.e., not -1)

        # If out of bounds, return False
        return False


    def is_lidar_occupied(self, x, y):
        """
        Check if the given coordinates (x, y) are inside a lidar occupied cell.
        """
        # Convert real-world coordinates to grid coordinates
        grid_x, grid_y = real_to_grid_coordinates([(x, y, None)], self.latest_lidar_grid)[0]

        # Get the grid dimensions
        width = self.latest_lidar_grid.info.width
        height = self.latest_lidar_grid.info.height

        # Check if the grid coordinates are within bounds
        if 0 <= grid_x < width and 0 <= grid_y < height:
            # Calculate the index in the grid data
            index = grid_y * width + grid_x
            # Check if the cell value is 99 (occupied)
            return self.latest_lidar_grid.data[index] == 99

        # If out of bounds, return False
        return False


    def stop_robot(self):
        msg = Bool()
        msg.data = True
        self.stop_publisher.publish(msg)
        time.sleep(1)  # Give time for the motion controller to stop. This avoids it missing the next path.


    def compute_exploration_points(self, occupancy_grid, step):
        exploration_points = []
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = occupancy_grid.data
        line_count = -1  # -1 to ignore the line y=0 (no free cells with workspace2)

        for y in range(0, height, step):  # Iterate over rows with the given step
            leftmost = None
            rightmost = None
            line_count += 1
            for x in range(0, width, step):
                index = y * width + x
                if data[index] == 0:  # Assuming 0 represents free space
                    if leftmost is None:
                        leftmost = (x, y)  # First free cell in the row
                    rightmost = (x, y)  # Update to the last free cell in the row

            # Alternate the order of adding points for zigzag pattern
            if leftmost and rightmost:
                if line_count % 2 != 0:  # Odd rows: leftmost first, then rightmost
                    exploration_points.append(leftmost)
                    if rightmost != leftmost:
                        exploration_points.append(rightmost)
                else:  # Even rows: rightmost first, then leftmost
                    exploration_points.append(rightmost)
                    if rightmost != leftmost:
                        exploration_points.append(leftmost)

        return exploration_points


    def mark_exploration_points(self, occupancy_grid, exploration_points):
        data = occupancy_grid.data
        width = occupancy_grid.info.width

        for x, y in exploration_points:
            index = y * width + x
            data[index] = 15  # Mark exploration points with a lighter shade of gray


    def publish_exploration_grid(self):
        self.exploration_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.exploration_grid_publisher.publish(self.exploration_occupancy_grid)


    def publish_planning_grid(self):
        self.path_planning_grid.header.stamp = self.get_clock().now().to_msg()
        self.planning_grid_publisher.publish(self.path_planning_grid)


    def publish_detections_periodically(self):
        """
        Periodically publish detections to RViz to keep transforms visible.
        """
        publish_detections_to_rviz(self.tf_broadcaster, self.detected_objects, self.detected_boxes, self.get_clock())


    # The map is saved in the directory where the node is run
    def write_map_file(self, file_name=MAP_FILE_NAME):
        current_directory = os.getcwd()  # Get the current working directory

        with open(file_name, "w") as file:
            # Write the objects to the file
            for x, y, category in self.detected_objects:
                file.write(f"{category}\t{x * 100:.0f}\t{y * 100:.0f}\t0\n")  # Angle is 0 for objects

            # Write the boxes to the file
            for x, y, theta in self.detected_boxes:
                file.write(f"B\t{x * 100:.0f}\t{y * 100:.0f}\t{theta:.0f}\n")  # Use theta for the angle

        self.get_logger().info(f"Map file '{file_name}' has been written successfully to '{current_directory}'.")


    # ------ PREVIOUS VERSION ------

    #  # OLD version with all intermediate points
    # def compute_exploration_points(self, occupancy_grid, step):
    #     exploration_points = []
    #     width = occupancy_grid.info.width
    #     height = occupancy_grid.info.height
    #     data = occupancy_grid.data
    #     line_count = -1 # -1 to ignore the line y=0 (no free cells with workspace2)

    #     for y in range(0, height, step): # Zigzag pattern in the y direction
    #         line_points = []
    #         line_count += 1
    #         for x in range(0, width, step):
    #             index = y * width + x
    #             if data[index] == 0:  # Assuming 0 represents free space
    #                 line_points.append((x, y))
    #         if line_count % 2 == 0:
    #             line_points.reverse()  # This makes inverse every even line
    #         exploration_points.extend(line_points)

    #     return exploration_points

    # def is_new_detection(self, msg):
    #     # Select the appropriate list based on the detection type
    #     detected_list = {
    #         'OBJECT': self.detected_objects,
    #         'BOX': self.detected_boxes
    #     }[msg.type]

    #     # Check if the detection already exists
    #     for detection in detected_list:
    #         if ((detection[0] - msg.x) ** 2 + (detection[1] - msg.y) ** 2) ** 0.5 < POSITION_THRESHOLD:
    #             return False  # Detection already exists

    #     return None  # No match, so it is a new detection


    # ---------------- DEBUGGING FUNCTIONS ----------------
    def mark_grid_path(self, occupancy_grid, grid_path):
        data = occupancy_grid.data
        width = occupancy_grid.info.width
        for x, y in grid_path:
            index = y * width + x
            data[index] = 70  # Mark the path points for debugging

        self.publish_exploration_grid()  # Publish the updated grid


# ------------------------------- Main function -------------------------------

def main(args=None):
    rclpy.init(args=args)
    exploration_controller = ExplorationController()
    exploration_controller.get_logger().info("ExplorationController node has been created.")
    try:
        exploration_controller.run()
    except Exception as e:
        exploration_controller.get_logger().error(f"An error occurred: {e}")
    finally:
        exploration_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
