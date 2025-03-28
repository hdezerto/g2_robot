#!/usr/bin/env python

""" 
EXPLORATION LOGIC:



"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PolygonStamped
from .mission_control_utils import (publish_workspace, compute_path, publish_detections_to_rviz, get_current_position,
                                    check_collision)
from .occupancy_grid_map import (initialize_occupancy_grid, inflate_occupied_cells, update_path_planning_grid,
                                grid_to_real_coordinates, real_to_grid_coordinates)
from nav_msgs.msg import OccupancyGrid

from enum import Enum, auto
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from detection_interfaces.msg import DetectionMsg
import os # To get the current directory

import time # DEBUG

"""
TO DO:
- Check the case when a box is also considered as a plushie
- Fix locked states for the motion controller when the robot moves on the floor
- Integrate lidar mapper
 -HUGE DRIFT


Check if the timer to populate the buffer is the best approach to avoid the transform error. Maybe async is better

"""


#self.get_logger().info('HERE DEBUG!!!')  # DEBUG



# -------- Tunable parameters --------
#EXPLORATION_STEP = 7  # Step size for generating exploration points [cells]
EXPLORATION_STEP = 15 # DEBUGGING
POSITION_THRESHOLD = 0.13  # Threshold for considering two detections as the same [m]
# ------------------------------------


# ------------------------------- ExplorationState class -------------------------------
class ExplorationState(Enum):
    INIT = auto()
    OBSERVING = auto()
    GET_NEXT_EXPLORATION_POINT = auto()
    START_MOVING = auto()
    MOVING = auto() # Just proccessing callbacks
    END_EXPLORATION = auto()


# ------------------------------- ExplorationController class -------------------------------
class ExplorationController(Node):

    def run(self):
        while rclpy.ok():
            if self.state == ExplorationState.OBSERVING:
                self.observing(3.0)  # Observe (spin) for 3 seconds
            elif self.state == ExplorationState.GET_NEXT_EXPLORATION_POINT:
                self.get_next_exploration_point()
            elif self.state == ExplorationState.START_MOVING:
                self.start_moving()
            elif self.state == ExplorationState.MOVING: # Just process callbacks
                rclpy.spin_once(self)
                #rclpy.spin_once(self, timeout_sec=1) # DEBUG
                #self.get_logger().info('Inside MOVING')  # DEBUG
            elif self.state == ExplorationState.END_EXPLORATION:
                self.end_exploration()
                break

    # ------------------- Initialization -------------------
    def __init__(self):
        # State to initialize the node
        super().__init__('ExplorationController_node')
        self.state = ExplorationState.INIT

        # Define a shared QoS profile for latched publishers
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # Publisher for the workspace (latched publisher)
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', latched_qos)
        # Publisher for the exploration grid (latched publisher)
        self.exploration_grid_publisher = self.create_publisher(OccupancyGrid, '/exploration_occupancy_grid', latched_qos)
        # Publish the workspace to RViz
        publish_workspace(self.workspace_publisher, self.get_clock())

        self.tf_broadcaster = TransformBroadcaster(self) # For publishing detected objects/boxes to RViz

        # Subscribe to the /mapper_occupancy_grid topic
        #self.mapper_occupancy_grid_subscriber = self.create_subscription(OccupancyGrid, '/mapper_occupancy_grid', self.mapper_occupancy_grid_callback, 10)

        # Subscribe to the /detections topic
        self.detections_subscriber = self.create_subscription(DetectionMsg, '/detections', self.detections_callback, 5) # CHECK if 5 is not too much here

        # Initialize a clean occupancy grid (workspace file and resolution defined in occupancy_grid_map.py)
        self.exploration_occupancy_grid = initialize_occupancy_grid()

        # Inflate the occupied cells to avoid collisions
        inflate_occupied_cells(self.exploration_occupancy_grid)

        # Compute exploration points using the clean grid
        self.exploration_points = self.compute_exploration_points(self.exploration_occupancy_grid, step=EXPLORATION_STEP)

        # Mark exploration points in the occupancy grid
        self.mark_exploration_points(self.exploration_occupancy_grid, self.exploration_points) # DEBUG

        # Publish the exploration occupancy grid
        self.publish_exploration_grid()
        self.get_logger().info('Exploration grid published.')  # DEBUG
        
        # DEBUG:
        real_world_points = grid_to_real_coordinates(self.exploration_points, self.exploration_occupancy_grid)
        self.get_logger().info(f'Exploration points (grid): {self.exploration_points}')
        formatted_real_world_points = [(f"{x:.2f}", f"{y:.2f}") for x, y in real_world_points]
        self.get_logger().info(f'Exploration points (real world): {formatted_real_world_points}')

        # Initialize TF2 Buffer and TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) # spin_thread=True to run the listener in a separate thread

        self.current_position = (0, 0)  # Initial position (0, 0) in real world coordinates
        self.current_grid_position = real_to_grid_coordinates([self.current_position], self.exploration_occupancy_grid)[0]

        self.exploration_point_index = 0
        self.exploration_point = None
        self.detections = []  # Unified list for all detections
        self.detected_objects = [] # List of tuples (x, y, category)
        self.detected_boxes = [] # List of tuples (x, y, theta)
        # Grid where the path will be computed. Obtained by adding the detected objects/boxes to the latest lidar grid.
        self.path_planning_grid = initialize_occupancy_grid()
        inflate_occupied_cells(self.path_planning_grid)
        self.grid_path = []  # Path in grid coordinates

        self.planning_grid_publisher = self.create_publisher(OccupancyGrid, '/planning_grid', latched_qos)

        # Subscribe to the /goal_reached topic
        self.reached_destination_subscriber = self.create_subscription(Bool, '/reached_destination', self.reached_destination_callback, 10)

        # Publisher for the path (for RViz and motion controller)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        # Publisher for the stop command (to motion controller)
        self.stop_publisher = self.create_publisher(Bool, '/stop_motion', 10)

        # Timer to periodically publish detections to RViz
        self.detections_timer = self.create_timer(0.5, self.publish_detections_periodically)

        # Add a delay to allow the TransformListener to populate the buffer and 
        self.get_logger().info('Waiting 3 sec for TF buffer to populate...')
        time.sleep(3)  # Wait for 2 second


        #self.state = ExplorationState.OBSERVING
        self.state = ExplorationState.MOVING # DEBUG detection
        #self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT  # DEBUG motion controller


    # ------------------- STATE FUNCTIONS -------------------
    def observing(self, duration):
        """
        Process callbacks for a specified duration, just for initial observation of the environment (using camera and lidar)
        """
        self.get_logger().info(f'Observing for {duration} seconds...')
        start_time = self.get_clock().now().nanoseconds() / 1e9  # Start time in seconds
    
        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            rclpy.spin_once(self)
    
        self.get_logger().info('Finished observing.')
        self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT  # Now it can get the first exploration point


    def get_next_exploration_point(self):
        # Get the next exploration point from the list (if it exists)
        if self.exploration_point_index < len(self.exploration_points):
            self.exploration_point = self.exploration_points[self.exploration_point_index] # Get grid coordinates of the next exploration point
            self.exploration_point_index += 1
            self.state = ExplorationState.START_MOVING
        else: # No more points to explore
            self.get_logger().info('No more exploration points. Ending exploration.')
            self.state = ExplorationState.END_EXPLORATION
    

    def start_moving(self):
        # Update the current position of the robot
        
        self.current_position, self.current_grid_position = get_current_position(self.tf_buffer, self.get_logger(), self.exploration_occupancy_grid)
        if self.current_position is None:
            self.get_logger().info('Failed to get current position!') # DEBUG
        self.get_logger().info(f'Current position (real): {self.current_position}  | (grid): {self.current_grid_position}')  # DEBUG
        
                # Compute or recompute the path to the exploration point (in case a collision is detected) and move to it
        # The grid_path is also saved to check for collisions while moving (much easier in grid coordinates)
        start = (self.current_grid_position, self.current_position)
        goal = (self.exploration_point, grid_to_real_coordinates([self.exploration_point], self.path_planning_grid)[0])
        self.get_logger().info(f'Start: {start} | Goal: {goal}')  # DEBUG
        self.grid_path, path = compute_path(start, goal, self.path_planning_grid, self.get_clock())

        # --------- JUST TO SEE THE GRID PATH ---------
        self.mark_grid_path(self.exploration_occupancy_grid, self.grid_path)  # DEBUG
        # --------------------------------------

        if path: # Path found
            self.publish_path(path) # Publish the path to the motion controller and RViz
            self.get_logger().info('Path published. Moving...')
            self.state = ExplorationState.MOVING
        else: # No path found
            self.get_logger().info('No path found. Getting the next exploration point.')
            self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT


    # TO FINISH
    def detections_callback(self, msg):
        #self.get_logger().info(f'Received detection: {msg.type} (class: {msg.cat}) at ({msg.x}, {msg.y}) with theta {msg.theta}')  # DEBUG
        # Check if the detection is inside the workspace. NOTE: objects that lie on the edge of the workspace are considered outside!
        if not self.is_inside_workspace(msg.x, msg.y):
            self.get_logger().info(f'Detection is outside the workspace. Ignoring it.')
            return
        
        # Update the detections list. It returns if it's a new detection (for collision check)
        is_new_detection = self.update_detections(msg)

        # # Check for collision with the current path
        # if is_new_detection:
        #     # Update the planning grid with the latest lidar occupancy grid and the detected objects/boxes
        #     update_path_planning_grid(self.path_planning_grid, self.detected_objects + self.detected_boxes) # DEBUG
        #     self.planning_grid_publisher.publish(self.path_planning_grid)  # DEBUG
        #     # Update the current position of the robot (from odometry or localization)
        #     self.current_position, self.current_grid_position = get_current_position(self.tf_buffer, self.get_logger(), self.exploration_occupancy_grid)
        #     if self.current_position is None:
        #         self.get_logger().info('Failed to get current position!') # DEBUG
        #     if check_collision(self.path_planning_grid, self.grid_path, self.current_grid_position):
        #         self.stop_robot()
        #         self.get_logger().info('Collision detected. Recomputing path.')
        #         self.state = ExplorationState.START_MOVING



    # ----------------- NEW FUNCTIONS WITH VOTING TO TEST -----------------

    def update_detections(self, msg):
        # Check if the detection is new or corresponds to an existing one
        detected_list = self.detections  # Unified list for all detections
        for detection in detected_list:
            if ((detection['x'] - msg.x) ** 2 + (detection['y'] - msg.y) ** 2) ** 0.5 < POSITION_THRESHOLD:
                # Existing detection: update position and add vote
                detection['x'] = msg.x
                detection['y'] = msg.y
                if msg.type == 'BOX':
                    detection['theta'] = msg.theta
                    detection['votes'].append(msg.type) # Box detections have no category
                else:
                    detection['votes'].append(msg.cat) # For the objects, category is used              
                detection['winner'] = max(set(detection['votes']), key=detection['votes'].count)  # Update winner
                #self.get_logger().info(f'Updated detection: {detection}')
                return False # Existing detection updated

        # New detection: add to the list
        new_detection = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta if msg.type == 'BOX' else None,
            'votes': [msg.cat] if msg.type == 'OBJECT' else [msg.type],  # Initialize votes with the category or type
            'winner': msg.cat if msg.type == 'OBJECT' else msg.type  # Initialize winner with the category or type
        }
        detected_list.append(new_detection) # detected_list is a list of dictionaries
        #self.get_logger().info(f'Added new detection: {new_detection}')

        self.update_detections_lists()  # Update the detected objects and boxes lists

        return True # New detection added


    def update_detections_lists(self):
        """
        Updates the individual lists of detected objects and boxes based on the unified detections list.
        """
        self.detected_objects = []  # Clear the existing list of objects
        self.detected_boxes = []    # Clear the existing list of boxes
    
        for detection in self.detections:
            if detection['winner'] == 'BOX':  # The detection is classified as a box (x, y, theta)
                self.detected_boxes.append((detection['x'], detection['y'], detection['theta']))
            else:  # The detection is classified as an object (x, y, category)
                self.detected_objects.append((detection['x'], detection['y'], detection['winner']))

    # ----------------------------------------


    # TO FINISH
    def mapper_occupancy_grid_callback(self, msg):
        self.get_logger().info('Received new mapper occupancy grid.') # DEBUG
        # Update the planning grid with the latest lidar occupancy grid and the detected objects/boxes
        update_path_planning_grid(self.path_planning_grid, msg, self.detected_objects + self.detected_boxes)
        # Update the current position of the robot (from odometry or localization)
        self.current_position, self.current_grid_position = get_current_position(self.tf_buffer, self.get_logger(), self.exploration_occupancy_grid) 
        if check_collision(self.path_planning_grid, self.grid_path, self.current_grid_position):
            self.stop_robot()
            self.get_logger().info('Collision detected. Recomputing path.')
            self.state = ExplorationState.START_MOVING
        else:
            pass  # No collision
        

    def reached_destination_callback(self, msg):
        if msg.data: # msg.data is True if the destination was reached
            self.get_logger().info('Destination reached successfully. Going to the next exploration point.')
            self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT
        else:
            self.get_logger().info('Failed to reach destination. Going to the next exploration point.')
            self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT


    def end_exploration(self):
        # Write the map file with detected objects/boxes
        self.write_map_file()
        self.get_logger().info('Exploration completed. Map file saved.')



    # ---------------- DEBUGGING FUNCTIONS ----------------    
    def mark_grid_path(self, occupancy_grid, grid_path):
        data = occupancy_grid.data
        width = occupancy_grid.info.width        
        for (x, y) in grid_path:
            index = y * width + x
            data[index] = 70  # Mark the path points for debugging        
        
        self.publish_exploration_grid()  # Publish the updated grid



    # ------------------- UTILS (specific to ExplorationController) ------------------- 

    def is_new_detection(self, msg):
        # Select the appropriate list based on the detection type
        detected_list = {
            'OBJECT': self.detected_objects,
            'BOX': self.detected_boxes
        }[msg.type]

        # Check if the detection already exists
        for detection in detected_list:
            if ((detection[0] - msg.x) ** 2 + (detection[1] - msg.y) ** 2) ** 0.5 < POSITION_THRESHOLD:
                return False  # Detection already exists
    
        return None  # No match, so it is a new detection
    
    
    def is_inside_workspace(self, x, y):
        """
        Check if the given coordinates (x, y) are inside the workspace.
        The workspace is defined as cells in the exploration grid with values 0 (free space) or 15 (exploration points).
        """
        # Convert real-world coordinates to grid coordinates
        grid_x, grid_y = real_to_grid_coordinates([(x, y)], self.exploration_occupancy_grid)[0]

        # Get the grid dimensions
        width = self.exploration_occupancy_grid.info.width
        height = self.exploration_occupancy_grid.info.height

        # Check if the grid coordinates are within bounds
        if 0 <= grid_x < width and 0 <= grid_y < height:
            # Calculate the index in the grid data
            index = grid_y * width + grid_x
            # Check if the cell value is 0 (free space), 15 (exploration point) or 50 (inflated cells of the workspace border)
            return self.exploration_occupancy_grid.data[index] in [0, 15, 50]

        # If out of bounds, return False
        return False


    def publish_detections_periodically(self):
        """
        Periodically publish detections to RViz to keep transforms visible.
        """
        publish_detections_to_rviz(self.tf_broadcaster, self.detected_objects, self.detected_boxes, self.get_clock())


    # The map is saved in the directory where the node is run
    def write_map_file(self):
        file_name = "map_file.txt"
        current_directory = os.getcwd()  # Get the current working directory
    
        with open(file_name, 'w') as file:
            # Write the objects to the file
            for x, y, category in self.detected_objects:
                file.write(f"{category}\t{x:.2f}\t{y:.2f}\t0\n")  # Angle is 0 for objects
    
            # Write the boxes to the file
            for x, y, theta in self.detected_boxes:
                file.write(f"B\t{x:.2f}\t{y:.2f}\t{theta:.0f}\n")  # Use theta for the angle
    
        self.get_logger().info(f"Map file '{file_name}' has been written successfully to '{current_directory}'.")


    def stop_robot(self):
        msg = Bool()
        msg.data = True
        self.stop_publisher.publish(msg)  


    def mark_exploration_points(self, occupancy_grid, exploration_points):
        data = occupancy_grid.data
        width = occupancy_grid.info.width

        for (x, y) in exploration_points:
            index = y * width + x
            data[index] = 15  # Mark exploration points with a lighter shade of gray


    def publish_exploration_grid(self):
        self.exploration_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.exploration_grid_publisher.publish(self.exploration_occupancy_grid)


    def publish_path(self, path):
        # Publish the path to the motion controller and RViz
        self.path_publisher.publish(path)
   

    def compute_exploration_points(self, occupancy_grid, step):
        exploration_points = []
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = occupancy_grid.data
        line_count = -1 # -1 to ignore the line y=0 (no free cells with workspace2)
    
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

    



# ------------------------------- Main function -------------------------------

def main(args=None):
    rclpy.init(args=args)
    exploration_controller = ExplorationController()
    exploration_controller.get_logger().info('ExplorationController node has been created.')

    try:
        exploration_controller.run()
    except Exception as e:
        exploration_controller.get_logger().error(f'An error occurred: {e}')
    finally:
        exploration_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




