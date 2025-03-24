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

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

from detection_interfaces.msg import DetectionMsg

import time # DEBUG

"""
TO DO:
- Check the case when a box is also considered as a plushie
- Integrate motion controller
- Integrate lidar mapper

Check if the timer to populate the buffer is the best approach to avoid the transform error. Maybe async is better

"""


#self.get_logger().info('HERE DEBUG!!!')  # DEBUG


# -------- Tunable parameters --------
#EXPLORATION_STEP = 7  # Step size for generating exploration points [cells]
EXPLORATION_STEP = 15 # DEBUGGING
POSITION_THRESHOLD = 0.1  # Threshold for considering two detections as the same [m]
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
                self.spin_for_duration(3.0)  # Spin for 3 seconds
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

        self.static_tf_broadcaster  = StaticTransformBroadcaster(self) # For publishing detected objects/boxes to RViz

        # Subscribe to the /lidar_occupancy_grid topic
        self.lidar_occupancy_grid_subscriber = self.create_subscription(OccupancyGrid, '/lidar_occupancy_grid', self.lidar_occupancy_grid_callback, 10)

        # Subscribe to the /detections topic
        self.detections_subscriber = self.create_subscription(DetectionMsg, '/detections', self.detections_callback, 5)

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


        # Add a delay to allow the TransformListener to populate the buffer
        self.get_logger().info('Waiting for TF buffer to populate...')
        time.sleep(1)  # Wait for 1 second




        self.current_position = (0, 0)  # Initial position (0, 0) in real world coordinates
        self.current_grid_position = real_to_grid_coordinates([self.current_position], self.exploration_occupancy_grid)[0]

        self.exploration_point_index = 0
        self.exploration_point = None
        self.detected_objects = [] # List of tuples (x, y, category)
        self.detected_boxes = [] # List of tuples (x, y, theta)
        # Grid where the path will be computed. Obtained by adding the detected objects/boxes to the latest lidar grid.
        self.path_planning_grid = initialize_occupancy_grid()
        inflate_occupied_cells(self.path_planning_grid)
        self.grid_path = []  # Path in grid coordinates  

        # Subscribe to the /goal_reached topic
        self.reached_destination_subscriber = self.create_subscription(Bool, '/reached_destination', self.reached_destination_callback, 10)

        # Publisher for the path (for RViz and motion controller)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        # Publisher for the stop command (to motion controller)
        self.stop_publisher = self.create_publisher(Bool, '/stop_motion', 10)

        #self.state = ExplorationState.OBSERVING
        #self.state = ExplorationState.MOVING # DEBUG detection
        self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT  # DEBUG


    # ------------------- STATE FUNCTIONS -------------------
    def spin_for_duration(self, duration):
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
        
        # Compute or recompute the path to the exploration point (in case a collision is detected) and move to it
        # The grid_path is also saved to check for collisions while moving (much easier in grid coordinates)
        self.get_logger().info(f'Exploration point: {self.exploration_point}')  # DEBUG
        self.grid_path, path = compute_path(self.current_grid_position, self.exploration_point, self.path_planning_grid, self.get_clock())
        
        if path: # Path found
            self.publish_path(path) # Publish the path to the motion controller and RViz
            self.get_logger().info('Path published. Moving...')
            self.state = ExplorationState.MOVING
        else: # No path found
            self.get_logger().info('No path found. Getting the next exploration point.')
            self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT


    # TO FINISH
    def detections_callback(self, msg):
        self.get_logger().info(f'Received detection: {msg.type} (class: {msg.cat}) at ({msg.x}, {msg.y}) with theta {msg.theta}')  # DEBUG
        # Check if the detection is new and inside the workspace
        # NOTE: objects that lie on the edge of the workspace are considered outside!
        if self.is_new_detection(msg) and self.is_inside_workspace(msg.x, msg.y):
            # TO DO: ADD LOGIC TO CHECK FOR COLLISION
            if msg.type == 'OBJECT':
                self.detected_objects.append((msg.x, msg.y, msg.cat))
            else:  # msg.type == 'BOX'
                self.detected_boxes.append((msg.x, msg.y, msg.theta))
            publish_detections_to_rviz(self.static_tf_broadcaster, self.detected_objects, self.detected_boxes, self.get_clock())
            #self.state = ExplorationState.START_MOVING
        else:
            # Ignore previously detected objects/obstacles (state remains the same)
            pass

    
    # TO FINISH
    def lidar_occupancy_grid_callback(self, msg):
        self.get_logger().info('Received new lidar occupancy grid.') # DEBUG
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



    # ------------------- UTILS (specific to ExplorationController) ------------------- 

    # TEST (check if boxes dont need a larger threshold)
    def is_new_detection(self, msg):
        # Select the appropriate list based on the detection type
        detected_list = {
            'OBJECT': self.detected_objects,
            'BOX': self.detected_boxes
        }[msg.type]  # No need for .get() since the type is always valid
    
        # Check if the detection already exists
        for detection in detected_list:
            if ((detection[0] - msg.x) ** 2 + (detection[1] - msg.y) ** 2) ** 0.5 < POSITION_THRESHOLD:
                return False  # Detection already exists
    
        return True  # No match, so it is a new detection
    
    
    # NOT TESTED
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
            # Check if the cell value is 0 (free space) or 15 (exploration point)
            return self.exploration_occupancy_grid.data[index] in [0, 15]

        # If out of bounds, return False
        return False


    # NOT TESTED
    # The map is saved in the directory where the node is run
    def write_map_file(self):
        file_name = "map_file.txt"
    
        with open(file_name, 'w') as file:
            # Write the objects to the file
            for x, y, category in self.detected_objects:
                file.write(f"{category}\t{x:.2f}\t{y:.2f}\t0\n")  # Angle is 0 for objects
    
            # Write the boxes to the file
            for x, y, theta in self.detected_boxes:
                file.write(f"B\t{x:.2f}\t{y:.2f}\t{theta:.0f}\n")  # Use theta for the angle
    
        self.get_logger().info(f"Map file '{file_name}' has been written successfully.")


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

        for y in range(0, height, step): # Zigzag pattern in the y direction
            line_points = []
            line_count += 1
            for x in range(0, width, step):
                index = y * width + x
                if data[index] == 0:  # Assuming 0 represents free space
                    line_points.append((x, y))
            if line_count % 2 == 0:
                line_points.reverse()  # This makes inverse every even line
            exploration_points.extend(line_points)

        return exploration_points


    



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




