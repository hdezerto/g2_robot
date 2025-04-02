#!/usr/bin/env python

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

"""


#self.get_logger().info('HERE DEBUG!!!')  # DEBUG



# -------- Tunable parameters --------
MAP_FILE_NAME = "map_3.tsv"  # Name of the map file to read
# ------------------------------------
#EXPLORATION_STEP = 15 # DEBUGGING


# ------------------------------- State class -------------------------------
class State(Enum):
    TESTING = auto()
    INIT = auto()
    SCANNING = auto()
    GET_NEXT_OBJECT = auto()
    PLAN_PATH = auto()
    MOVING = auto()
    OBSERVE_OBJECT = auto()
    MOVE_TO_PICK = auto()
    PICK = auto()
    WAIT_FOR_ARM = auto()
    MOVE_TO_BOX = auto()
    DROP = auto()
    END_COLLECTION = auto()


# ------------------------------- ExplorationController class -------------------------------
class CollectionController(Node):

    def run(self):
        """
        Main loop for the state machine.
        """
        while rclpy.ok():
            try:
                self.handle_state()
            except StopIteration:
                #self.get_logger().info("Exiting the main loop.")
                break
            except Exception as e:
                self.get_logger().error(f"Error in state {self.state}: {e}")
                break
    
    def handle_state(self):
        """
        Handles the current state by calling the corresponding method.
        """
        state_methods = {
            State.TESTING: lambda: rclpy.spin_once(self),
            # lamda is needed when the function takes parameters (to avoid calling it immediately)
            # Ex.: self.function() is called immediately, while self.function is passed as a reference
            # State.SCANNING: lambda: self.observing(3.0), # Observe (spin) for 3 seconds
            # State.GET_NEXT_OBJECT: self.get_next_object,
            # State.PLAN_PATH: self.plan_path,
            # State.MOVING: lambda: rclpy.spin_once(self),
            # State.OBSERVE_OBJECT: lambda: self.observe_object(3.0),
            # State.MOVE_TO_PICK: self.move_to_pick,
            # State.PICK: self.pick,
            # State.WAIT_FOR_ARM: lambda: rclpy.spin_once(self),
            # State.MOVE_TO_BOX: self.move_to_box,
            # State.DROP: self.drop,
            # State.END_COLLECTION: self.end_collection,
        }

        if self.state in state_methods:
            #self.get_logger().info(f"Current state: {self.state.name}")
            state_methods[self.state]()  # Call the corresponding method

            # Stop the loop if the state is END_COLLECTION
            if self.state == State.END_COLLECTION:
                #self.get_logger().info("Collection process completed. Stopping the robot.")
                raise StopIteration  # Exit the loop in the `run` method
        else:
            #self.get_logger().error(f"Unknown state: {self.state}")
            raise ValueError(f"Unknown state: {self.state}")


    # ------------------- Initialization -------------------
    def __init__(self):
        # State to initialize the node
        super().__init__('CollectionController_node')
        self.state = State.INIT

        # Publishers and subscribers
        latched_qos = QoSProfile(depth=1) # Define a shared QoS profile for latched publishers
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', latched_qos)
        self.tf_broadcaster = TransformBroadcaster(self) # For publishing objects/boxes to RViz
        self.mapper_occupancy_grid_subscriber = self.create_subscription(OccupancyGrid, '/mapper_occupancy_grid', self.mapper_occupancy_grid_callback, 10)
        #self.detections_subscriber = self.create_subscription(DetectionMsg, '/detections', self.detections_callback, 5)
        self.planning_grid_publisher = self.create_publisher(OccupancyGrid, '/planning_grid', latched_qos)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        #self.reached_destination_subscriber = self.create_subscription(Bool, '/reached_destination', self.reached_destination_callback, 10)
        self.stop_publisher = self.create_publisher(Bool, '/stop_motion', 10)

        # Initialize TransformListener to get current position of the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) # spin_thread=True to run the listener in a separate thread

        # Publish the workspace to RViz
        publish_workspace(self.workspace_publisher, self.get_clock())

        # Initilizate variables for collection
        self.workspace_grid = initialize_occupancy_grid() # Clean grid with workspace boundaries
        self.current_position = (0, 0)  # Initial position (0, 0) in real world coordinates
        self.current_grid_position = real_to_grid_coordinates([self.current_position], self.workspace_grid)[0]

        self.exploration_point_index = 0
        self.exploration_point = None

        self.objects = []
        self.boxes = []
        self.read_map_file() # Read the map file and populate the objects and boxes lists
        self.grid_path = []  # Path in grid coordinates        
        self.path_planning_grid = update_path_planning_grid(self.workspace_grid, self.objects, self.boxes) # Grid where the path will be computed
        self.planning_grid_publisher.publish(self.path_planning_grid)

        # Timer to periodically publish objects/boxes to RViz
        self.detections_timer = self.create_timer(0.5, self.publish_transforms_periodically)

        self.next_object_position = None
        self.destination = None  # Destination point in real world coordinates




        # # ------ DEBUGGING EXPLORATION POINTS ------
        # # Initialize exploration grid (workspace file and resolution defined in occupancy_grid_map.py) to:
        # # - compute the exploration points
        # # - check if the detected objects/boxes are inside the workspace
        # self.exploration_occupancy_grid = initialize_occupancy_grid()
        # inflate_occupied_cells(self.exploration_occupancy_grid)
        # #self.exploration_points = self.compute_exploration_points(self.exploration_occupancy_grid, step=EXPLORATION_STEP)
        # self.exploration_points = [(10, 45), (185, 60), (185, 60), (185, 75), (135, 30), (105, 15), (20, 15), (20, 30), (105, 30)] # HARD CODED values
        # self.mark_exploration_points(self.exploration_occupancy_grid, self.exploration_points) # Just for DEBUG
        # self.exploration_grid_publisher = self.create_publisher(OccupancyGrid, '/exploration_occupancy_grid', latched_qos)
        # self.publish_exploration_grid()
        # # DEBUG:
        # self.get_logger().info(f'Exploration points (grid): {self.exploration_points}')
        # # real_world_points = grid_to_real_coordinates(self.exploration_points, self.exploration_occupancy_grid)
        # # formatted_real_world_points = [(f"{x:.2f}", f"{y:.2f}") for x, y in real_world_points]
        # # self.get_logger().info(f'Exploration points (real world): {formatted_real_world_points}')
        # # -----------------------------------------


        self.get_logger().info('Waiting 3 sec...') 
        time.sleep(3) # Wait for all nodes to be ready and the TF buffer to populate

        self.state = State.TESTING # DEBUGGING


    # ------------------- STATE FUNCTIONS -------------------
    def observing(self, duration):
        """
        Process callbacks for a specified duration, just for initial observation of the environment (using lidar)
        """
        self.get_logger().info(f'Observing for {duration} seconds...')
        start_time = self.get_clock().now().nanoseconds() / 1e9  # Start time in seconds
    
        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            rclpy.spin_once(self)
    
        self.get_logger().info('Finished observing.')
        self.state = State.GET_NEXT_OBJECT  # Now it can get the first object



    def get_next_object(self):

        # Select the closest object to the current position of the robot


        # Compute the observation point
        self.compute_observation_point()
             # Use an observation grid (copy of planning grid) where it is just uninflated around the objet to pick
             # Select the cells from distance = MIN_OBSERVATION_DISTANCE that are free
             # Filter the ones which are line free
             # If no cells found, return None
             # If it exists, store the observation point in self.destination (real coordinates, including orientation)

        if self.destination:
            self.State = State.PLAN_PATH # Plan the path to the observation point
        else:
            self.State = State.GET_NEXT_OBJECT # No observation point found, get the next object


    def plan_path(self):
        # TO DO


        self.State = State.MOVING # Avoid collision using Lidar
    




    def mapper_occupancy_grid_callback(self, msg):
        """
        Callback for processing the /mapper_occupancy_grid topic.
        """
        if self.state not in [State.OBSERVING, State]:
            return
        # FILL WITH CODE FROM EXPLORATION CONTROLLER
 
    


    # ------------------- UTILS ------------------- 

    def publish_transforms_periodically(self):
        """
        Periodically publish objects/boxes to RViz to keep transforms visible.
        """
        publish_detections_to_rviz(self.tf_broadcaster, self.objects, self.boxes, self.get_clock())


    def read_map_file(self, file_name=MAP_FILE_NAME):
        """
        Reads the map file and populates the objects and boxes lists.
        """
        if not os.path.exists(file_name):
            self.get_logger().error(f"Map file '{file_name}' does not exist.")
            return
        
        with open(file_name, 'r') as file:
            for line in file:
                parts = line.strip().split('\t')[:4]  # Only consider the first 4 parts
                category, x, y, angle = parts
                x, y = float(x) / 100, float(y) / 100  # Convert cm to m
                angle = float(angle) # Convert string to float
    
                if category == 'B':  # Box
                    self.boxes.append((x, y, angle))
                else:  # Object
                    self.objects.append((x, y, int(category)))
        
        self.get_logger().info(f"Map file '{file_name}' has been read successfully.")



        # # ---------- DEBUGGING (ignore) ----------


    # def compute_exploration_points(self, occupancy_grid, step):
    #     exploration_points = []
    #     width = occupancy_grid.info.width
    #     height = occupancy_grid.info.height
    #     data = occupancy_grid.data
    #     line_count = -1 # -1 to ignore the line y=0 (no free cells with workspace2)
    
    #     for y in range(0, height, step):  # Iterate over rows with the given step
    #         leftmost = None
    #         rightmost = None
    #         line_count += 1
    #         for x in range(0, width, step):
    #             index = y * width + x
    #             if data[index] == 0:  # Assuming 0 represents free space
    #                 if leftmost is None:
    #                     leftmost = (x, y)  # First free cell in the row
    #                 rightmost = (x, y)  # Update to the last free cell in the row
    
    #         # Alternate the order of adding points for zigzag pattern
    #         if leftmost and rightmost:
    #             if line_count % 2 != 0:  # Odd rows: leftmost first, then rightmost
    #                 exploration_points.append(leftmost)
    #                 if rightmost != leftmost:
    #                     exploration_points.append(rightmost)
    #             else:  # Even rows: rightmost first, then leftmost
    #                 exploration_points.append(rightmost)
    #                 if rightmost != leftmost:
    #                     exploration_points.append(leftmost)
    
    #     return exploration_points


    # def mark_exploration_points(self, occupancy_grid, exploration_points):
    #     data = occupancy_grid.data
    #     width = occupancy_grid.info.width

    #     for (x, y) in exploration_points:
    #         index = y * width + x
    #         data[index] = 15  # Mark exploration points with a lighter shade of gray


    # def publish_exploration_grid(self):
    #     self.exploration_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
    #     self.exploration_grid_publisher.publish(self.exploration_occupancy_grid)



# ------------------------------- Main function -------------------------------

def main(args=None):
    rclpy.init(args=args)
    exploration_controller = CollectionController()
    exploration_controller.get_logger().info('CollectionController node has been created.')

    try:
        exploration_controller.run()
    except Exception as e:
        exploration_controller.get_logger().error(f'An error occurred: {e}')
    finally:
        exploration_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()






