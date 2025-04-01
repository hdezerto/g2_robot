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


# ------------------------------- State class -------------------------------
class State(Enum):
    INIT = auto()
    OBSERVING = auto()
    GET_NEXT_OBJECT = auto()
    START_MOVING = auto()
    MOVING = auto() # Just proccessing callbacks
    END_COLLECTION = auto()


# ------------------------------- ExplorationController class -------------------------------
class CollectionController(Node):

    def run(self):
        while rclpy.ok():
            if self.state == State.OBSERVING:
               self.observing(3.0)  # Observe (spin) for 3 seconds
            elif self.state == State.GET_NEXT_OBJECT:
                self.get_next_object()
            # elif self.state == State.MOVE_TO_OBSERVATION:
            #     pass      
            elif self.state == State.MOVING:
                rclpy.spin_once(self) # DEBUGGING

            # elif self.state == State.MOVE_TO_PICK:
            #     pass
            elif self.state == State.END_COLLECTION:
                self.end_collection()
                break

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
        #self.mapper_occupancy_grid_subscriber = self.create_subscription(OccupancyGrid, '/mapper_occupancy_grid', self.mapper_occupancy_grid_callback, 10)
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

        self.get_logger().info('Waiting 3 sec...') 
        time.sleep(3) # Wait for all nodes to be ready and the TF buffer to populate

        self.state == State.MOVING # DEBUGGING




    # ------------------- STATE FUNCTIONS -------------------




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






