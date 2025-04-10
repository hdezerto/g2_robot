#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PolygonStamped
from .mission_control_utils import (publish_workspace, compute_path, publish_detections_to_rviz, get_current_pose,
                                    check_collision)
from .occupancy_grid_map import (initialize_occupancy_grid, inflate_occupied_cells, update_path_planning_grid,
                                grid_to_real_coordinates, real_to_grid_coordinates)
from nav_msgs.msg import OccupancyGrid

from enum import Enum, auto
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from detection_interfaces.msg import DetectionMsg
import os # To get the current directory

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler
import numpy as np

import time # DEBUG

"""
NOTES (HUGO):
- Check how the motion controller executes the yaw from the PoseStamped() of the Path()

- I assume the observation point and drop point (near a box) are not inflated on the planning map,
otherwise A* fails. I also assume the closest uninflated point to a box is good enough for dropping.


"""


#self.get_logger().info('HERE DEBUG!!!')  # DEBUG



# -------- Tunable parameters --------
MAP_FILE_NAME = "map_3.tsv"  # Name of the map file to read
SCANNING_TIME = 3.0  # Time to scan the environment [s]
DETECTION_TIMEOUT = 5.0  # Timeout for waiting for object detection [s]
# NOTE: OBSERVATION_DISTANCE >= PICK_DISTANCE 
OBSERVATION_DISTANCE = 0.30 # Distance to the object for observation [m]
PICK_DISTANCE = 0.17 # Distance to the object for pick [m]
#PLACE_DISTANCE = ??  # Distance to the box for drop [m]
# ------------------------------------


#EXPLORATION_STEP = 15 # DEBUGGING


# ------------------------------- State class -------------------------------
class State(Enum):
    TESTING = auto() # To delete later

    INIT = auto()
    SCANNING = auto()
    GET_NEXT_OBJECT = auto()
    PLAN_PATH = auto()
    MOVING = auto()
    OBSERVE_OBJECT = auto()
    MOVE_TO_PICK = auto()
    MOVING_BLINDLY = auto()
    PICK = auto()
    WAIT_FOR_ARM = auto()
    MOVE_TO_BOX = auto()
    DROP = auto()
    END_COLLECTION = auto()


# ------------------------------- CollectionController class -------------------------------
class CollectionController(Node):

    def run(self):
        """
        Main loop for the state machine.
        """
        while rclpy.ok():
            try:
                self.handle_state()
            except StopIteration:
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
            # Ex.: self.function(args) is called immediately, while self.function is passed as a reference
            State.SCANNING: lambda: self.scanning(SCANNING_TIME),
            State.GET_NEXT_OBJECT: self.get_next_object,
            State.PLAN_PATH: self.plan_path,
            State.MOVING: lambda: rclpy.spin_once(self),
            State.OBSERVE_OBJECT: lambda: self.observe_object(DETECTION_TIMEOUT),
            State.MOVE_TO_PICK: self.move_to_pick,
            State.MOVING_BLINDLY: lambda: rclpy.spin_once(self),
            State.PICK: self.pick,
            State.WAIT_FOR_ARM: lambda: rclpy.spin_once(self),
            State.MOVE_TO_BOX: self.move_to_box,
            State.DROP: self.drop,
        }

        if self.state in state_methods:
            #self.get_logger().info(f"Current state: {self.state.name}")
            state_methods[self.state]()  # Call the corresponding method

            # Stop the loop if the state is END_COLLECTION
            if self.state == State.END_COLLECTION:
                self.get_logger().info("Collection process completed.")
                raise StopIteration
        else:
            raise ValueError(f"Unknown state")


    # ------------------- Initialization -------------------
    def __init__(self):
        # State to initialize the node
        super().__init__('CollectionController_node')
        self.state = State.INIT

        # Interface with RViz:
        latched_qos = QoSProfile(depth=1) # Define a shared QoS profile for latched publishers
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', latched_qos)
        self.workspace_grid_publisher = self.create_publisher(OccupancyGrid, '/workspace_occupancy_grid', latched_qos)
        self.tf_broadcaster = TransformBroadcaster(self) # For publishing objects/boxes
        self.planning_grid_publisher = self.create_publisher(OccupancyGrid, '/planning_grid', latched_qos)
        # Interface with mapper:
        self.mapper_occupancy_grid_subscriber = self.create_subscription(OccupancyGrid, '/mapper_occupancy_grid', self.mapper_occupancy_grid_callback, 1)
        # Interface with motion controller:
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.reached_destination_subscriber = self.create_subscription(Bool, '/reached_destination', self.reached_destination_callback, 10)
        self.stop_publisher = self.create_publisher(Bool, '/stop_motion', 10)
        # Interface with arm:
        self.arm_command_publisher = self.create_publisher(Int32, '/arm_controller', 10)
        self.arm_feedback_subscriber = self.create_subscription(Bool, '/arm_controller_feedback', self.arm_feedback_callback, 10)
        # TODO: change arm interface if needed

        # Initialize TransformListener to get current position of the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True) # spin_thread=True to run the listener in a separate thread

        # Publish the workspace to RViz
        publish_workspace(self.workspace_publisher, self.get_clock())

        # Timer to periodically publish objects/boxes to RViz
        self.detections_timer = self.create_timer(0.5, self.publish_transforms_periodically)

        # Clean grid with workspace boundaries. Used to initialize planning grid and to mark the grid path.
        # Only published inside mark_grid_path() 
        self.workspace_grid = initialize_occupancy_grid()

        # -- Variables for collection --
        self.current_pose = (0, 0, 0)  # Initial position (x, y, yaw) in real world coordinates
        self.current_grid_position = real_to_grid_coordinates([self.current_pose], self.workspace_grid)[0]
        self.objects = []
        self.boxes = []
        self.read_map_file() # Read the map file and populate the objects and boxes lists
        self.grid_path = []  # Path in grid coordinates        
        self.path_planning_grid = update_path_planning_grid(self.workspace_grid, self.objects, self.boxes) # Grid where the path will be computed
        self.publish_planning_grid() # Publish the initial grid to RViz
        self.next_object = {"position": None, "category": None, "index": None}
        self.closest_box = None # (x, y, theta) in real world coordinates
        self.destination_pose = None # (x, y, theta) in real world coordinates. Only for observation and drop poses
        self.detected_position = None # (x, y) in real world coordinates. To store the observed position of the object

        # # ------ DEBUGGING EXPLORATION POINTS (ignore it) ------
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

        self.task = None # State.PICK or State.DROP

        #self.state = State.TESTING # DEBUGGING
        #self.state = State.SCANNING
        self.state = State.GET_NEXT_OBJECT
        


    # ------------------- STATE FUNCTIONS -------------------
    # MIGHT BE REMOVED!
    def scanning(self, duration):
        """
        Process callbacks for a specified duration, just for initial observation of the environment (using lidar)
        """
        self.get_logger().info(f'Observing for {duration} seconds...')
        start_time = self.get_clock().now().nanoseconds() / 1e9  # Start time in seconds
    
        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            rclpy.spin_once(self)
    
        self.get_logger().info('Finished scanning. SCANNING -> GET_NEXT_OBJECT') # DEBUG
        self.state = State.GET_NEXT_OBJECT  # Now it can get the first object


    def get_next_object(self):
        if not self.objects: # Empty list of objects
            self.get_logger().info("No more objects to collect. GET_NEXT_OBJECT -> END_COLLECTION")
            self.state = State.END_COLLECTION
            return
        
        self.task = State.PICK
        self.compute_closest_object() # Select the closest object to the current position of the robot

        # --------------------- TODO -------------------
        # Compute the observation point
        observation_pose = self.compute_observation_pose(OBSERVATION_DISTANCE)
            # Returns the observation point with (x_real, y_real, final orientation)        
            # Mattias wants a distance of 17 cm relative to the base_link 
        # ---------------------------------------------

        if observation_pose:
            self.destination_pose = observation_pose # (x, y, theta) in real world coordinates, where theta is the final orientation to observe
            self.get_logger().info(f"Observation point: {self.destination_pose}. GET_NEXT_OBJECT -> PLAN_PATH")
            self.state = State.PLAN_PATH # Plan the path to the observation point
        else:
            self.get_logger().info("No observation point found. GET_NEXT_OBJECT -> END_COLLECTION") # DEBUG
            self.state = State.END_COLLECTION # DEBUG since it should in principle find an observation point


    def plan_path(self):
        self.update_current_pose()  # Update the current pose of the robot
        start = (self.current_grid_position, self.current_pose)
        goal = (real_to_grid_coordinates([self.destination_pose], self.path_planning_grid)[0], self.destination_pose)
        #self.get_logger().info(f'Start: {start} | Goal: {goal}')  # DEBUG
        self.grid_path, path = compute_path(start, goal, self.path_planning_grid, self.get_clock())
        if path: # Path found
            self.path_publisher.publish(path) # Publish the path to the motion controller and RViz
            self.get_logger().info('Path published. PLAN_PATH -> MOVING')
            # --------- JUST TO SEE THE GRID PATH ---------
            self.mark_grid_path(self.workspace_grid, self.grid_path)  # DEBUG
            # --------------------------------------
            self.state = State.MOVING
        else:
            self.get_logger().info('No path found. PLAN_PATH -> END_COLLECTION') # DEBUG
            self.state = State.END_COLLECTION # DEBUG since it should in principle find an a path


    def mapper_occupancy_grid_callback(self, msg):
        if self.state != State.MOVING: # Ignore lidar mapping when not moving
            return
        # TODO : Use the same code from exploration
 
    
    def observe_object(self, timeout):
        self.get_logger().info(f"Observing for object category {self.next_object['category']} for {timeout} seconds...")
    
        start_time = self.get_clock().now().nanoseconds / 1e9  # Start time in seconds
    
        # Callback to process detections
        def detection_callback(msg):
            if msg.cat == self.next_object["category"]:  # Check if the detection matches the desired category
                self.get_logger().info(f"Detected desired object: category {msg.cat} at ({msg.x}, {msg.y}). OBSERVE_OBJECT -> MOVE_TO_PICK")
                self.detected_position = (msg.x, msg.y)
                self.state = State.MOVE_TO_PICK
                self.detections_subscriber.destroy()  # Stop listening to detections
    
        # Subscribe to the /detections topic
        self.detections_subscriber = self.create_subscription(DetectionMsg, '/detections', detection_callback, 10)
    
        # Wait for the detection or timeout
        while self.state == State.OBSERVE_OBJECT:
            rclpy.spin_once(self) # Process callbacks
            elapsed_time = self.get_clock().now().nanoseconds / 1e9 - start_time
            if elapsed_time > timeout:
                self.get_logger().info("Timeout reached while waiting for object detection. OBSERVE_OBJECT -> END_COLLECTION")
                self.state = State.END_COLLECTION  # Transition to END_COLLECTION if no detection is received
                self.detections_subscriber.destroy()  # Stop listening to detections
                break


    def move_to_pick(self):
        """
        Creates a straight path from the current position to a position at a certain distance
        from the detected object and publishes the path. Transitions to MOVING_BLINDLY.
        """
        self.update_current_pose()  # Update the robot's current pose

        pick_path, pick_pose = self.create_pick_path(PICK_DISTANCE)
      
        # Publish the path
        self.path_publisher.publish(pick_path)
        self.get_logger().info(f"Path to pick position published: Start {self.current_pose} | Goal{pick_pose}. MOVE_TO_PICK -> MOVING_BLINDLY")
    
        self.state = State.MOVING_BLINDLY
    

    def move_to_box(self):
        self.task = State.DROP

        # Remove the picked object from the objects list
        removed_object = self.objects.pop(self.next_object["index"])
        self.get_logger().info(f'Removed object from list: {removed_object}')
    
        # Update the planning grid to mark the object's position as free space
        self.path_planning_grid = update_path_planning_grid(self.workspace_grid, self.objects, self.boxes)
        self.publish_planning_grid()  # Publish the updated grid to RViz

        self.compute_closest_box()  # Select the closest box to the current position of the robot

        # ---------------------- TODO -------------------
        # Compute the drop pose
        drop_pose = self.compute_drop_pose()
            # Computes the closest pose to the robot around the box that is not inflated nor occupied
            # Returns the drop pose with (x_real, y_real, final orientation)
        # ---------------------------------------------
            
        if drop_pose:
            self.destination_pose = drop_pose  # Set the drop pose as the destination
            self.get_logger().info(f'Drop pose: {self.destination_pose}. MOVE_TO_BOX -> PLAN_PATH')
            self.state = State.PLAN_PATH
        else:
            self.get_logger().error('Failed to compute drop pose. MOVE_TO_BOX -> END_COLLECTION')
            self.state = State.END_COLLECTION # DEBUG since it should in principle find a drop pose


    def pick(self):
        msg = Int32()
        msg.data = 1 # 1 for PICK
        self.arm_command_publisher.publish(msg)
        self.get_logger().info('Sent arm command to PICK object. PICK -> WAIT_FOR_ARM')
        self.state = State.WAIT_FOR_ARM
    

    def drop(self):
        msg = Int32()
        msg.data = 2 # 2 for DROP
        self.arm_command_publisher.publish(msg)
        self.get_logger().info('Sent arm command to DROP object. DROP -> WAIT_FOR_ARM')
        self.state = State.WAIT_FOR_ARM


    def arm_feedback_callback(self, msg):
        if msg.data:  # True indicates success
            if self.task == State.PICK:
                self.get_logger().info('Pick operation successful. WAIT_FOR_ARM -> MOVE_TO_BOX')
                self.state = State.MOVE_TO_BOX
            elif self.task == State.DROP:
                self.get_logger().info('Drop operation successful. WAIT_FOR_ARM -> GET_NEXT_OBJECT')
                self.state = State.GET_NEXT_OBJECT
        else:  # False indicates failure
            self.get_logger().error('Arm operation failed. State -> END_COLLECTION')
            self.state = State.END_COLLECTION # DEBUG. Improve how to handle failure


    def reached_destination_callback(self, msg):
        if msg.data: # msg.data is True if the destination was reached
            if self.state == State.MOVING and self.task == State.PICK:
                self.get_logger().info('Observation position reached. MOVING -> OBSERVE_OBJECT')
                self.state = State.OBSERVE_OBJECT
            elif self.state == State.MOVING and self.task == State.DROP:
                self.get_logger().info('Drop position reached. MOVING -> DROP')
                self.state = State.DROP
            elif self.state == State.MOVING_BLINDLY:
                self.get_logger().info('Pick position reached. MOVING_BLINDLY -> PICK')
                self.state = State.PICK
        else:
            self.get_logger().info(f'Failed to reach destination. State: {self.state.name} -> END_COLLECTION')
            self.state = State.END_COLLECTION


    # ------------------- UTILS ------------------- 
    def create_pick_path(self, pick_distance):
        # Extract current position and detected position
        current_x, current_y, current_theta = self.current_pose
        detected_x, detected_y = self.detected_position
    
        # Calculate the direction vector and target position
        dx = detected_x - current_x
        dy = detected_y - current_y
        distance = (dx**2 + dy**2)**0.5
    
        scale = (distance - pick_distance) / distance
        target_x = current_x + dx * scale
        target_y = current_y + dy * scale
        target_theta = np.arctan2(dy, dx) # The last yaw is oriented to the object
    
        # Create the Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
    
        # Add start and target poses to the path
        path_msg.poses.append(self.create_pose_stamped(current_x, current_y, current_theta))
        path_msg.poses.append(self.create_pose_stamped(target_x, target_y, target_theta))
    
        return path_msg, (target_x, target_y, target_theta)
    

    def create_pose_stamped(self, x, y, theta):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
    
        # Convert yaw (theta) to quaternion
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x, pose.pose.orientation.y, \
        pose.pose.orientation.z, pose.pose.orientation.w = q
    
        return pose


    def compute_closest_object(self):
        self.update_current_pose()  # Update the current pose of the robot
        closest_distance = float('inf')
        closest_idx = None
    
        # Iterate through the objects to find the closest one
        for idx, (x, y, _) in enumerate(self.objects):
            distance = ((x - self.current_pose[0]) ** 2 + (y - self.current_pose[1]) ** 2) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_idx = idx
    
        self.next_object["position"] = (self.objects[closest_idx][0], self.objects[closest_idx][1])
        self.next_object["index"] = closest_idx
        self.next_object["category"] = self.objects[closest_idx][2]
        self.get_logger().info(f"Closest object selected: Position {self.next_object['position']} | Category {self.next_object['category']}.")


    def compute_closest_box(self):       
        self.update_current_pose()  # Update the robot's current pose
        closest_distance = float('inf')
        closest_box = None

        # Iterate through the boxes to find the closest one
        for box in self.boxes:
            box_x, box_y, _ = box
            distance = ((box_x - self.current_pose[0]) ** 2 + (box_y - self.current_pose[1]) ** 2) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_box = box
        
        self.closest_box = closest_box # (x, y, theta) in real world coordinates
        self.get_logger().info(f"Closest box selected: Pose {self.closest_box}.")


    def update_current_pose(self):
        # Update the current pose of the robot
        self.current_pose, self.current_grid_position = get_current_pose(self.tf_buffer, self.get_logger(), self.workspace_grid)
        if self.current_pose is None:
            self.get_logger().info('Failed to get current pose!')
        #self.get_logger().info(f'Current pose (real): {self.current_pose}  | (grid): {self.current_grid_position}')  # DEBUG
        

    def publish_transforms_periodically(self):
        """
        Periodically publish objects/boxes to RViz to keep transforms visible.
        """
        publish_detections_to_rviz(self.tf_broadcaster, self.objects, self.boxes, self.get_clock())


    def publish_planning_grid(self):
        self.path_planning_grid.header.stamp = self.get_clock().now().to_msg()
        self.planning_grid_publisher.publish(self.path_planning_grid)


    def publish_workspace_grid(self):
        self.workspace_grid.header.stamp = self.get_clock().now().to_msg()
        self.workspace_grid_publisher.publish(self.workspace_grid)


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



    # ---------------- DEBUGGING FUNCTIONS (ignore it) ----------------   

    def mark_grid_path(self, occupancy_grid, grid_path):
        data = occupancy_grid.data
        width = occupancy_grid.info.width        
        for (x, y) in grid_path:
            index = y * width + x
            data[index] = 70  # Mark the path points for debugging        
        
        self.publish_workspace_grid()  # Publish the updated grid


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






