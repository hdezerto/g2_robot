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
    check_valid_observation_position,
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
from std_msgs.msg import Bool, Int32

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from detection_interfaces.msg import DetectionMsg
import os  # To get the current directory

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler
import numpy as np

import time

from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from my_custom_interfaces.srv import Pickup
from std_srvs.srv import Trigger
import time




"""
NOTES (HUGO):
- Check how the motion controller executes the yaw from the PoseStamped() of the Path()

- I assume the observation point and drop point (near a box) are not inflated on the planning map,
otherwise A* fails. I also assume the closest uninflated point to a box is good enough for dropping.


"""


# self.get_logger().info('HERE DEBUG!!!')  # DEBUG


# -------- Tunable parameters --------
MAP_FILE_NAME = "map_1.tsv"  # Name of the map file to read
SCANNING_TIME = 3.0  # Time to scan the environment [s]
DETECTION_TIMEOUT = 5.0  # Timeout for waiting for object detection [s]
# NOTE: OBSERVATION_DISTANCE >= PICK_DISTANCE
OBSERVATION_DISTANCE = 0.60  # Distance to the object for observation [m]
PICK_DISTANCE_X = 0.12  # Distance to the object for pick [m]
PICK_DISTANCE_Y = 0.02  # Distance to the object for pick [m]
# PLACE_DISTANCE = ??  # Distance to the box for drop [m]
# ------------------------------------


# ------------------------------- State class -------------------------------
class State(Enum):
    TESTING = auto()  # To delete later

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
            # self.get_logger().info(f"Current state: {self.state.name}")
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
        super().__init__("CollectionController_node")
        self.state = State.INIT

        # Interface with RViz:
        latched_qos = QoSProfile(
            depth=1
        )  # Define a shared QoS profile for latched publishers
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.workspace_publisher = self.create_publisher(
            PolygonStamped, "/workspace_polygon", latched_qos
        )
        self.workspace_grid_publisher = self.create_publisher(
            OccupancyGrid, "/workspace_occupancy_grid", latched_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)  # For publishing objects/boxes
        self.planning_grid_publisher = self.create_publisher(
            OccupancyGrid, "/planning_grid", latched_qos
        )
        self.observation_pos_marker_publisher = self.create_publisher(
            Marker, "/observation_pos", 10
        )
        # Interface with mapper:
        self.mapper_occupancy_grid_subscriber = self.create_subscription(
            OccupancyGrid,
            "/mapper_occupancy_grid",
            self.mapper_occupancy_grid_callback,
            1,
        )
        # Interface with motion controller:
        self.path_publisher = self.create_publisher(Path, "/planned_path", 10)
        self.reached_destination_subscriber = self.create_subscription(
            Bool, "/reached_destination", self.reached_destination_callback, 10
        )
        self.stop_publisher = self.create_publisher(Bool, "/stop_motion", 10)
        # Interface with arm:
        self.arm_command_publisher = self.create_publisher(Int32, "/arm_controller", 10)
        self.arm_feedback_publisher = self.create_publisher(
            Bool, "/arm_controller_feedback", 10
        )
        self.arm_feedback_subscriber = self.create_subscription(
            Bool, "/arm_controller_feedback", self.arm_feedback_callback, 10
        )

        self.pickupClient = self.create_client(Pickup, 'pickup')
        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub',10)
        self.dropClient = self.create_client(Trigger, 'drop')
        # TODO: change arm interface if needed

        # Initialize TransformListener to get current position of the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True
        )  # spin_thread=True to run the listener in a separate thread

        # Publish the workspace to RViz
        publish_workspace(self.workspace_publisher, self.get_clock())

        # Clean grid with workspace boundaries. Used to initialize planning grid and to mark the grid path.
        # Only published inside mark_grid_path()
        self.workspace_grid = initialize_occupancy_grid()

        # -- Variables for collection --
        self.current_pose = (
            0,
            0,
            0,
        )  # Initial position (x, y, yaw) in real world coordinates
        self.current_grid_position = real_to_grid_coordinates(
            [(self.current_pose[0], self.current_pose[1])], self.workspace_grid
        )[0]
        self.objects = []
        self.boxes = []
        self.read_map_file()  # Read the map file and populate the objects and boxes lists
        self.grid_path = []  # Path in grid coordinates
        self.path_planning_grid = update_path_planning_grid(
            self.workspace_grid, self.objects, self.boxes
        )  # Grid where the path will be computed
        self.publish_planning_grid()  # Publish the initial grid to RViz
        self.next_object = {"position": None, "category": None, "index": None}
        self.closest_box = None  # (x, y, theta) in real world coordinates
        self.destination_pose = None  # (x, y, theta) in real world coordinates. Only for observation and drop poses
        self.detected_position = None  # (x, y) in real world coordinates. To store the observed position of the object
        
        self.get_logger().info("Waiting 3 sec...")
        time.sleep(3)  # Wait for all nodes to be ready and the TF buffer to populate

        self.task = None  # State.PICK or State.DROP

        # self.state = State.TESTING # DEBUGGING
        # self.state = State.SCANNING
        self.state = State.GET_NEXT_OBJECT

        # Timer to periodically publish objects/boxes to RViz
        self.detections_timer = self.create_timer(
            0.5, self.publish_transforms_periodically
        )

    # ------------------- STATE FUNCTIONS -------------------
    # MIGHT BE REMOVED!
    def scanning(self, duration):
        """
        Process callbacks for a specified duration, just for initial observation of the environment (using lidar)
        """
        self.get_logger().info(f"Observing for {duration} seconds...")
        start_time = self.get_clock().now().nanoseconds() / 1e9  # Start time in seconds

        while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
            rclpy.spin_once(self)

        self.get_logger().info(
            "Finished scanning. SCANNING -> GET_NEXT_OBJECT"
        )  # DEBUG
        self.state = State.GET_NEXT_OBJECT  # Now it can get the first object

    def get_next_object(self):
        if not self.objects:  # Empty list of objects
            self.get_logger().info(
                "No more objects to collect. GET_NEXT_OBJECT -> END_COLLECTION"
            )
            self.state = State.END_COLLECTION
            return

        self.task = State.PICK
        self.compute_closest_object()  # Select the closest object to the current position of the robot

        objects_without = self.objects
        objects_without.pop(self.next_object["index"])
        self.path_planning_grid = update_path_planning_grid(self.workspace_grid, objects_without, self.boxes)
        # Compute the observation point
        observation_pose = self.compute_observation_pose(OBSERVATION_DISTANCE)
        # Returns the observation point with (x_real, y_real, final orientation)

        if observation_pose:
            self.destination_pose = observation_pose  # (x, y, theta) in real world coordinates, where theta is the final orientation to observe
            self.get_logger().info(
                f"Observation point: {self.destination_pose}. GET_NEXT_OBJECT -> PLAN_PATH"
            )
            self.state = State.PLAN_PATH  # Plan the path to the observation point
        else:
            self.get_logger().info(
                "No observation point found. GET_NEXT_OBJECT -> END_COLLECTION"
            )  # DEBUG
            self.state = (
                State.END_COLLECTION
            )  # DEBUG since it should in principle find an observation point

    def plan_path(self):
        self.update_current_pose()  # Update the current pose of the robot
        start = (self.current_grid_position, self.current_pose)
        # self.get_logger().info(f'Start: {start}')  # DEBUG
        goal_grid = real_to_grid_coordinates(
            [self.destination_pose[:2]], self.path_planning_grid
        )[0]
        # self.get_logger().info(f'Goal grid: {goal_grid}')  # DEBUG
        goal = (
            goal_grid,
            self.destination_pose,
        )
        self.get_logger().info(f"Start: {start} | Goal: {goal}")  # DEBUG
        self.grid_path, path = compute_path(
            start,
            goal,
            self.path_planning_grid,
            self.get_clock(),
            logger=self.get_logger(),
        )
        self.get_logger().info(f"Path computed?")  # DEBUG
        if path:  # Path found
            self.path_publisher.publish(
                path
            )  # Publish the path to the motion controller and RViz
            self.get_logger().info("Path published. PLAN_PATH -> MOVING")
            # --------- JUST TO SEE THE GRID PATH ---------
            self.mark_grid_path(self.workspace_grid, self.grid_path)  # DEBUG
            # --------------------------------------
            self.state = State.MOVING
        else:
            self.get_logger().info(
                "No path found. PLAN_PATH -> END_COLLECTION"
            )  # DEBUG
            self.state = (
                State.END_COLLECTION
            )  # DEBUG since it should in principle find an a path

    def mapper_occupancy_grid_callback(self, msg):
        if self.state != State.MOVING:  # Ignore lidar mapping when not moving
            return
        # ------------------ TODO -----------------
        # Use the same code from exploration
        # -----------------------------------------

    def observe_object(self, timeout):
        self.get_logger().info(
            f"Observing for object category {self.next_object['category']} for {timeout} seconds..."
        )

        start_time = self.get_clock().now().nanoseconds / 1e9  # Start time in seconds

        # Callback to process detections
        def detection_callback(msg):
            if (
                msg.cat == self.next_object["category"]
            ):  # Check if the detection matches the desired category
                self.get_logger().info(
                    f"Detected desired object: category {msg.cat} at ({msg.x}, {msg.y}). OBSERVE_OBJECT -> MOVE_TO_PICK"
                )
                self.detected_position = (msg.x, msg.y)
                self.state = State.MOVE_TO_PICK
                self.detections_subscriber.destroy()  # Stop listening to detections

        # Subscribe to the /detections topic
        # self.detections_subscriber = self.create_subscription(DetectionMsg, '/detections', detection_callback, 10)

        # Wait for the detection or timeout
        while self.state == State.OBSERVE_OBJECT:
            rclpy.spin_once(self)  # Process callbacks
            elapsed_time = self.get_clock().now().nanoseconds / 1e9 - start_time
            if elapsed_time > timeout:
                self.detected_position = (0.85, -0.2094)
                self.get_logger().info(
                    "Timeout reached while waiting for object detection. OBSERVE_OBJECT -> END_COLLECTION"
                )
                self.state = State.MOVE_TO_PICK
                # self.state = (
                #     State.END_COLLECTION
                # )  # Transition to END_COLLECTION if no detection is received
                # self.detections_subscriber.destroy()  # Stop listening to detections
                # break

    def move_to_pick(self):
        """
        Creates a straight path from the current position to a position at a certain distance
        from the detected object and publishes the path. Transitions to MOVING_BLINDLY.
        """
        self.update_current_pose()  # Update the robot's current pose

        pick_path = self.get_pickup_path(PICK_DISTANCE_X, PICK_DISTANCE_Y)
        pick_pose = (
            pick_path.poses[-1].pose.position.x,
            pick_path.poses[-1].pose.position.y,
            pick_path.poses[-1].pose.orientation.z,
            pick_path.poses[-1].pose.orientation.w,
        )

        # Publish the path
        self.path_publisher.publish(pick_path)
        self.path_publisher.publish(pick_path)
        self.path_publisher.publish(pick_path)
        self.path_publisher.publish(pick_path)
        self.get_logger().info(
            f"Path to pick position published: Start {self.current_pose} | Goal{pick_pose}. MOVE_TO_PICK -> MOVING_BLINDLY"
        )

        self.state = State.MOVING_BLINDLY

    def move_to_box(self):
        self.task = State.DROP

        # Remove the picked object from the objects list
        removed_object = self.objects.pop(self.next_object["index"])
        self.get_logger().info(f"Removed object from list: {removed_object}")
        
        # Update the planning grid to mark the object's position as free space
        self.path_planning_grid = update_path_planning_grid(
            self.workspace_grid, self.objects, self.boxes
        )
        self.publish_planning_grid()  # Publish the updated grid to RViz

        self.compute_closest_box()  # Select the closest box to the current position of the robot

        boxes_without = self.boxes
        boxes_without.remove(self.closest_box)  # Remove the closest box from the list
        self.path_planning_grid = update_path_planning_grid(self.workspace_grid, self.objects, boxes_without)
        # Compute the drop pose
        drop_pose = self.compute_drop_pose()
        # Computes the closest pose to the robot around the box that is not inflated nor occupied
        # Returns the drop pose with (x_real, y_real, final orientation)
        # ---------------------------------------------

        if drop_pose:
            self.destination_pose = drop_pose  # Set the drop pose as the destination
            self.get_logger().info(
                f"Drop pose: {self.destination_pose}. MOVE_TO_BOX -> PLAN_PATH"
            )
            self.state = State.PLAN_PATH
        else:
            self.get_logger().error(
                "Failed to compute drop pose. MOVE_TO_BOX -> END_COLLECTION"
            )
            self.state = (
                State.END_COLLECTION
            )  # DEBUG since it should in principle find a drop pose

    def pick(self):
        """ msg = Int32()
        msg.data = 1  # 1 for PICK
        self.arm_command_publisher.publish(msg) """
        """ self.state = State.WAIT_FOR_ARM
        self.arm_feedback_publisher.publish(Bool(data=True))  # Reset the feedback to False """

        request = Pickup.Request()
        object_type = "Cube"  # Retrieve from topic?
        request.object_type = object_type
        request.color = "Red" # Example color, mainly for testing/debugging
        angles = [12000,10000,18500,2500]
        servos_angles_times1 = [[3000,12000,12000,12000,12000,12000, 2000,2000,2000,2000,2000,2000],
                            [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]]

        msg1 = Int16MultiArray()
        msg1.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)

        for angles in servos_angles_times1:
            self.get_logger().info(f'Angles: {angles}')
            msg1.data = angles
            self.servos_publisher.publish(msg1)
            self.get_logger().info(f'Published message: {msg1.data}')
            time.sleep(3)

        # Call the service asynchronously and get a future
        future = self.pickupClient.call_async(request)
        self.state = State.WAIT_FOR_ARM
        # Wait for the response from the service
        rclpy.spin_until_future_complete(self, future)

        # Handle the response
        if future.result() is not None:
            self.get_logger().info(f'Success: {future.result().message}')
            success_msg = Bool()
            success_msg.data = future.result().success
            self.arm_feedback_publisher.publish(success_msg)
        else:
            self.get_logger().error('Service call failed')


        #self.get_logger().info("Sent arm command to PICK object. PICK -> WAIT_FOR_ARM")
        

    def drop(self):
        """ msg = Int32()
        msg.data = 2  # 2 for DROP
        self.arm_command_publisher.publish(msg)
        self.get_logger().info("Sent arm command to DROP object. DROP -> WAIT_FOR_ARM")
        self.state = State.WAIT_FOR_ARM """

        servos_angles_times1 = [[11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000],
                                    [11000,12000,3000,12116,6683,11999,2000,2000,2000,2000,2000,2000],
                                    [3000,12000,3000,12116,6683,11999,2000,2000,2000,2000,2000,2000],
                                    [11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]]
            
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        valid_angles = [True, True, True, True]
        if all(valid_angles):        
            for angles in servos_angles_times1:
                msg.data = angles
                print(msg.data)
                self.servos_publisher.publish(msg)
                #self.get_logger().info(f'Published message: {msg.data}')
                time.sleep(3)

        """ request = Trigger.Request()
        # Call the service asynchronously and get a future  
        future = self.dropClient.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Handle the response
        if future.result() is not None:
            self.get_logger().info(f'Success: {future.result().message}')
            #success_msg = Bool()
            #success_msg.data = future.result().success
            #self.arm_feedback_publisher.publish(success_msg)
        else:
            self.get_logger().error('Service call failed') """

        self.state = State.GET_NEXT_OBJECT




    def arm_feedback_callback(self, msg):
        if msg.data:  # True indicates success
            if self.task == State.PICK:
                self.get_logger().info(
                    "Pick operation successful. WAIT_FOR_ARM -> MOVE_TO_BOX"
                )
                self.state = State.MOVE_TO_BOX
            elif self.task == State.DROP:
                self.get_logger().info(
                    "Drop operation successful. WAIT_FOR_ARM -> GET_NEXT_OBJECT"
                )
                self.state = State.GET_NEXT_OBJECT
        else:  # False indicates failure
            self.get_logger().error("Arm operation failed. State -> END_COLLECTION")
            self.state = State.END_COLLECTION  # DEBUG. Improve how to handle failure

    def reached_destination_callback(self, msg):
        self.get_logger().info(f"msg.data")
        if msg.data:  # msg.data is True if the destination was reached
            if self.state == State.MOVING and self.task == State.PICK:
                self.get_logger().info(
                    "Observation position reached. MOVING -> OBSERVE_OBJECT"
                )
                self.state = State.OBSERVE_OBJECT
            elif self.state == State.MOVING and self.task == State.DROP:
                self.get_logger().info("Drop position reached. MOVING -> DROP")
                self.state = State.DROP
            elif self.state == State.MOVING_BLINDLY:
                self.get_logger().info("Pick position reached. MOVING_BLINDLY -> PICK")
                self.state = State.PICK
        else:
            self.get_logger().info(
                f"Failed to reach destination. State: {self.state.name} -> END_COLLECTION"
            )
            self.state = State.END_COLLECTION

    # ------------------- UTILS -------------------
    def create_pick_path(self, pick_distance):
        # Extract current position and detected position
        current_x, current_y, current_theta = self.current_pose
        detected_x, detected_y = self.detected_position

        # Calculate the direction vector and target position
        dx = detected_x - current_x
        dy = detected_y - current_y
        distance = (dx**2 + dy**2) ** 0.5

        scale = (distance - pick_distance) / distance
        target_x = current_x + dx * scale
        target_y = current_y + dy * scale
        target_theta = np.arctan2(dy, dx)  # The last yaw is oriented to the object

        # Create the Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Add start and target poses to the path
        path_msg.poses.append(
            self.create_pose_stamped(current_x, current_y, current_theta)
        )
        path_msg.poses.append(
            self.create_pose_stamped(target_x, target_y, target_theta)
        )

        return path_msg, (target_x, target_y, target_theta)

    def get_pickup_path(self, pickup_tf_x, pickup_tf_y) -> Path:
        """
        Calculate the path for the robot to pick up an object.

        Direct path with only the only pose as the goal point.
        Moves the base_link such that the pickup_place is at the object position.
        Args:
            pickup_tf_x (float): The x translation to go from base_link to pickup_place as an absolute value
            pickup_tf_y (float): The y translation to go from base_link to pickup_place as an absolute value
        Returns:
            Path: A Path message containing the pose where the robot should move to pick up the object.

        """
        object_position = self.detected_position
        current_position = self.current_pose

        # Calculate the orientation of the robot towards the object
        dx = object_position[0] - current_position[0]
        dy = object_position[1] - current_position[1]
        theta = np.arctan2(dy, dx)

        # Calculate the position of the pickup place
        pickup_place_x = (
            object_position[0]
            - pickup_tf_x * np.cos(theta)
            - pickup_tf_y * np.sin(theta)
        )
        pickup_place_y = (
            object_position[1]
            - pickup_tf_x * np.sin(theta)
            + pickup_tf_y * np.cos(theta)
        )

        # Calculate final orientation
        dx = object_position[0] - pickup_place_x
        dy = object_position[1] - pickup_place_y
        theta = np.arctan2(dy, dx)
        q = quaternion_from_euler(0, 0, theta)  # Convert yaw (theta) to quaternion

        # Create the pose, where the base_link should be at
        time = self.get_clock().now().to_msg()
        pickup_pose = PoseStamped()
        pickup_pose.header.stamp = time
        pickup_pose.header.frame_id = "map"
        pickup_pose.pose.position.x = pickup_place_x
        pickup_pose.pose.position.y = pickup_place_y
        pickup_pose.pose.orientation.x = q[0]
        pickup_pose.pose.orientation.y = q[1]
        pickup_pose.pose.orientation.z = q[2]
        pickup_pose.pose.orientation.w = q[3]

        # Create the path message
        path = Path()
        path.header.stamp = time
        path.header.frame_id = "map"
        path.poses.append(
            self.create_pose_stamped(current_position[0], current_position[1], theta)
        )
        path.poses.append(pickup_pose)

        return path

    def create_pose_stamped(self, x, y, theta):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw (theta) to quaternion
        q = quaternion_from_euler(0, 0, theta)
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = q

        return pose

    def compute_closest_object(self):
        self.update_current_pose()  # Update the current pose of the robot
        closest_distance = float("inf")
        closest_idx = None

        # Iterate through the objects to find the closest one
        for idx, (x, y, _) in enumerate(self.objects):
            distance = (
                (x - self.current_pose[0]) ** 2 + (y - self.current_pose[1]) ** 2
            ) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_idx = idx

        self.next_object["position"] = (
            self.objects[closest_idx][0],
            self.objects[closest_idx][1],
        )
        self.next_object["index"] = closest_idx
        self.next_object["category"] = self.objects[closest_idx][2]
        self.get_logger().info(
            f"Closest object selected: Position {self.next_object['position']} | Category {self.next_object['category']}."
        )

    def compute_closest_box(self):
        self.update_current_pose()  # Update the robot's current pose
        closest_distance = float("inf")
        closest_box = None

        # Iterate through the boxes to find the closest one
        for box in self.boxes:
            box_x, box_y, _ = box
            distance = (
                (box_x - self.current_pose[0]) ** 2
                + (box_y - self.current_pose[1]) ** 2
            ) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_box = box

        self.closest_box = closest_box  # (x, y, theta) in real world coordinates
        self.get_logger().info(f"Closest box selected: Pose {self.closest_box}.")

    def update_current_pose(self):
        # Update the current pose of the robot
        self.current_pose, self.current_grid_position = get_current_pose(
            self.tf_buffer, self.get_logger(), self.workspace_grid
        )
        if self.current_pose is None:
            self.get_logger().info("Failed to get current pose!")
        # self.get_logger().info(f'Current pose (real): {self.current_pose}  | (grid): {self.current_grid_position}')  # DEBUG

    def publish_transforms_periodically(self):
        """
        Periodically publish objects/boxes to RViz to keep transforms visible.
        """
        publish_detections_to_rviz(
            self.tf_broadcaster, self.objects, self.boxes, self.get_clock()
        )

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
        package_share_dir = get_package_share_directory("mission_control")
        file_path = os.path.join(
            get_package_share_directory("mission_control"), "workspaces", "map_1.tsv"
        )
        # file_path = os.path.join(package_share_dir, 'resource', file_name)
        self.get_logger().info(f"Reading map file '{file_name}'...")
        if not os.path.exists(file_path):
            self.get_logger().error(f"Map file '{file_path}' does not exist.")
            return
        # else:
        #     self.get_logger().info(f"Map file '{file_path}' exists.")

        with open(file_path, "r") as file:
            for line in file:
                parts = line.strip().split("\t")[:4]  # Only consider the first 4 parts
                category, x, y, angle = parts
                x, y = float(x) / 100, float(y) / 100  # Convert cm to m
                angle = float(angle)  # Convert string to float

                if category == "B":  # Box
                    self.boxes.append((x, y, angle))
                else:  # Object
                    self.objects.append((x, y, int(category)))

        self.get_logger().info(f"Map file '{file_name}' has been read successfully.")

    def compute_observation_pose(
        self,
        observing_distance,
    ) -> tuple[float, float, float] | None:
        """
        Plan the path to the position for the reobservation of the object.

        1. Compute 12 points around the object with the observation distance. These are the possible positions.
        2. Check if these positions are occupied or don't have a clear view to the object. If so, remove them.
        3. If no possible positions are left, return False.
        4. Compute the path to all possible positions using A* and keep the one with the lowest cost.
        5. Simplify the path and make sure that the orientation of the last waypoint is facing towards the object.

        Args:
            observing_distance (float): Distance to the object for observation [m].

        Returns:
            tuple[float, float, float] | None: The position (x, y, theta) of the observation point or None if no valid position is found.
        """
        object_position = self.next_object["position"]
        current_grid_position = self.current_grid_position
        collection_occupancy_grid = self.path_planning_grid

        # Init
        object_x, object_y = object_position
        object_grid_position = real_to_grid_coordinates(
            [object_position], collection_occupancy_grid
        )[0]
        possible_positions = []

        # positions are in a circle around the object every 30 degrees
        for i in range(0, 360, 90):
            x = object_x + observing_distance * np.cos(np.radians(i))
            y = object_y + observing_distance * np.sin(np.radians(i))
            possible_positions.append((x, y))
        possible_grid_positions = real_to_grid_coordinates(
            possible_positions, collection_occupancy_grid
        )
        feasible_positions = []
        feasible_grid_positions = []

        # remove occupied positions or position without a clear view to the object
        for i, grid_position in enumerate(possible_grid_positions):
            if check_valid_observation_position(
                collection_occupancy_grid,
                object_position,
                possible_positions[i],
                grid_position,
                logger=self.get_logger(),
            ):
                feasible_positions.append(possible_positions[i])
                feasible_grid_positions.append(grid_position)

        # Visualisation
        non_feasible_positions = []
        for possible_position in possible_positions:
            if possible_position not in feasible_positions:
                non_feasible_positions.append(possible_position)
        # If no possible positions are left, return False
        if not feasible_positions:
            self.get_logger().warn("No valid observation positions found.")
            return None
        self.visualize_observation_positions(feasible_positions, non_feasible_positions)

        # Calculate the path to all possible positions and keep the one with the lowest cost
        # minimum_cost = float("inf")
        # path = []
        # final_position = []
        # for i, goal_grid_position in enumerate(possible_grid_positions):
        #     path_points, cost = compute_grid_path(
        #         current_grid_position,
        #         goal_grid_position,
        #         collection_occupancy_grid,
        #         return_cost=True,
        #     )
        #     if cost < minimum_cost:
        #         minimum_cost = cost
        #         path = path_points
        #         final_position = possible_positions[i]

        # Take closest position to current position
        closest_distance = float("inf")
        final_position = []
        found_way = False
        # while possible_positions and (not found_way):
        for i, goal_position in enumerate(possible_positions):
            distance = np.sqrt(
                (goal_position[0] - self.current_pose[0]) ** 2
                + (goal_position[1] - self.current_pose[1]) ** 2
            )
            if distance < closest_distance:
                closest_distance = distance
                final_position = possible_positions[i]
                

        if final_position:
            # Add final orientation
            dx = -final_position[0] + object_x
            dy = -final_position[1] + object_y
            theta = np.arctan2(dy, dx)  # Angle to the object
            final_position = (final_position[0], final_position[1], theta)
            return final_position

    def compute_drop_pose(self, drop_distance=0.22):
        # self.get_logger().info("Computing drop pose...")
        box_position = self.closest_box
        current_grid_position = self.current_grid_position
        collection_occupancy_grid = self.path_planning_grid
        # self.get_logger().info("got box pos")
        # Init
        box_x, box_y = box_position[:2]
        possible_positions = []
        self.get_logger().info(f"Make circle...")
        # positions are in a circle around the object every 30 degrees
        for i in range(0, 360, 90):
            x = box_x + drop_distance * np.cos(np.radians(i))
            y = box_y + drop_distance * np.sin(np.radians(i))
            possible_positions.append((x, y))
        # self.get_logger().info(f"Possible drop positions: {possible_positions}")
        possible_grid_positions = real_to_grid_coordinates(
            possible_positions, collection_occupancy_grid
        )
        feasible_positions = []
        feasible_grid_positions = []
        self.get_logger().info(f"Conversion to grid done")
        # remove occupied positions or position without a clear view to the object
        for i, grid_position in enumerate(possible_grid_positions):
            # Check if the position is within bounds
            width = collection_occupancy_grid.info.width
            height = collection_occupancy_grid.info.height
            x, y = grid_position
            # self.get_logger().info(
            #     f"Possible Position {possible_position} with grid position {possible_grid_position}."
            # )

            if not (
                (x < 0 or x >= width or y < 0 or y >= height)
                or (collection_occupancy_grid.data[y * width + x] > 40)
            ):
                feasible_positions.append(possible_positions[i])
                feasible_grid_positions.append(grid_position)
        # self.get_logger().info(f"Feasible positions: {feasible_positions}")
        # If no possible positions are left, return False
        if not feasible_positions:
            self.get_logger().info("No valid DropOff positions found.")
            return None
        
        # Visualisation
        non_feasible_positions = []
        for possible_position in possible_positions:
            if possible_position not in feasible_positions:
                non_feasible_positions.append(possible_position)
        # If no possible positions are left, return False
        if not feasible_positions:
            self.get_logger().warn("No valid observation positions found.")
            return None
        self.visualize_observation_positions(feasible_positions, non_feasible_positions)

        drop_off_position = min(
            feasible_positions,
            key=lambda pos: np.linalg.norm(
                np.array(pos) - np.array(self.current_pose[:2])
            ),
        )
        # Add final orientation
        dx = -drop_off_position[0] + box_x
        dy = -drop_off_position[1] + box_y
        theta = np.arctan2(dy, dx)  # Angle to the object

        return (drop_off_position[0], drop_off_position[1], theta)

    def visualize_observation_positions(
        self, feasible_positions, non_feasible_positions
    ):
        """
        Visualize the observation positions in RViz using markers.
        Args:
            feasible_positions (list): List of feasible observation positions.
            non_feasible_positions (list): List of non-feasible observation positions.
        """
        markers = Marker()
        markers.header.frame_id = "map"
        markers.header.stamp = self.get_clock().now().to_msg()
        markers.ns = "observation_positions"
        markers.id = 0
        markers.type = Marker.POINTS
        markers.action = Marker.ADD
        markers.scale.x = 0.1
        markers.scale.y = 0.1
        markers.color.a = 1.0

        # Example valid and invalid positions (Replace with actual data)
        valid_positions = feasible_positions
        invalid_positions = non_feasible_positions

        # Mark valid positions with blue
        markers.color.r = 0.0
        markers.color.g = 0.0
        markers.color.b = 1.0
        for pos in valid_positions:
            p = Point()
            p.x, p.y, p.z = pos[0], pos[1], 0.0
            markers.points.append(p)

        # Mark invalid positions with black
        invalid_markers = Marker()
        invalid_markers.header = markers.header
        invalid_markers.ns = markers.ns
        invalid_markers.id = 1
        invalid_markers.type = Marker.POINTS
        invalid_markers.action = Marker.ADD
        invalid_markers.scale.x = markers.scale.x
        invalid_markers.scale.y = markers.scale.y
        invalid_markers.color.a = 1.0
        invalid_markers.color.r = 0.0
        invalid_markers.color.g = 0.0
        invalid_markers.color.b = 0.0

        for pos in invalid_positions:
            p = Point()
            p.x, p.y, p.z = pos[0], pos[1], 0.0
            invalid_markers.points.append(p)

        # Publish both markers
        self.observation_pos_marker_publisher.publish(markers)
        self.observation_pos_marker_publisher.publish(invalid_markers)

    # ---------------- DEBUGGING FUNCTIONS (ignore it) ----------------

    def mark_grid_path(self, occupancy_grid, grid_path):
        data = occupancy_grid.data
        width = occupancy_grid.info.width
        for x, y in grid_path:
            index = y * width + x
            data[index] = 70  # Mark the path points for debugging

        self.publish_workspace_grid()  # Publish the updated grid


# ------------------------------- Main function -------------------------------


def main(args=None):
    rclpy.init(args=args)
    exploration_controller = CollectionController()
    exploration_controller.get_logger().info(
        "CollectionController node has been created."
    )

    try:
        exploration_controller.run()
    except Exception as e:
        exploration_controller.get_logger().error(f"An error occurred: {e}")
    finally:
        exploration_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
