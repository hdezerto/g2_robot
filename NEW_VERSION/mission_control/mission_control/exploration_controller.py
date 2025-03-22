#!/usr/bin/env python

"""
EXPLORATION LOGIC:

1. Initialization:
    - Publish the workspace to RViz (just once).
    - Subscribe to the /detections topic, which will be used to receive the positions of detected 
    objects/boxes/obstacles from the 3D camera.
    - Define the exploration points in the workspace and the order in which they should be visited.  

2. Exploration:
    - Pick the next exploration point:
        - Add the detected objects/boxes/obstacles to the occupancy grid map.
        - With the updated occupancy grid map, compute a path to the point and move to it if a path is found.
        - Publish the path to RViz.
    - While moving, when something is published to /detections:
        - If it is a new object/box, add it to the respective list and republish the positions and labels to RViz.
        # - If it is a new obstacle, add it to the respective list and stop the robot. Recompute the path to the point (now with
        #  the new obstacle in the occupancy grid map) and move to it if a path is found.
        - If it is a previously detected object/box, ignore it. Alternatively, use it to improve the
          detected object/box position.

4. End exploration (when no more exploration points remain):
    - Write the map file with the detected objects/boxes.
     
"""




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from mission_control_utils import publish_workspace, compute_path, is_in_collision
from occupancy_grid_map import initialize_occupancy_grid, inflate_occupied_cells, grid_to_real_coordinates, update_path_planning_grid
from nav_msgs.msg import OccupancyGrid
#from detection.msg import DetectionMsg 


from enum import Enum, auto
from nav_msgs.msg import Path
from std_msgs.msg import Bool

import time # DEBUG

# -------- Tunable parameters --------
#EXPLORATION_STEP = 7  # Step size for generating exploration points [cells]
EXPLORATION_STEP = 15 # DEBUGGING
POSITION_THRESHOLD = 0.1  # Threshold for considering two detections as the same [m]
# ------------------------------------


# ------------------------------- ExplorationState class -------------------------------
class ExplorationState(Enum):
    INIT = auto()
    GET_NEXT_EXPLORATION_POINT = auto()
    START_MOVING = auto()
    MOVING = auto()
    END_EXPLORATION = auto()


# ------------------------------- ExplorationController class -------------------------------
class ExplorationController(Node):

    def run(self):
        while rclpy.ok():
            if self.state == ExplorationState.GET_NEXT_EXPLORATION_POINT:
                self.get_next_exploration_point()
            elif self.state == ExplorationState.START_MOVING:
                self.start_moving()
            elif self.state == ExplorationState.MOVING:
                rclpy.spin_once(self)
                #rclpy.spin_once(self, timeout_sec=1) # DEBUG
                #self.get_logger().info('Inside MOVING')  # DEBUG
            elif self.state == ExplorationState.END_EXPLORATION:
                self.end_exploration()
                break

    # ------------------- STATE FUNCTIONS -------------------
    def __init__(self):
        # State to initialize the node
        super().__init__('ExplorationController_node')
        self.state = ExplorationState.INIT
        # Publish the workspace to RViz
        self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', 10)
        publish_workspace(self.workspace_publisher, self.get_clock())

        # Subscribe to the /lidar_occupancy_grid topic
        self.lidar_occupancy_grid_subscriber = self.create_subscription(
            OccupancyGrid,
            '/lidar_occupancy_grid',
            self.lidar_occupancy_grid_callback,
            10
        )

        # Subscribe to the /detections topic
        # self.detections_subscriber = self.create_subscription(
        #     DetectionMsg,
        #     '/detections',
        #     self.detections_callback,
        #     5
        # )

        # Publisher for the exploration grid (only with workspace and computed exploration points)
        self.exploration_grid_publisher = self.create_publisher(OccupancyGrid, '/exploration_occupancy_grid', 10)

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
        
        # DEBUG:
        real_world_points = grid_to_real_coordinates(self.exploration_points, self.exploration_occupancy_grid)
        self.get_logger().info(f'Exploration points (grid): {self.exploration_points}')
        formatted_real_world_points = [(f"{x:.2f}", f"{y:.2f}") for x, y in real_world_points]
        self.get_logger().info(f'Exploration points (real world): {formatted_real_world_points}')
    

        self.exploration_point_index = 0
        self.exploration_point = None
        self.current_position = None
        self.detected_objects = [] # List of tuples (x, y, category)
        self.detected_boxes = [] # List of tuples (x, y, theta)
        # Grid where the path will be computed. Obtained by adding the detected objects/boxes to the latest lidar grid.
        self.path_planning_grid = None  

        # Subscribe to the /goal_reached topic
        self.reached_destination_subscriber = self.create_subscription(
            Bool,
            '/goal_reached',
            self.reached_destination_callback,
            10
        )

        # Publisher for the path (for RViz and motion controller)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        # Publisher for the stop command (to motion controller)
        self.stop_publisher = self.create_publisher(Bool, '/stop_motion', 10)

        self.state = ExplorationState.GET_NEXT_EXPLORATION_POINT
        #self.state = ExplorationState.MOVING # DEBUG detection


    def get_next_exploration_point(self):
        # Get the next exploration point from the list (if it exists)
        if self.exploration_point_index < len(self.exploration_points):
            self.exploration_point = self.exploration_points[self.current_point_index] # Get grid coordinates of the next exploration point
            self.exploration_point_index += 1
            self.state = ExplorationState.START_MOVING
        else: # No more points to explore
            self.get_logger().info('No more exploration points. Ending exploration.')
            self.state = ExplorationState.END_EXPLORATION
    

    def start_moving(self):

        # Debug. Just print and sleep
        # self.get_logger().info('Moving to point')  # DEBUG
        # time.sleep(2)  # DEBUG
        # self.state = ExplorationState.MOVING

        # Compute or recompute the path to the exploration point (in case a collision is detected) and move to it
        self.grid_path, path = compute_path(self.current_position, self.exploration_point, self.path_planning_grid)
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
        #Handle the detection message
        if self.is_new_detection(msg):
            self.stop_robot()
            if msg.type == 'OBJECT':
                self.detected_objects.append((msg.x, msg.y, msg.cat))
            else:  # msg.type == 'BOX'
                self.detected_boxes.append((msg.x, msg.y, msg.theta))
            #else: # 'OBSTACLE' case
            #    self.detected_obstacles.append((msg.x, msg.y))
            self.publish_detections_to_rviz() # Update RViz with the new detections (labels and positions)
            self.state = ExplorationState.START_MOVING
        else:
            # Ignore previously detected objects/obstacles (state remains the same)
            pass

    
    # TO FINISH
    def lidar_occupancy_grid_callback(self, msg):
        self.get_logger().info('Received new lidar occupancy grid.') # DEBUG
        latest_lidar_occupancy_grid = msg 
        # Update the planning grid with the latest lidar occupancy grid and the detected objects/boxes
        update_path_planning_grid(self.path_planning_grid, latest_lidar_occupancy_grid, self.detected_objects + self.detected_boxes)

        get_current_position()  # Get the current position of the robot (from odometry or localization)
        if check_collision(self.path_planning_grid, self.grid_path, self.current_position):
            self.get_logger().info('Collision detected. Recomputing path.')
            self.state = ExplorationState.START_MOVING
        else:
            pass  # No collision, keep moving (state remains the same)
        

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
    
    
    def get_current_position(self):
        # TO DO
        pass

    # ------------------- UTILS (specific to ExplorationController) ------------------- 

    def publish_detections_to_rviz(self):
        # Implementation to publish detections to RViz
        # TO DO
        pass
    
    
    def write_map_file(self):
        # TO DO
        pass

    
    # TO FINISH
    def is_new_detection(self, msg):
        # Select the appropriate list based on the detection type
        detected_list = {
            'OBJECT': self.detected_objects,
            'BOX': self.detected_boxes,
            'OBSTACLE': self.detected_obstacles
        }[msg.type]  # No need for .get() since the type is always valid
    
        # Check if the detection already exists
        for detection in detected_list:
            if ((detection[0] - msg.x) ** 2 + (detection[1] - msg.y) ** 2) ** 0.5 < POSITION_THRESHOLD:
                return False  # Detection already exists
    
        return True  # No match, so it is a new detection


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




