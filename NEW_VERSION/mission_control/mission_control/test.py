# ------------------- TEST publish_detections_to_rviz -------------------

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from mission_control_utils import publish_detections_to_rviz
import numpy as np

class TestPublishDetectionsNode(Node):
    def __init__(self):
        super().__init__('test_publish_detections_node')

        # Initialize the TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to periodically call the test function
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        # Simulate detected objects and boxes
        detected_objects = [
            (1.0, 2.0, '1'),  # Cube at (1.0, 2.0)
            (3.0, 4.0, '2'),  # Sphere at (3.0, 4.0)
            (5.0, 6.0, '3')   # Plushie at (5.0, 6.0)
        ]
        detected_boxes = [
            (7.0, 8.0, 45.0),  # Box at (7.0, 8.0) with 45 degrees rotation
            (9.0, 10.0, 90.0)  # Box at (9.0, 10.0) with 90 degrees rotation
        ]

        # Call the function to test
        publish_detections_to_rviz(
            tf_broadcaster=self.tf_broadcaster,
            detected_objects=detected_objects,
            detected_boxes=detected_boxes,
            clock=self.get_clock()
        )

        self.get_logger().info('Published detections to RViz')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublishDetectionsNode()
    node.get_logger().info('Test Publish Detections Node has started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# --------------------------------------------------



# """

# Node just for testing code. IGNORE IT !

# """




# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Path

# from geometry_msgs.msg import PolygonStamped

# from tf2_ros import StaticTransformBroadcaster
# from geometry_msgs.msg import TransformStamped

# from mission_control_utils import compute_path, publish_workspace
# from occupancy_grid_map import initialize_occupancy_grid, inflate_occupied_cells
# import time


# EXPANSION_RADIUS = 2



# #  ------------ TEST PATH PLANNING ------------

# class TestComputePathNode(Node):
#     def __init__(self):
#         super().__init__('test_compute_path_node')

#          # Initialize the static transform broadcaster
#         self.static_tf_broadcaster = StaticTransformBroadcaster(self)

#         # Broadcast the static transform for the map frame
#         self.broadcast_map_frame()

#         # Publish the workspace to RViz
#         self.workspace_publisher = self.create_publisher(PolygonStamped, '/workspace_polygon', 10)
#         publish_workspace(self.workspace_publisher, self.get_clock())

#         self.grid_publisher = self.create_publisher(OccupancyGrid, 'test_occupancy_grid', 10)
#         self.path_publisher = self.create_publisher(Path, 'test_path', 10)

#         self.timer = self.create_timer(3.0, self.timer_callback)

#         # Initialize the occupancy grid
#         self.occupancy_grid = initialize_occupancy_grid()

#         # ----- Add some obstacles -----
#         self.mark_square(20, 13, 0)
#         self.mark_square(20, 10, 0)
#         self.mark_square(20, 4, 0)

#         self.mark_square(30, 7, 0)
    

#         # Dilate the occupancy grid
#         inflate_occupied_cells(self.occupancy_grid, EXPANSION_RADIUS)

#         # Define start and goal points (in grid coordinates)
#         self.start = (10, 10)
#         self.goal = (40, 10)
    
    
#     def broadcast_map_frame(self):
#         # Create a static transform for the map frame
#         transform = TransformStamped()
#         transform.header.stamp = self.get_clock().now().to_msg()
#         transform.header.frame_id = 'world'  # Parent frame
#         transform.child_frame_id = 'map'    # Child frame
#         transform.transform.translation.x = 0.0
#         transform.transform.translation.y = 0.0
#         transform.transform.translation.z = 0.0
#         transform.transform.rotation.x = 0.0
#         transform.transform.rotation.y = 0.0
#         transform.transform.rotation.z = 0.0
#         transform.transform.rotation.w = 1.0

#         # Broadcast the transform
#         self.static_tf_broadcaster.sendTransform(transform)


#     def mark_square(self, x, y, size):
#         for i in range(-size, size + 1):
#             for j in range(-size, size + 1):
#                 self.occupancy_grid.data[(y + j) * self.occupancy_grid.info.width + (x + i)] = 100

#     def timer_callback(self):
        
#         # Mark the start and goal points in the occupancy grid
#         self.occupancy_grid.data[self.start[1] * self.occupancy_grid.info.width + self.start[0]] = 15
#         self.occupancy_grid.data[self.goal[1] * self.occupancy_grid.info.width + self.goal[0]] = 15
#         # Publish the occupancy grid (with the start and goal points marked)
#         self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
#         self.grid_publisher.publish(self.occupancy_grid)
#         # Return the start and goal points to their original values (for the path computation)
#         self.occupancy_grid.data[self.start[1] * self.occupancy_grid.info.width + self.start[0]] = 0
#         self.occupancy_grid.data[self.goal[1] * self.occupancy_grid.info.width + self.goal[0]] = 0

#         # Compute the path
#         start_time = time.time() # Measure the time to compute the path
#         _, path = compute_path(self.start, self.goal, self.occupancy_grid, self.get_clock)
#         end_time = time.time()
#         computation_time = end_time - start_time

#         self.get_logger().info(f'Computed path with {len(path.poses)} waypoints in {computation_time:.4f} seconds')

#         if path:
#             # Publish the path
#             self.path_publisher.publish(path)
#             self.get_logger().info('Published path')
#         else:
#             self.get_logger().info('No path found')



# def main(args=None):
#     rclpy.init(args=args)
#     node = TestComputePathNode()
#     node.get_logger().info('Test Compute Path Node has started')
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
