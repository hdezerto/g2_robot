
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path


from mission_control_utils import compute_path_to_point, dilate_occupied_cells
from occupancy_grid_map import initialize_occupancy_grid
import time


EXPANSION_RADIUS = 2



#  ------------ TEST PATH PLANNING ------------

class TestComputePathNode(Node):
    def __init__(self):
        super().__init__('test_compute_path_node')

        self.grid_publisher = self.create_publisher(OccupancyGrid, 'test_occupancy_grid', 10)
        self.path_publisher = self.create_publisher(Path, 'test_path', 10)

        self.timer = self.create_timer(3.0, self.timer_callback)

        # Initialize the occupancy grid
        self.occupancy_grid = initialize_occupancy_grid()

        # ----- Add some obstacles -----
        self.mark_square(20, 13, 0)
        self.mark_square(20, 10, 0)
        self.mark_square(20, 4, 0)


        self.mark_square(30, 7, 0)
    

        # Dilate the occupancy grid
        dilate_occupied_cells(self.occupancy_grid, EXPANSION_RADIUS)

        # Define start and goal points (in grid coordinates)
        self.start = (10, 10)
        self.goal = (40, 10)


    def mark_square(self, x, y, size):
        for i in range(-size, size + 1):
            for j in range(-size, size + 1):
                self.occupancy_grid.data[(y + j) * self.occupancy_grid.info.width + (x + i)] = 100

    def timer_callback(self):
        
        # Mark the start and goal points in the occupancy grid
        self.occupancy_grid.data[self.start[1] * self.occupancy_grid.info.width + self.start[0]] = 15
        self.occupancy_grid.data[self.goal[1] * self.occupancy_grid.info.width + self.goal[0]] = 15
        # Publish the occupancy grid (with the start and goal points marked)
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_publisher.publish(self.occupancy_grid)
        # Return the start and goal points to their original values (for the path computation)
        self.occupancy_grid.data[self.start[1] * self.occupancy_grid.info.width + self.start[0]] = 0
        self.occupancy_grid.data[self.goal[1] * self.occupancy_grid.info.width + self.goal[0]] = 0

        # Compute the path
        start_time = time.time() # Measure the time to compute the path
        path = compute_path_to_point(self.start, self.goal, self.occupancy_grid, self.get_clock)
        end_time = time.time()
        computation_time = end_time - start_time

        self.get_logger().info(f'Computed path with {len(path.poses)} waypoints in {computation_time:.4f} seconds')

        if path:
            # Publish the path
            self.path_publisher.publish(path)
            self.get_logger().info('Published path')
        else:
            self.get_logger().info('No path found')



def main(args=None):
    rclpy.init(args=args)
    node = TestComputePathNode()
    node.get_logger().info('Test Compute Path Node has started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
