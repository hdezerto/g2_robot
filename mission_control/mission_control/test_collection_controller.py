import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from mission_control.mission_control_utils import check_valid_observation_position
import numpy as np


class VisualizationNode(Node):
    def __init__(self):
        super().__init__("visualization_node")
        self.path_publisher = self.create_publisher(Path, "/mock_path", 10)
        self.pose_publisher = self.create_publisher(PoseStamped, "/mock_pose", 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, "/mock_occupancy_grid", 10
        )
        self.marker_publisher = self.create_publisher(Marker, "/mock_markers", 10)

    def publish_path(self, path):
        self.path_publisher.publish(path)

    def publish_pose(self, pose):
        self.pose_publisher.publish(pose)

    def publish_grid(self, grid):
        self.grid_publisher.publish(grid)

    def publish_marker(self, marker):
        self.marker_publisher.publish(marker)


def create_mock_occupancy_grid():
    """
    Create a mock occupancy grid for a 5x5 meter workspace with obstacles and an object.
    """
    width, height, resolution = 50, 50, 0.1  # 5x5 meters, 10 cm resolution
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.data = [0] * (width * height)

    # Add an obstacle (line of occupied cells closer to the object)
    for x in range(12, 18):  # Obstacle from (1.2, 2.2) to (1.8, 2.2)
        grid.data[22 * width + x] = 100  # Occupancy = 100 for the obstacle

    # Inflate the obstacle (30 cm around it, occupancy = 50)
    for x in range(10, 20):
        for y in range(20, 25):
            if grid.data[y * width + x] == 0:
                grid.data[y * width + x] = 50

    # Add the object (occupancy = 99, closer to the obstacle)
    object_x, object_y = 15, 18  # Grid cell (1.5, 1.2)
    grid.data[object_y * width + object_x] = 99

    # Inflate the object (30 cm around it, occupancy = 30)
    for x in range(object_x - 3, object_x + 4):
        for y in range(object_y - 3, object_y + 4):
            if grid.data[y * width + x] == 0:
                grid.data[y * width + x] = 30

    return grid, (
        object_x / 10,
        object_y / 10,
    )  # Return the grid and the object's real-world position


def create_marker(x, y, marker_id, color, frame_id="map"):
    """
    Create a marker to display a position as an "X" in RViz2.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.ns = "positions"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.2  # Slightly above the grid
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.3  # Size of the "X"
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.text = "X"
    return marker


def compute_observation_pose(
    object_position, occupancy_grid, observing_distance, node, current_position
):
    """
    Compute the observation position dynamically based on the occupancy grid.
    """
    possible_positions = [
        (
            object_position[0] + observing_distance * np.cos(np.radians(i)),
            object_position[1] + observing_distance * np.sin(np.radians(i)),
        )
        for i in range(0, 360, 30)
    ]
    node.get_logger().info(f"Number of possible positions: {len(possible_positions)}")
    node.get_logger().info(f"Possible positions: {possible_positions}")

    # origin_x = occupancy_grid.info.origin.position.x
    # origin_y = occupancy_grid.info.origin.position.y
    # resolution = occupancy_grid.info.resolution
    # possible_grid_positions = []
    # for x, y in possible_positions:
    #     real_x = origin_x + x * resolution
    #     real_y = origin_y + y * resolution
    #     possible_grid_positions.append((real_x, real_y))
    # node.get_logger().info(f"Possible grid positions: {possible_grid_positions}")
    possible_grid_positions = [
        (
            int(pos[0] / occupancy_grid.info.resolution),
            int(pos[1] / occupancy_grid.info.resolution),
        )
        for pos in possible_positions
    ]

    valid_positions = []
    chosen_position = None

    for idx, (pos, grid_pos) in enumerate(
        zip(possible_positions, possible_grid_positions)
    ):
        # Ensure all positions are displayed, even if invalid
        if (
            0 <= grid_pos[0] < occupancy_grid.info.width
            and 0 <= grid_pos[1] < occupancy_grid.info.height
        ):
            if check_valid_observation_position(
                collection_occupancy_grid=occupancy_grid,
                object_position=object_position,
                possible_position=pos,
                possible_grid_position=grid_pos,
                logger=node.get_logger(),
            ):
                valid_positions.append(pos)
                node.get_logger().info(f"Valid position at {pos} (grid: {grid_pos})")
                # Publish valid positions in blue
                valid_marker = create_marker(pos[0], pos[1], idx, (0.0, 0.0, 1.0))
                node.publish_marker(valid_marker)
            else:
                node.get_logger().info(f"Invalid position at {pos} (grid: {grid_pos})")
                # Publish invalid positions in black
                invalid_marker = create_marker(pos[0], pos[1], idx, (0.0, 0.0, 0.0))
                node.publish_marker(invalid_marker)
        else:
            node.get_logger().info(
                f"Out of bounds position at {pos} (grid: {grid_pos})"
            )
            # Publish out-of-bounds positions in gray
            out_of_bounds_marker = create_marker(pos[0], pos[1], idx, (0.5, 0.5, 0.5))
            node.publish_marker(out_of_bounds_marker)

    if valid_positions:
        node.get_logger().info(f"Number of valid positions: {len(valid_positions)}")
        # Choose the closest valid position to the object
        chosen_position = min(
            valid_positions,
            key=lambda p: np.sqrt(
                (p[0] - current_position[0]) ** 2 + (p[1] - current_position[1]) ** 2
            ),
        )
        dx = object_position[0] - chosen_position[0]
        dy = object_position[1] - chosen_position[1]
        theta = np.arctan2(dy, dx)
        chosen_position = (chosen_position[0], chosen_position[1], theta)

        # Publish the chosen position in green
        chosen_marker = create_marker(
            chosen_position[0], chosen_position[1], 1000, (0.0, 1.0, 0.0)
        )
        node.publish_marker(chosen_marker)

    return chosen_position


def test_observation_position(node):
    # Create the mock occupancy grid
    occupancy_grid, object_position = create_mock_occupancy_grid()

    # Publish the grid for visualization
    occupancy_grid.header.stamp = node.get_clock().now().to_msg()
    occupancy_grid.header.frame_id = "map"
    node.publish_grid(occupancy_grid)

    # Hardcoded current position
    current_position = (2.0, 4.0)  # Example hardcoded position
    node.get_logger().info(f"Hardcoded current position: {current_position}")

    # Compute observation position dynamically
    observing_distance = 0.5  # 50 cm
    observation_position = compute_observation_pose(
        object_position, occupancy_grid, observing_distance, node, current_position
    )

    # Publish markers for current position and observation position
    current_marker = create_marker(
        current_position[0], current_position[1], 999, (1.0, 0.0, 0.0)
    )  # Red "X" for current position
    node.publish_marker(current_marker)
    node.get_logger().info(f"Published current marker at {current_position}")

    if observation_position:
        observation_marker = create_marker(
            observation_position[0], observation_position[1], 1000, (0.0, 1.0, 0.0)
        )  # Green "X" for observation position
        node.publish_marker(observation_marker)
        node.get_logger().info(
            f"Published observation marker at {observation_position}"
        )

        print(f"Current Position: {current_position}")
        print(f"Observation Position: {observation_position}")
    else:
        print("No valid observation position found.")


def main():
    rclpy.init()
    node = VisualizationNode()

    try:
        print("Testing observation position near a wall...")
        test_observation_position(node)

        # Keep the node alive to allow RViz to display the data
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down visualization...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
