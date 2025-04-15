import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
import numpy as np


class PickupPathTester(Node):
    def __init__(self):
        super().__init__("pickup_path_tester")

        # Publishers for RViz visualization
        self.path_publisher = self.create_publisher(Path, "/test_pickup_path", 10)
        self.marker_publisher = self.create_publisher(Marker, "/test_markers", 10)

    def test_pickup_path(
        self, object_position, current_position, pickup_tf_x, pickup_tf_y
    ):
        """
        Test the get_pickup_path function by simulating an object and robot position.

        Args:
            object_position (tuple): The (x, y) position of the object.
            current_position (tuple): The (x, y, theta) position of the robot.
            pickup_tf_x (float): The x translation to go from base_link to pickup_place.
            pickup_tf_y (float): The y translation to go from base_link to pickup_place.
        """
        # Calculate the orientation of the robot towards the object
        dx = object_position[0] - current_position[0]
        dy = object_position[1] - current_position[1]
        theta = np.arctan2(dy, dx)
        q = quaternion_from_euler(0, 0, theta)

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

        # Create the pose for the pickup place
        pickup_pose = PoseStamped()
        pickup_pose.header.stamp = self.get_clock().now().to_msg()
        pickup_pose.header.frame_id = "map"
        pickup_pose.pose.position.x = pickup_place_x
        pickup_pose.pose.position.y = pickup_place_y
        pickup_pose.pose.orientation.x = q[0]
        pickup_pose.pose.orientation.y = q[1]
        pickup_pose.pose.orientation.z = q[2]
        pickup_pose.pose.orientation.w = q[3]

        # Create the path message
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # Add the current position and pickup place to the path
        path.poses.append(self.create_pose_stamped(*current_position))
        path.poses.append(pickup_pose)

        # Publish the path
        self.path_publisher.publish(path)

        # Visualize the object, current position, and pickup place in RViz
        self.publish_marker(
            object_position[0], object_position[1], 0, (1.0, 0.0, 0.0)
        )  # Red for object
        self.publish_marker(
            current_position[0], current_position[1], 1, (0.0, 0.0, 1.0)
        )  # Blue for robot
        self.publish_marker(
            pickup_place_x, pickup_place_y, 2, (0.0, 1.0, 0.0)
        )  # Green for pickup place

        self.get_logger().info(f"Object Position: {object_position}")
        self.get_logger().info(f"Current Position: {current_position}")
        self.get_logger().info(
            f"Pickup Place: ({pickup_place_x}, {pickup_place_y}, {theta})"
        )

    def create_pose_stamped(self, x, y, theta):
        """
        Create a PoseStamped message.

        Args:
            x (float): The x-coordinate.
            y (float): The y-coordinate.
            theta (float): The yaw angle.

        Returns:
            PoseStamped: The PoseStamped message.
        """
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

    def publish_marker(self, x, y, marker_id, color, frame_id="map"):
        """
        Publish a marker to RViz to visualize a position.

        Args:
            x (float): The x-coordinate of the marker.
            y (float): The y-coordinate of the marker.
            marker_id (int): A unique ID for the marker.
            color (tuple): The (r, g, b) color of the marker.
            frame_id (str): The frame ID for the marker.
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pickup_test"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2  # Slightly above the grid
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Size of the sphere
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    tester = PickupPathTester()

    try:
        # Test parameters
        object_position = (1.0, 1.0)  # Example object position
        random_angle = np.random.uniform(0, 2 * np.pi)
        current_x = object_position[0] + 0.5 * np.cos(random_angle)
        current_y = object_position[1] + 0.5 * np.sin(random_angle)
        current_position = (
            current_x,
            current_y,
            0.0,
        )  # Example robot position (x, y, theta)
        pickup_tf_x = 0.17  # Example pickup x offset
        pickup_tf_y = 0.02  # Example pickup y offset

        tester.test_pickup_path(
            object_position, current_position, pickup_tf_x, pickup_tf_y
        )

        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Shutting down PickupPathTester...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
