import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from robp_interfaces.msg import DutyCycles

import time
import math
import numpy as np

class TestStop(Node):
    def __init__(self):
        super().__init__('test_stop')
        self.get_logger().info('Initializing test_stop node...')

        self.stop_pub = self.create_publisher(Bool, "/stop_motion", 10)

        self.continue_pub = self.create_publisher(Path, "/planned_path", 10)
        self.get_logger().info("Initialized publishers.")

    def stop(self, yesno):
        msg = Bool()
        msg.data = yesno
        self.stop_pub.publish(msg)
        self.get_logger().info("Stopping motion...")

    # def continue_motion(self):
    #     self.get_logger().info("Get ready to continue motion...")
    #     msg = Path()
    #     msg.header.frame_id = "map"
    #     msg.poses = []  # Initialize poses as an empty list
    #     current_time = self.get_clock().now().to_msg()
    #     msg.header.stamp = current_time

    #     pose = PoseStamped()
    #     pose.header.frame_id = "map"
    #     pose.header.stamp = current_time

    #     pose2 = PoseStamped()
    #     pose2.header.frame_id = "map"
    #     pose2.header.stamp = current_time
    #     pose2.pose.position.x = 1
    #     pose2.pose.position.y = 1

    #     msg.poses.append(pose)
    #     msg.poses.append(pose2)

    #     self.continue_pub.publish(msg)
    #     self.get_logger().info("Continuing motion...")

def main(args=None):
    rclpy.init(args=args)


    node = TestStop()
    node.get_logger().info("MotionController node has started.")


    while rclpy.ok():
        node.stop(True)
        time.sleep(2)
        node.stop(False)
        time.sleep(2)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()