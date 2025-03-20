#!/usr/bin/env python

import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np

"""
CHECK:

- The path might be too long and cause memory issues.

"""

class Odometry(Node):

    def __init__(self):
        super().__init__("odometry_node")

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Create a QoS profile with a queue size of 1
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        # Use the QoS profile in the subscription
        self.create_subscription(Encoders, "/motor/encoders", self.encoder_callback, qos_profile)
        
        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, "path", 10)
        # Store the path here
        self._path = Path()

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Initialize last total ticks
        self._last_total_ticks_left = None
        self._last_total_ticks_right = None

        # Initialize last time
        self._last_time = None

        # Robot parameters
        self.ticks_per_rev = 48 * 64  # Encoder ticks per wheel revolution
        self.wheel_radius = 0.04921  # Wheel radius [m]
        self.base = 0.295  # Distance between wheels [m]

        # Precompute constants (to improve performance)
        self._ticks_to_radians = 2 * np.pi / self.ticks_per_rev

        # Limit path length to avoid memory issues
        #self.MAX_PATH_LENGTH = 1000


    def encoder_callback(self, msg: Encoders):
        """Takes encoder readings and updates the odometry."""
        current_time = self.get_clock().now()

        # Use the total number of ticks
        total_ticks_left = msg.encoder_left
        total_ticks_right = msg.encoder_right

        if self._last_total_ticks_left is None or self._last_time is None:
            # Initialize the last total ticks with the first received values
            self._last_total_ticks_left = total_ticks_left
            self._last_total_ticks_right = total_ticks_right
            self._last_time = current_time
            return  # Skip the first update
        
        # Compute time difference (dt)
        dt = (current_time - self._last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        if dt <= 0:
            self.get_logger().warn("Time difference (dt) is too small or zero. Skipping this update.")
            return
        self._last_time = current_time

        # Compute tick differences
        delta_ticks_left = total_ticks_left - self._last_total_ticks_left
        delta_ticks_right = total_ticks_right - self._last_total_ticks_right

        # Print difference between the two wheels
        #self.get_logger().info(f"Delta ticks left: {delta_ticks_left}, Delta ticks right: {delta_ticks_right}") # DEBUGGING

        self._last_total_ticks_left = total_ticks_left
        self._last_total_ticks_right = total_ticks_right

        # Wheel angle since last tick
        delta_phi_left = delta_ticks_left * self._ticks_to_radians
        delta_phi_right = delta_ticks_right * self._ticks_to_radians

        # Angular velocity of the wheels
        omega_left = delta_phi_left / dt
        omega_right = delta_phi_right / dt

        # Linear velocities
        v_left = self.wheel_radius * omega_left
        v_right = self.wheel_radius * omega_right

        # Compute linear and angular displacements
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.base
        
        delta_x = v * np.cos(self._yaw) * dt
        delta_y = v * np.sin(self._yaw) * dt
        delta_theta = omega * dt

        self._x += delta_x
        self._y += delta_y
        self._yaw = ( (self._yaw + delta_theta) + np.pi) % (2 * np.pi) - np.pi  # Normalize angle to [-pi, pi)

        # Compute quaternion once and reuse
        q = quaternion_from_euler(0.0, 0.0, self._yaw)

        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self._x, self._y, q)
        self.publish_path(stamp, self._x, self._y, q)

    def broadcast_transform(self, stamp, x, y, q):
        """Broadcasts a 3D transform with z, roll, and pitch all zero."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, q):
        """Appends the 2D pose to the path and publishes it."""
        self._path.header.stamp = stamp
        self._path.header.frame_id = "odom"

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm above ground level

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        # # Trim the path if it exceeds the maximum length
        # if len(self._path.poses) > self.MAX_PATH_LENGTH:
        #     self._path.poses.pop(0)

        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()