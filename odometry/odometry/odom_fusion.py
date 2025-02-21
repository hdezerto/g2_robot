import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from pyfilter.filters import ExtendedKalmanFilter

class OdometryEKF(Node):
    def __init__(self):
        super().__init__("odometry_ekf")
        
        # Transform broadcaster for publishing odometry
        self._tf_broadcaster = TransformBroadcaster(self)
        
        # Path publisher
        self._path_pub = self.create_publisher(Path, "path", 10)
        self._path = Path()
        
        # Subscribe to encoder and IMU topics
        self.create_subscription(Encoders, "/motor/encoders", self.encoder_callback, 10)
        self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 10)
        
        # Robot state: [x, y, yaw, v, yaw_rate]
        self.state = np.zeros(5)
        self.last_time = None
        
        # Process and measurement noise covariances
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])  # Process noise covariance
        self.R = np.diag([0.05, 0.05, 0.05])  # Measurement noise covariance
        
        # Initialize EKF
        self.ekf = ExtendedKalmanFilter(self.state, self.Q, self.R)
        
    def encoder_callback(self, msg: Encoders):
        """Processes encoder data to estimate velocity."""
        dt = 0.05  # 50ms update interval
        wheel_radius = 0.04921
        base = 0.31
        ticks_per_rev = 48 * 64
        
        # Convert encoder ticks to wheel motion
        delta_phi_left = 2 * np.pi * msg.delta_encoder_left / ticks_per_rev
        delta_phi_right = 2 * np.pi * msg.delta_encoder_right / ticks_per_rev
        
        # Compute linear and angular velocity
        v = wheel_radius * (delta_phi_right / dt + delta_phi_left / dt) / 2
        yaw_rate = wheel_radius * (delta_phi_right - delta_phi_left) / base
        
        # Time update (prediction step)
        self.ekf.predict([v, yaw_rate], dt)
        
        # Publish transform and path
        stamp = msg.header.stamp
        self.publish_odometry(stamp)
    
    def imu_callback(self, msg: Imu):
        """Processes IMU data and fuses it with odometry."""
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return
        
        # Convert quaternion to yaw
        quat = msg.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw_rate = msg.angular_velocity.z
        
        # Update the EKF with IMU yaw and yaw rate
        self.ekf.update([yaw, yaw_rate])
    
    def publish_odometry(self, stamp):
        """Publishes the estimated odometry as a TF transform and updates the path."""
        x, y, yaw, _, _ = self.ekf.state
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation = Quaternion(*q)
        self._tf_broadcaster.sendTransform(t)
        
        # Update path
        self._path.header.stamp = stamp
        self._path.header.frame_id = "odom"
        pose = PoseStamped()
        pose.header = self._path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01
        pose.pose.orientation = Quaternion(*q)
        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = OdometryEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
