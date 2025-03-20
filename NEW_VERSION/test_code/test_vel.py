import rclpy
from rclpy.node import Node
from robp_interfaces.msg import Encoders
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import math
from collections import deque  # Import deque for fixed-size queue


class ComputeVelocity(Node):
    def __init__(self):
        super().__init__('compute_velocity_node')

        # Create a QoS profile with a queue size of 1
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        # Use the QoS profile in the subscription
        self.create_subscription(Encoders, "/motor/encoders", self.encoder_callback, qos_profile)

        # Initialize variables for velocity computation
        self._last_total_ticks_left = None
        self._last_total_ticks_right = None
        self._last_time = None

        # Robot parameters
        self.ticks_per_rev = 48 * 64  # Encoder ticks per wheel revolution
        self.wheel_radius = 0.04921  # Wheel radius in meters
        self.base = 0.31  # Distance between wheels in meters

        # Queue to store the last N linear velocity readings
        self.velocity_queue = deque(maxlen=30)  # Adjust N by changing maxlen

    def encoder_callback(self, msg: Encoders):
        """Callback to compute linear and angular velocities from encoder data."""
        current_time = self.get_clock().now()

        # Use the total number of ticksros2 launch g2_robot_launch g2_robot_launch_hardware.xml
        total_ticks_left = msg.encoder_left
        total_ticks_right = msg.encoder_right

        if self._last_total_ticks_left is None or self._last_time is None:
            # Initialize the last total ticks and time with the first received values
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

        self._last_total_ticks_left = total_ticks_left
        self._last_total_ticks_right = total_ticks_right

        # Compute wheel angular displacements
        delta_phi_left = 2 * math.pi * delta_ticks_left / self.ticks_per_rev
        delta_phi_right = 2 * math.pi * delta_ticks_right / self.ticks_per_rev

        # Compute linear and angular velocities
        v = self.wheel_radius * (delta_phi_right + delta_phi_left) / (2 * dt)
        w = self.wheel_radius * (delta_phi_right - delta_phi_left) / (self.base * dt)

        # Add the linear velocity to the queue
        self.velocity_queue.append(v)

        # Compute the average linear velocity
        avg_v = sum(self.velocity_queue) / len(self.velocity_queue)

        # Log the computed velocities and the average linear velocity
        self.get_logger().info(f"Linear Velocity: {v:.3f} m/s, Angular Velocity: {w:.3f} rad/s")
        self.get_logger().info(f"Average Linear Velocity (last {len(self.velocity_queue)} readings): {avg_v:.3f} m/s")


def main():
    rclpy.init()
    node = ComputeVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()