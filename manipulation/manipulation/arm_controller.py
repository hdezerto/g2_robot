import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout

import time

# ------------------- HUGO: TO FINISH -------------------
"""
ADD:
- Add a subscriber to the topic that received the desired pick/place position in the correct frame (still to see)
- Compute the inverse kinematics to get the desired joint angles
- Publish the joint angles to the multi_servo_cmd_sub topic
- Track the position of the end effector with arm camera to check if the pick/place was successful
- Publish the result of the pick/place to the topic that the subscriber is listening for feedback
"""

"""
Check github for more info: https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/README.md
"""


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('move_servos_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        
        # Invokes the timer_callback every 3 seconds (message to the servos)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.count_message = 0

    def timer_callback(self):

        # Angles for each of the 6 servos and the respective movement time in the format:
        # [servo1, servo2, servo3, servo4, servo5, servo6, time1, time2, time3, time4, time5, time6]
        # Angles in centidegrees and time in milliseconds
        # Give -1 as angle to not move a servo.  If a position is given outside of a servo angular range, the servo will move to its limit
        servos_angles_times = [[3000,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000],
                    [3000,12000,8000,20000,6700,12000,  2000,2000,2000,2000,2000,2000],
                    [12000,12000,8000,20000,6700,12000,  2000,2000,2000,2000,2000,2000],
                    [12000,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000]]

        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        msg.data = servos_angles_times[self.count_message]
        self.publisher_.publish(msg)

        self.count_message += 1
        if self.count_message == 4:
            self.count_message = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout

import time

# ------------------- HUGO: TO FINISH -------------------
"""
ADD:
- Add a subscriber to the topic that received the desired pick/place position in the correct frame (still to see)
- Compute the inverse kinematics to get the desired joint angles
- Publish the joint angles to the multi_servo_cmd_sub topic
- Track the position of the end effector with arm camera to check if the pick/place was successful
- Publish the result of the pick/place to the topic that the subscriber is listening for feedback
"""

"""
Check github for more info: https://github.com/migsdigs/Hiwonder_xArm_ESP32/blob/main/README.md
"""


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('move_servos_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        
        # Invokes the timer_callback every 3 seconds (message to the servos)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.count_message = 0

    def timer_callback(self):
        self.get_logger().info('Timer callback triggered.')

        # Angles for each of the 6 servos and the respective movement time in the format:
        # [servo1, servo2, servo3, servo4, servo5, servo6, time1, time2, time3, time4, time5, time6]
        # Angles in centidegrees and time in milliseconds
        # Give -1 as angle to not move a servo.  If a position is given outside of a servo angular range, the servo will move to its limit
        servos_angles_times = [[3000,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000],
                    [3000,12000,8000,20000,6700,12000,  2000,2000,2000,2000,2000,2000],
                    [12000,12000,8000,20000,6700,12000,  2000,2000,2000,2000,2000,2000],
                    [12000,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000]]

        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        msg.data = servos_angles_times[self.count_message]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published message: {msg.data}')

        self.count_message += 1
        if self.count_message == 4:
            self.count_message = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.get_logger().info('MinimalPublisher node has been created.')

    try:
        rclpy.spin(minimal_publisher)
    except Exception as e:
        minimal_publisher.get_logger().error(f'An error occurred: {e}')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()