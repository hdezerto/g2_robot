import rclpy
from rclpy.node import Node

from std_msgs.msg import String # Previous interface
from std_msgs.msg import Bool, Int32  # Interface messages with collection_controller
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout

import time


"""
To test the node, run the following command in the terminal:

# Send a PICK command (1)
ros2 topic pub -1 /arm_controller std_msgs/msg/Int32 "{data: 1}"

# Send a PLACE command (2)
ros2 topic pub -1 /arm_controller std_msgs/msg/Int32 "{data: 2}"

"""


class ArmController(Node):

    def __init__(self):
        super().__init__('ArmController_node')
        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        self.feedback_publisher = self.create_publisher(Bool, '/arm_controller_feedback', 10)  # Use Bool for feedback
        self.subscription = self.create_subscription(Int32, '/arm_controller', self.listener_callback, 10)


    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        if msg.data == 1:  # 1 for PICK
            self.control_servos(action = 1)
        elif msg.data == 2:  # 2 for DROP
            self.control_servos(action = 2)
        else:
            self.get_logger().info('Invalid command received.')


    def control_servos(self, action):
        self.get_logger().info('Control servos triggered.')

        # Servo angles for pick and place actions
        if action == 1:  # PICK
            servos_angles_times = [[3000, 12000, 12000, 12000, 12000, 12000, 2000, 2000, 2000, 2000, 2000, 2000],
                                   [3000, 16500, 3500, 12800, 4000, 12000, 2000, 2000, 2000, 2000, 2000, 2000],
                                   [12500, 16500, 3500, 12800, 4000, 12000, 2000, 2000, 2000, 2000, 2000, 2000],
                                   [12500, 12000, 12000, 12000, 12000, 12000, 2000, 2000, 2000, 2000, 2000, 2000]]
        else: # DROP (action == 2)
            servos_angles_times = [[12500, 12000, 3500, 12800, 6000, 12000, 2000, 2000, 2000, 2000, 2000, 2000],
                                   [3000, 12000, 3500, 12800, 6000, 12000, 2000, 2000, 2000, 2000, 2000, 2000],
                                   [3000, 12000, 12000, 12000, 12000, 12000, 2000, 2000, 2000, 2000, 2000, 2000]]

        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)

        for angles in servos_angles_times:
            msg.data = angles
            self.servos_publisher.publish(msg)
            self.get_logger().info(f'Published message: {msg.data}')
            time.sleep(3)

        # Publish SUCCESS feedback
        feedback_msg = Bool()
        feedback_msg.data = True  # True for success
        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info('Published feedback message: SUCCESS')




# class ArmController(Node):

#     def __init__(self):
#         super().__init__('ArmController_node')
#         self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10) # 10 as maximum number of messages to be stored in the publisher queue
#         self.feedback_publisher = self.create_publisher(String, '/arm_controller_feedback', 10) 
#         self.subscription = self.create_subscription(
#             String,
#             '/arm_controller',
#             self.listener_callback,
#             10) # 10 as maximum number of messages to be stored in the subscription queue
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info(f'Received message: {msg.data}')
#         if msg.data == 'PICK':
#             self.control_servos(pick=True)
#         elif msg.data == 'PLACE':
#             self.control_servos(pick=False)
#         else:
#             self.get_logger().info('Invalid message received.')

#     def control_servos(self, pick=True):
#         self.get_logger().info('Control servos triggered.')

#         # These angles were tested and found to be the best for the pick and drop actions
#         if pick:
#             servos_angles_times = [[3000,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000],
#                                    [3000,16500,3500,12800,4000,12000,  2000,2000,2000,2000,2000,2000],
#                                    [12500,16500,3500,12800,4000,12000,  2000,2000,2000,2000,2000,2000],
#                                    [12500,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000]]
#         else:
#             servos_angles_times = [[12500,12000,3500,12800,6000,12000,  2000,2000,2000,2000,2000,2000],
#                                    [3000,12000,3500,12800,6000,12000,  2000,2000,2000,2000,2000,2000],
#                                    [3000,12000,12000,12000,12000,12000,  2000,2000,2000,2000,2000,2000]]

#         msg = Int16MultiArray()
#         msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)

#         for angles in servos_angles_times:
#             msg.data = angles
#             self.servos_publisher.publish(msg)
#             self.get_logger().info(f'Published message: {msg.data}')
#             time.sleep(3)

#         # Publish SUCCESS message to /arm_controller_feedback
#         feedback_msg = String()
#         feedback_msg.data = 'SUCCESS'
#         self.feedback_publisher.publish(feedback_msg)
#         self.get_logger().info('Published feedback message: SUCCESS')





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ArmController()
    minimal_publisher.get_logger().info('ArmController node has been created.')

    try:
        rclpy.spin(minimal_publisher)
    except Exception as e:
        minimal_publisher.get_logger().error(f'An error occurred: {e}')
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()


