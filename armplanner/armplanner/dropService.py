import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, Int16MultiArray,MultiArrayDimension, MultiArrayLayout, String
import time
from armplanner.kinematics3 import inverse_kinematics, compute_fk, translate_to_servo
from std_srvs.srv import Trigger


class DropService(Node):
    def __init__(self):
        super().__init__('drop_service')
        self.srv = self.create_service(Trigger, 'drop', self.callback)
        self.get_logger().info('Service ready: /drop')
        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)

    def callback(self, request, response):
        self.get_logger().info('Trigger received! Dropping object')

        try:
            # Example: Run a shell command
            servos_angles_times1 = [[11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000],
                                    [11000,12000,3000,12116,6683,11999,2000,2000,2000,2000,2000,2000],
                                    [3000,12000,3000,12116,6683,11999,2000,2000,2000,2000,2000,2000],
                                    [11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]]
            
            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
            valid_angles = [True, True, True, True]
            if all(valid_angles):        
                for angles in servos_angles_times1:
                    msg.data = angles
                    print(msg.data)
                    self.servos_publisher.publish(msg)
                    #self.get_logger().info(f'Published message: {msg.data}')
                    time.sleep(3)
            
            response.success = True
            response.message = result.stdout.strip()
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DropService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()