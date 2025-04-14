import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension



from my_custom_interfaces.srv import Pickup
from std_srvs.srv import Trigger
import time
from armplanner.kinematics3 import translate_to_servo
from armplanner.kinematics3 import inverse_kinematics, compute_fk

def init():
    print("Initializing the environment...")
    self.pickupClient = (Pickup, 'pickup')

    self.servos_publisher = (Int16MultiArray, 'multi_servo_cmd_sub')

    self.dropClient = (Trigger, 'drop')


def pick(self):
    request = Pickup.Request()
    object_type = "Cube"  # Retrieve from topic?
    request.object_type = object_type
    request.color = "Red" # Example color, mainly for testing/debugging
    angles = [12000,10000,18500,2500]
    servos_angles_times1 = [[3000,12000,12000,12000,12000,12000, 2000,2000,2000,2000,2000,2000],
                        [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]]

    msg1 = Int16MultiArray()
    msg1.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)

    for angles in servos_angles_times1:
        self.get_logger().info(f'Angles: {angles}')
        msg1.data = angles
        self.servos_publisher.publish(msg1)
        self.get_logger().info(f'Published message: {msg1.data}')
        time.sleep(3)

    # Call the service asynchronously and get a future
    future = self.client.call_async(request)
    
    # Wait for the response from the service
    rclpy.spin_until_future_complete(self, future)

    # Handle the response
    if future.result() is not None:
        self.get_logger().info(f'Success: {future.result().message}')
        sucesss = future.result().success
        self.publisher(success)
    else:
        self.get_logger().error('Service call failed')


def drop(self):
    request = Trigger.Request()
    future = self.dropClient.call_async(request)
    
    # Wait for the response from the service
    rclpy.spin_until_future_complete(self, future)

    # Handle the response
    if future.result() is not None:
        self.get_logger().info(f'Success: {future.result().message}')
        success = future.result().success
    else:
        self.get_logger().error('Service call failed')


def main(args=None):
    pick()

main()