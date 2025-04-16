import rclpy
import time
from rclpy.node import Node
from my_custom_interfaces.srv import Pickup  
from std_msgs.msg import Int64MultiArray, Int16MultiArray,MultiArrayDimension, MultiArrayLayout, String
from armplanner.kinematics3 import inverse_kinematics, compute_fk, translate_to_servo

class PickupClient(Node):
    def __init__(self):
        super().__init__('pickup_client')
        
        # Create a service client for the Pickup service
        self.client = self.create_client(Pickup, 'pickup')

        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pickup service not available, waiting...')

    def send_request(self, object_type, color):
        # Create a request object
        request = Pickup.Request()
        request.object_type = object_type
        request.color = color
        angles = translate_to_servo([0,70,65,95]) # arm camera angles
        angles = [int(angle1 * 100) for angle1 in angles]
        servos_angles_times1 = [[3000,12000,12000,12000,12000,12000, 2000,2000,2000,2000,2000,2000],
                            [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]]
    
        msg1 = Int16MultiArray()
        msg1.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
    
        for angles in servos_angles_times1:
            self.get_logger().info(f'Angles: {angles}')
            msg1.data = angles
            self.servos_publisher.publish(msg1)
            self.get_logger().info(f'Published message: {msg1.data}')
            time.sleep(4)

        
        # Call the service asynchronously and get a future
        future = self.client.call_async(request)
        
        # Wait for the response from the service
        rclpy.spin_until_future_complete(self, future)

        # Handle the response
        if future.result() is not None:
            self.get_logger().info(f'Success: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    # Create the client node
    client_node = PickupClient()

    # Example usage: Request pickup of a Green Cube
    client_node.send_request('Plushie', 'Red')

    # Spin the client node to process the request
    rclpy.spin(client_node)

    # Shutdown when done
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()