import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import Pickup

class ArmControlService(Node):
    def __init__(self):
        super().__init__('arm_control_service')
        self.srv = self.create_service(Pickup, 'pickup', self.pickup_callback)

    def pickup_callback(self, request, response):
        self.get_logger().info(f"Request received: Pick up {request.object_names}")

        # Simulate pickup action for all objects
        success = True
        for object_name in request.object_names:
            success &= self.perform_pickup(object_name)

        response.success = success
        if success:
            response.message = f"Successfully picked up {', '.join(request.object_names)}"
        else:
            response.message = f"Failed to pick up some objects"
        return response

    def perform_pickup(self, object_name):
        # Simulate the logic to control the robot arm for each object
        self.get_logger().info(f"Picking up {object_name}")
        return True  # Simulating success

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()