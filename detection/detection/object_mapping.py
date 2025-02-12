import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros
import geometry_msgs.msg
import numpy as np

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_file = "object_map.txt"
        self.timer = self.create_timer(1.0, self.check_transforms)
        self.detected_objects = {"detected_sphere": 2, "detected_square": 1, "detected_fluffy": 3, "detected_box": "B"}
        self.object_detections = {}
        self.detection_threshold = 5
        self.area_threshold = 0.15  # 15 cm
    
    def check_transforms(self):
        for frame_id, label in self.detected_objects.items():
            try:
                trans = self.tf_buffer.lookup_transform("map", frame_id, rclpy.time.Time())
                x, y = trans.transform.translation.x, trans.transform.translation.y
                
                if self.is_already_logged(label, x, y):
                    continue
                
                if frame_id not in self.object_detections:
                    self.object_detections[frame_id] = []
                
                self.object_detections[frame_id].append((x, y))
                
                if len(self.object_detections[frame_id]) >= self.detection_threshold:
                    mean_x = np.mean([p[0] for p in self.object_detections[frame_id]])
                    mean_y = np.mean([p[1] for p in self.object_detections[frame_id]])
                    angle = 0 if label in [1, 2, 3] else 90  # Boxes (label B) get an angle of 90 degrees
                    self.write_to_map_file(label, mean_x, mean_y, angle)
                    self.object_detections[frame_id] = []  # Reset detections after logging
            except tf2_ros.LookupException:
                self.get_logger().warn(f"No transform available for {frame_id}")
            except tf2_ros.ConnectivityException:
                self.get_logger().error(f"Connectivity issue for {frame_id}")
            except tf2_ros.ExtrapolationException:
                self.get_logger().error(f"Extrapolation error for {frame_id}")
    
    def is_already_logged(self, label, x, y):
        try:
            with open(self.map_file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if len(parts) != 4:
                    continue
                existing_label, existing_x, existing_y, _ = parts
                if int(existing_label) == label:
                    if abs(float(existing_x) - x) < self.area_threshold and abs(float(existing_y) - y) < self.area_threshold:
                        return True
        except FileNotFoundError:
            return False
        return False

    def write_to_map_file(self, label, x, y, angle):
        with open(self.map_file, 'a') as f:
            f.write(f"{label} {x:.2f} {y:.2f} {angle}\n")
        self.get_logger().info(f"Logged: {label} {x:.2f} {y:.2f} {angle}")

def main():
    rclpy.init()
    node = TransformListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
