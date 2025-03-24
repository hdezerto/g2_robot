import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener')
        
        # Create a TF buffer and listener to receive transforms.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_file = "object_map.txt"
        self.timer = self.create_timer(1.0, self.check_transforms)
        
        # The keys here should match the child_frame_id published by your detection node.
        # For example, if your detection node publishes transforms with names 'red_sphere' and 'green_square'
        self.detected_objects = {"red_sphere": 2, "green_square": 1}
        self.object_detections = {}
        self.detection_threshold = 1
        self.area_threshold = 0.15  # 15 cm

    def check_transforms(self):
        # Use the current time for transform lookup.
        current_time = rclpy.time.Time(seconds=0)

        for frame_id, label in self.detected_objects.items():
            try:
                # Lookup the transform from 'map' (target) to the object frame (source).
                # The third argument is the time at which the transform is desired.
                # Using current_time works if the broadcaster is sending continuous updates.
                trans = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame=frame_id,
                    time=current_time,
                    timeout=Duration(seconds=0.5)
                )

                # Extract the (x, y) position from the transform.
                x = trans.transform.translation.x
                y = trans.transform.translation.y

                if self.is_already_logged(label, x, y):
                    continue

                if frame_id not in self.object_detections:
                    self.object_detections[frame_id] = []

                self.object_detections[frame_id].append((x, y))

                # If we have enough detections, average the positions and log to file.
                if len(self.object_detections[frame_id]) >= self.detection_threshold:
                    mean_x = np.mean([p[0] for p in self.object_detections[frame_id]])
                    mean_y = np.mean([p[1] for p in self.object_detections[frame_id]])
                    
                    # For demonstration, we set an angle of 0 or 90 depending on the label.
                    angle = 0 if label in [1, 2, 3] else 90  
                    
                    self.write_to_map_file(label, mean_x, mean_y, angle)
                    self.object_detections[frame_id] = []  # Reset detections after logging

            except LookupException:
                self.get_logger().warn(f"No transform available for {frame_id}")
            except ConnectivityException:
                self.get_logger().error(f"Connectivity issue for {frame_id}")
            except ExtrapolationException:
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
