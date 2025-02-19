import rclpy
from rclpy.node import Node
import rclpy.time
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point  # Import Point from geometry_msgs
import csv
from ament_index_python.packages import get_package_share_directory
import os
from builtin_interfaces.msg import Duration


class WorkspaceVisualizer(Node):
    def __init__(self):
        super().__init__("workspace_visualizer")
        self.marker_publisher = self.create_publisher(
            Marker, "/visualization_marker", 10
        )
        package_share_directory = get_package_share_directory("path_planning")
        self.ws_file = os.path.join(
            package_share_directory, "resource", "workspace_2.tsv"
        )
        self.vertices = self.read_tsv(self.ws_file)
        self.publish_workspace()

    def read_tsv(self, file_path):
        vertices = []
        with open(file_path, mode="r") as file:
            reader = csv.reader(file, delimiter="\t")
            next(reader)  # Skip header row
            for row in reader:
                x, y = float(row[0]) / 100, float(row[1]) / 100
                vertices.append((x, y))
        return vertices

    def publish_workspace(self):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.x = 0.05  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0  # Green color
        marker.color.b = 0.0
        marker.color.a = 1.0
        print(type(Duration(seconds=0)))
        marker.lifetime = Duration(seconds=10)  # Infinite lifetime

        for vertex in self.vertices:
            point = Point()  # Use Point from geometry_msgs
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = 0.0
            marker.points.append(point)

        # Close the loop by adding the first point again
        if len(self.vertices) > 0:
            point = Point()  # Use Point from geometry_msgs
            point.x = self.vertices[0][0]
            point.y = self.vertices[0][1]
            point.z = 0.0
            marker.points.append(point)

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
