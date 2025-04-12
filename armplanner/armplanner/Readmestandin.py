"""

Start program:

source:
source ~/Desktop/MATTIAS_WS/install/setup.bash

ros2 topic pub /arm_controller std_msgs/msg/String "{data: 'PICK'}" --once
# NEW TESTING::
ros2 service call /pickup my_custom_interfaces/srv/Pickup "{object_type: 'Cube', color: 'Green'}"


#OLD TESTING:
ros2 topic pub --once /object_position geometry_msgs/PointStamped "{
  'header': {
    'stamp': {
      'sec': 0,
      'nanosec': 0
    },
    'frame_id': 'map'
  },
  'point': {
    'x': 0.2,
    'y': 0.0,
    'z': 0.0
  }
}"


test pick

"""