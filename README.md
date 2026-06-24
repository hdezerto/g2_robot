# G2 Robot - ROS 2 Search and Collection System

ROS 2 workspace for an autonomous mobile manipulator that explores a bounded arena, builds and updates an occupancy grid, detects objects and boxes, plans collision-aware paths, and collects detected objects with an arm.

This was a group project for **DD2419 VT25 Project Course in Robotics and Autonomous Systems**, using the KTH G2 robot platform. The system is organized as a set of ROS 2 packages that coordinate exploration, mapping, perception, localization, motion control, and manipulation.

## System Overview

The high-level behavior is split into two mission phases:

- **Exploration:** the mission controller visits exploration points, receives occupancy-grid updates, tracks detected objects and boxes, and replans with A* if the current path becomes invalid.
- **Collection:** the mission controller moves to re-observation poses near known objects, refines object and box positions with the camera pipeline, calls the arm pickup/drop services, and repeats until no known objects remain.

Core packages:

| Package | Purpose |
| --- | --- |
| `mission_control` | Exploration and collection finite-state machines, A* path planning utilities, occupancy-grid helpers, and map processing. |
| `motion_control` | Odometry, joystick/motor control, stop handling, and path-following motion control. |
| `detection` | Camera/point-cloud based object and box detection. |
| `detection_interfaces` | Custom ROS 2 detection message definitions. |
| `icp` | ICP-based localization correction. |
| `arm` | Simple arm controller node. |
| `armplanner` | Arm camera processing and pickup/drop service nodes. |
| `my_custom_interfaces` | Custom pickup and position services. |
| `g2_robot_launch` | Launch files for hardware, mission, arm, and frame setup. |
| `MAPS` | Saved arena maps used during exploration and collection experiments. |

## Demo

These videos show the final system running the two main mission phases.

### Exploration

<video src="docs/media/exploration-demo.mp4" controls muted width="100%"></video>

[Open exploration demo](docs/media/exploration-demo.mp4)

### Collection

<video src="docs/media/collection-demo.mp4" controls muted width="100%"></video>

[Open collection demo](docs/media/collection-demo.mp4)

## Build

The project is intended to be built inside a ROS 2 workspace with the course robot dependencies available.

```sh
cd ~/dd2419_ws
colcon build --symlink-install
source install/setup.bash
```

## Running

Hardware and mission launch files:

```sh
ros2 launch g2_robot_launch g2_robot_launch_hardware.xml
ros2 launch g2_robot_launch g2_robot_launch_mission.xml
```

Arm-related nodes:

```sh
ros2 launch g2_robot_launch g2_robot_launch_arm.xml
ros2 run arm simple_arm_controller
ros2 run armplanner pickup_service
ros2 run armplanner drop_service
```

Common individual nodes used during development:

```sh
ros2 run icp icp_processor
ros2 run mission_control processor_mapper
ros2 run motion_control motion_control
ros2 run detection detection
ros2 run mission_control exploration_controller
ros2 run mission_control collection_controller
```

When running on the physical robot over the course network, start the Fast DDS discovery server with the robot IP configured for the current session:

```sh
fastdds discovery -i 0 -t <robot-ip> -q 42100
```

## Repository Notes

- Generated `colcon` outputs (`build/`, `install/`, `log/`) are excluded from version control.
- Raw calibration photos, ROS bags, local debug images, and ad-hoc recordings are ignored going forward; the curated demo videos in `docs/media/` are kept.
- The small camera calibration `.npz` files used by the arm camera code are kept in the repository.

## Contributors

- Hugo Dezerto
- Maria Carolina Sebastião
- Jule Haala
- Mattias Ewerby
- Maximos Agis Bolotas
