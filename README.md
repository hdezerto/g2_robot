# g2_robot

## Overview

EDIT OVERVIEW 

The `g2_robot` repository contains packages and launch files for running various components of the robot, including object detection, motor encoders, camera, and more. This README provides instructions on how to set up and run the different components.

## Setup

1. Navigate to the workspace directory:
    ```sh
    cd ~/dd2419_ws/
    ```

2. Build the workspace:
    ```sh
    # Option 1: Build the workspace by copying files (default method)
    colcon build

    # Option 2: Build the workspace by creating symbolic links (useful for development)
    colcon build --symlink-install
    ```

3. Source the setup file:
    ```sh
    source install/setup.bash
        or
    source install/local_setup.bash
    ```


## Running the System

To run the system, use the `g2_robot_launch` package:

```sh
ros2 launch g2_robot_launch g2_robot_launch.xml
```





----------------------------------------------------------------- PREVIOUS README  -----------------------------------------------------------------


Detection package adds capability for red and blue object detection and localization to the map frame.
To run Detection follow steps:
Run the code TF3:

ros2 launch robp_boot_camp_launch boot_camp_part3_launch.xml

ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

ros2 run odometry odometry

ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/bags/boot_camp
#Instead of rosbag running the robot normaly with the point cloud publishing should work.

For detection:
ros2 run detection detection

RUN MOTOR ENCODERS
ros2 launch robp_launch phidgets_launch.py

RUN CAMERA
ros2 launch robp_launch rs_d435i_launch.py

cd ~dd2419_ws/
colcon build --symlink-install

source install/setup.bash

CREATE PACKAGE
cd ~dd2419_ws
ros2 pkg create pkg_name --build-type ament_python --node-name node_name --dependencies geometry_msgs nav_msgs python3-numpy robp_interfaces rclpy tf_transformations tf2_ros --license MIT

MAKE SURE DEPENDENCIES ARE INSTALLED
cd ~/dd2419_ws
rosdep install -i --from-path src --rosdistro jazzy -y --as-root pip:false

CONNECT LAPTOP TO ROBOT
ssh happy@<IP-Adress>

-------------------------------------------------------

General order of run:

1: Run the frames:launch to orient everything
ros2 launch robp_launch frames_launch.xml

(dont do it) 2: Run Odoemtry related stuff 
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

3: More Odometry
ros2 run odometry odometry

4: Run Motor peripherals and motors
ros2 launch robp_launch phidgets_launch.py

5: Run Camera to initiate pointcloud:
ros2 launch robp_launch rs_d435i_launch.py

6: Run Rviz configuring as needed:
rviz2

7: Run Joy teleop system
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' 

8: Run Joy controller
ros2 run odometry controller

9:Run Detection (once working properly)
ros2 run detection detection

10: Run Lidar (Once working properly)
ros2 launch robp_launch lidar_launch.yaml


11: Arm movement
ros2 launch arm_servos_pubs_subs servo_arm_esp32_launch.py


12: Arm camera (first run command 11)

ros2 launch robp_launch arm_camera_launch.yaml
Add image topic to rviz

13: Record and play rosbag

PLAY ROSBAG
ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/rosbag2_2025_02_05-15_57_16


RECORD ROSBAG
ros2 bag record -a