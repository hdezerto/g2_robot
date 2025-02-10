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

## Running the System

### Using the Remote Laptop

On the **robot**, run the following commands:

1. Start the FastDDS discovery server:
    ```sh
    fastdds discovery -i 0 -t X.Y.Z.W -q 42100
    ```
    where `X.Y.Z.W` is the IP address of the robot.

On the **remote laptop**, run the following commands:

2. SSH into  robotthe:
    ```sh
    ssh -X <username>@<robot_ip>
    ```
    The `-X` flag enables X11 forwarding (necessary for GUI like RViz).

3. Source the setup file:
    ```sh
    source ~/dd2419_ws/install/setup.bash
    ```

4. Run the system using the `g2_robot_launch` package:
    ```sh
    ros2 launch g2_robot_launch g2_robot_launch.xml
    ```

### Using the Robot

1. Comment out the following lines in the bashrc file to disable the FastDDS discovery server:
    ```sh
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER=TCPv4:[X.Y.Z.W]:42100
    export ROS_SUPER_CLIENT=TRUE
    ```

2. Run the system using the `g2_robot_launch` package:
    ```sh
    ros2 launch g2_robot_launch g2_robot_launch.xml
    ```

**Note:**
The FastDDS discovery server makes the nodes communicate through the server using TCP (Transmission Control Protocol), instead of using Simple Discovery Protocol (SDP) over UDP multicast. This isolates each group's robot in the shared KTH-IoT network, preventing cross-talk between groups. Without it, multicast discovery would make laptops detect all robots, causing topic conflicts and data mix-ups. Additionally, with many robots broadcasting discovery messages, network congestion could occur, leading to delays, dropped packets, and instability. The server ensures efficient, reliable communication while reducing network load.





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