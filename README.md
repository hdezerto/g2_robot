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

On the **remote laptop**, run the following commands:

1. SSH into the robot:
    ```sh
    ssh happy@192.168.128.110
    ```

2. Start the FastDDS discovery server:
    ```sh
    fastdds discovery -i 0 -t 192.168.128.110 -q 42100
    ```

3. In a new terminal, SSH into the robot again:
    ```sh
    ssh happy@192.168.128.110
    ```

4. Source the setup file:
    ```sh
    source ~/dd2419_ws/install/setup.bash
    ```

5. Run the system using the `g2_robot_launch` package:
    ```sh
    ros2 launch g2_robot_launch g2_robot_launch.xml
    ```

6. In a new terminal, run RViz on the laptop (NOT using SSH):
    ```sh
    rviz2
    ```


### Using the Robot (avoid it!)

On the **robot**, run the following commands:

1. Start the FastDDS discovery server:
    ```sh
    fastdds discovery -i 0 -t 192.168.128.110 -q 42100
    ```

2. Run the system using the `g2_robot_launch` package:
    ```sh
    ros2 launch g2_robot_launch g2_robot_launch.xml
    ```



RECORD ROSBAG
ros2 bag record -o bag_hugo --topics Topic /camera/camera/color/camera_info /camera/camera/color/image_raw /camera/camera/color/metadata /camera/camera/depth/camera_info /camera/camera/depth/color/points /camera/camera/depth/metadata /camera/camera/rgbd /cmd_vel /imu/data_raw /imu/mag /imu/temperature /joy /joy/set_feedback /motor/current_duty_cycles /motor/duty_cycles /motor/encoders /parameter_events /path /rosout /scan /tf /tf_static

ros2 bag record -o bag1 -a


PLAY ROSBAG
ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused bag1



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
ros2 launch teleop_twist_joy    

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