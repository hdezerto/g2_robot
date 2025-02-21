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




test2