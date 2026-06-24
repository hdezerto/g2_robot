#!/usr/bin/env python

import math
import cv2

import numpy as np
from numpy import pi, cos, sin
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Int64MultiArray, Int16MultiArray,MultiArrayDimension, MultiArrayLayout, String
#from armCameraObj import findObjectCenter
from armplanner.kinematics3 import inverse_kinematics, compute_fk, translate_to_servo


class robotArmNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.start = [7000,13000,14000,10000,16000,8000,2000,2000,2000,2000,2000,2000] # Angle config to default to
        self.tuck =  [7000,13000,14000,10000,16000,8000,2000,2000,2000,2000,2000,2000] # Angle config to tuck the arm in
        self.pose = 0
        self.buffer = Buffer()
        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        self.camera_publisher = self.create_publisher(String, '/arm_camera_controller',10)
        self.feedback_publisher = self.create_publisher(String, '/arm_controller_feedback', 10) 
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.subscription = self.create_subscription(String, '/arm_controller', self.listener_callback,qos_profile)
        self.subscription2 = self.create_subscription(PointStamped, 'object_position', self.listener_callback2, qos_profile)        
        self.position_reached = True

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        if msg.data == 'PICK':
            self.camera_publisher.publish(msg)
            temppos = PointStamped()
            temppos.point.x = 0.2
            temppos.point.y = 0.0
            temppos.point.z = -0.13
            self.listener_callback2(temppos)
        elif msg.data == 'DROP':
            self.control_servos([], 'DROP')
        else:
            print('Message not correct format')

    def front_camera_pick():
        pass
        
    def listener_callback2(self,msg):
        self.pose = msg
        angles, servo_angles = inverse_kinematics(msg)
        print("Solution joint angles (degrees):", int(angles[0]),int(angles[1]),int(angles[2]),int(angles[3]))
        print("Solution servo-joint angles (degrees):", servo_angles)
        print("End-effector position:", compute_fk(angles))
        self.control_servos3(servo_angles, 'PICK')


        # placeholder location
        """ servos_angles_times1 = [[3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000], 
                                [3000,12000,9000,angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000] 

        ] """

    def control_servos3(self, calculated_angles, command):
        self.get_logger().info('Control servos triggered')

        # move into preset angles
        #calculated_angles = [int(angle1 * 100) for angle1 in calculated_angles]
        angles = translate_to_servo([0,70,65,95]) # arm camera angles

        valid_angles = [True, True, True, True]
        if angles[2] < 30:
            valid_angles[2] = False
        if angles[1] < 0:
            valid_angles[1] = False
        if angles[0] > 150:
             valid_angles[0] = False
        if angles[3] < 0:
            valid_angles[3] = False

        angles = [int(angle1 * 100) for angle1 in angles]
        servos_angles_times1 = [[3000,12000,12000,12000,12000,12000, 2000,2000,2000,2000,2000,2000],
                                [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]]
        
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        
        for angles in servos_angles_times1:
            self.get_logger().info(f'Angles: {angles}')
            msg.data = angles
            self.servos_publisher.publish(msg)
            self.get_logger().info(f'Published message: {msg.data}')
            time.sleep(3)
        

    def control_servos2(self, calculated_angles, command):
        self.get_logger().info('Control servos triggered')

        # move into preset angles
        calculated_angles = [int(angle1 * 100) for angle1 in calculated_angles]
        servos_angles_times1 = [[3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000],
                                [3000,12000,3000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]]
        
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        
        for angles in servos_angles_times1:
            self.get_logger().info(f'Angles: {angles}')
            msg.data = angles
            self.servos_publisher.publish(msg)
            self.get_logger().info(f'Published message: {msg.data}')
            time.sleep(3)


    def control_servos(self, angles, command):
        self.get_logger().info('Control servos triggered.')
        """ 
        index0 - gripper
        index1 - rotate wrist around axis
        index2 - rotate wrist up and down
        index3 - rotate forearm
        index4 - rotate "upper" arm
        index5 - rotate base
        index6-11 - time to move to angle
          
            """
        # row1 - tuck position
        # row2 - pick position
        if command == 'PICK':

            valid_angles = [True, True, True, True]
            if angles[2] < 30:
                valid_angles[2] = False
            if angles[1] < 0:
                valid_angles[1] = False
            if angles[0] > 150:
                valid_angles[0] = False
            if angles[3] < 0:
                valid_angles[3] = False
            angles = [int(angle * 100) for angle in angles]
            #servos_angles_times1 = [[3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]]
            """ servos_angles_times1 = [[3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000], 
                                    [3000,12000,9000,angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000] 

            ] """

            """ 
            servolimits:
            base: 150 
            shoulder: 30
            elbow: 0 (i think)

            """
            # PICKUP SEQUENCE
            upright_open = [3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
            grab_pos = [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
            grab = [11000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
            upright_hold = [11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
            servos_angles_times1 = [upright_open, grab_pos, grab, upright_hold]       
            """ servos_angles_times1 = [[3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000], 
                                    [3000,12000,3000,angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000],
                                    [11000,12000,3000,angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                                    [11000,12000,3000,angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]   

                ] """
        elif command == 'DROP':
            servos_angles_times1 = [[11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000],
                                    [11000,12000,3000,12116,6683,11999,2000,2000,2000,2000,2000,2000],
                                    [3000,12000,3000,12116,6683,11999,2000,2000,2000,2000,2000,2000]
                                    ]
            valid_angles = [True, True, True, True]
            
        else:
            print('Invalid command')
            return
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        if all(valid_angles):        
            for angles in servos_angles_times1:
                msg.data = angles
                print(msg.data)
                self.servos_publisher.publish(msg)
                #self.get_logger().info(f'Published message: {msg.data}')
                time.sleep(3)

            # Publish SUCCESS message to /arm_controller_feedback
            feedback_msg = String()
            feedback_msg.data = 'SUCCESS'
            self.feedback_publisher.publish(feedback_msg)
            self.get_logger().info('Published feedback message: SUCCESS')
        else:
            # Publish FAILURE message to /arm_controller_feedback
            feedback_msg = String()
            feedback_msg.data = 'FAILURE'
            self.feedback_publisher.publish(feedback_msg)
            self.get_logger().info('Published feedback message: FAILURE')
            print('Invalid angles')
                        
                        
                        
                    


    def findObjCenter(self):
        raise NotImplementedError(
            "Use the arm_camera_detection node for camera-based object localization."
        )


    def trajectoryCalc(self,x,y):
        print('Calculating trajectory')
        # Creates iterative path for robot movement

    def cartestianToConfig(self):
        print('Configuring coordinates')
        # Converting from cartesian coordinates to configuration space
        L1 = 0 # Distance from arm base to arm camera X (default config)
        L2 = 0 # Distance from arm base to arm camera Y (default config)
        C = 0  # Center of camera coordinate 
        x_cam, y_cam = self.findObjCenter()
        pos_x = x_cam - C 
        pos_y = y_cam - C + L2
        self.trajectoryCalc(pos_x,pos_y)


def main():
    # do arm shit
    rclpy.init()
    node = robotArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
