#!/usr/bin/env python

import math

import numpy as np
from numpy import pi, cos, sin

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

'''
Code Skeleton:

def pickUp(x,y,z,yaw):
    jointAngles = inverseKinematics(x,y,z,yaw)
    # GIVE ARM ANGLES and PICKUP

def adjustRobot():
    # Gives arm angles so camera is looking straight down
    # 0,0 is at bottom left
    cameraCenter = [X,Y]
    objectCenter = findObjectCenter(): ([X1,Y1])


    

'''





def inverseKinematics(point, R, joint_positions):
    # Starting values
    x, y, z = point[0], point[1], point[2]
    q = np.array(joint_positions)
    eps = 10**-2  # Acceptable error

    for i in range(100):
        X, R1 = forwardKinematics(q) # New position and rotation matrix (R)
        epsX = X - point # Error in position
        
        R_new = np.array(R1) 
        R_old = np.array(R)

        # Take out each column of the R-matrix
        c11,c12,c13= np.column_stack(R_new) 
        c21,c22,c23= np.column_stack(R_old)
        epsR = 1/2*(np.cross(c11,c21)+np.cross(c12,c22)+np.cross(c13,c23)) # Error in rotation
        
        eps = np.concatenate((epsX,epsR))
        print(eps)
        """ 
        J=Jacobian(joint_positions)
        epsQ=np.dot(np.linalg.pinv(J),eps)
        q=q-epsQ """
        
        
        if np.max(np.abs(eps)) < Eps:
            break
    return q

def Jacobian(q):
    # Link lengths (m)
    L1 = 101/1000 
    L2 = 94/1000
    L3 = 169/1000
    
    table_DH = [
        [pi/2,0,0,q[0]],
        [-pi/2,0,0,q[1]],
        [-pi/2,0,0,q[2]],
        [pi/2,0,0,q[3]],
        [pi/2,0,0,q[4]],
        [-pi/2,0,0,q[5]],
    ]       
    #print(tab)
    #input()

    P,R = DH(q)
    

    T=list(map(lambda x:matrix(*x),tab))
    transf = []
    for i in range(len(T)+1):
        result = np.eye(4)  # Start with identity matrix
        for j in range(i):  # Multiply all matrices from T[0] to T[i-1]
            result = np.dot(result, T[j])
        transf.append(result)
    #print(np.size(transf))
    
        
    rot=list(map(lambda x:x[:3,:3],transf))
    transl=list(map(lambda x:x[:3,3],transf))
    J_all=[]
        
    for i in range(0,7):
       
        r_i=rot[i]
        tl_i=transl[i]
        z_i=np.dot(r_i,np.array([0.,0,1]))
        J_orientation_i=z_i
        J_position_i =np.cross(z_i,P-tl_i)
        J_i=np.concatenate((J_position_i,J_orientation_i))
        J_all.append(J_i)
    J=np.stack(J_all).T
    
    return J

def forwardKinematics(q):
    # Link lengths (m)
    L1 = 101/1000 
    L2 = 94/1000
    L3 = 169/1000

    # DH-Table (PLACEHOLDER)
    table_DH = [
        [pi/2,0,0,q[0]],
        [-pi/2,0,0,q[1]],
        [-pi/2,0,0,q[2]],
        [pi/2,0,0,q[3]],
        [pi/2,0,0,q[4]],
        [-pi/2,0,0,q[5]],
    ]

    # Create matrix for each joint
    T=list(map(lambda x:matrix(*x),table_DH))

    # Combine all matrices 
    T_F = np.eye(4)  # Start with identity matrix
    for t in T:  # Multiply all the matrices in the list T
        T_F = np.dot(T_F, t)
        
    R=T_F[:3, :3]
    Q=np.dot(T_F,np.array([0,0,D,1]))
    Q=Q[:3]
    
    Q[2]+= L1

    return Q,R

def matrix(alpha, d, r, theta):
    matrix =np.array(\
        [[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),r*cos(theta)],
         [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),r*sin(alpha)],
         [0,sin(alpha),cos(alpha),d],
         [0,0,0,1]
        ])
    return matrix

def main():
    print('Hi from kinematics.')
    inverseKinematics(np.array([1,1,1]), np.array([[1,0,0], [0,1,0], [0,0,1]]), np.array([1,1,1,1,1,1]))


if __name__ == '__main__':
    main()
