#!/usr/bin/env python

import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import numpy as np
from numpy import pi, cos, sin, arctan, arccos, arcsin, arctan2, sqrt, rad2deg

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped




""" def inverseKinematics(x_target, y_target, z_target):
    # 1. Compute base angle: angle between target projection and X-axis.
    theta1 = np.arctan2(y_target, z_target)
    
    # 2. Project the target onto the plane after base rotation:
    # The effective horizontal distance is d_xy:
    d_zy = np.sqrt(z_target**2 + y_target**2)
    
    # In the vertical (planar) problem, the target is (d_xy, z_target)
    # Distance from shoulder (origin) to target in the plane:
    d = np.sqrt(d_zy**2 + x_target**2)
    
    # Check reachability: target must be reachable (d <= L1 + L2)
    if d > (L2 + L3):
        print("Target is out of reach!")
        return None, None, None
    
    # 3. Compute the elbow angle (θ₃) using the law of cosines.
    # Here, the cosine of the elbow angle (the internal angle between L1 and L2):
    cos_theta3 = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    # θ₃ here is the *external* joint angle. One common convention is:
    theta3 = np.arccos(cos_theta3)  # in radians
    
    # 4. Compute the shoulder angle (θ₂).
    # First, compute the angle (alpha) between the horizontal line (d_xy) and the line from shoulder to target:
    alpha = np.arctan2(x_target, d_zy)
    # Next, compute the angle (beta) at the shoulder using the law of cosines.
    cos_beta = (L2**2 + d**2 - L3**2) / (2 * L2 * d)
    cos_beta = np.clip(cos_beta, -1, 1)
    beta = np.arccos(cos_beta)
    
    # One common solution:
    theta2 = alpha - beta  # (in radians)
    
    return theta1, theta2, theta3 """
def inverseKinematics5(x,y,z):
    R = sqrt(x**2 + y**2 + z**2)
    theta1 = arctan2(y,x)
    theta3 = phi

    len1 = sqrt(L1**2 + L2**2 + 2*L1*L2*cos(180-phi))
    theta4 = arccos((len1**2 + L3**2)/(2*L3*len1))
    theta2 = 

def inverseKinematics6(x,y,z):
    R = sqrt(x**2 + y**2 + z**2)
    theta1 = arctan2(y,x)
    theta2 = phi
    zf = sin(phi) * L1
    xy_fore = cos(phi) * L1 # Point at beginning of forearm (XY-plane)
    xf = xy_fore * cos(theta1) # only x
    yf = xy_fore * sin(theta1) # only y
    print('x and y', xf, yf)
    dxy = sqrt((x - xf)**2 + (y - yf)**2)
    d = sqrt((dxy)**2 + (z - zf)**2)  # Same as R, distance from forearm to target point
    print('distance', d)
    


def inverseKinematics2(x,y,z):
    theta1 = arctan2(y,x) # Base angle
    
    z_fore = sin(phi) * L1  # Point at beginning of forearm
    xy_fore = cos(phi) * L1 # Point at beginning of forearm (XY-plane)
    x_fore = xy_fore * cos(theta1) # only x
    y_fore = xy_fore * sin(theta1) # only y
    print('x and y', x_fore, y_fore)
    dxy = sqrt((x - x_fore)**2 + (y - y_fore)**2)
    d = sqrt((dxy)**2 + (z - z_fore)**2) # Same as R, distance from forearm to target point
    print('distance', d)
    theta3 = arccos((L2**2 + L3**2 - d**2)/(2*L2*L3)) # Wrist angle
    alpha = arccos((L2**2 + d**2 - L3**2)/(2*L2*d))
    theta2 = arctan(z-z_fore/dxy) + alpha

    return theta1, theta2, theta3
    
def inverseKinematics3(x,y,z):
    theta1 = arctan2(y,x)
    r = (x**2 + y**2)
    p = z
    rprim = r - L3 * cos(phi)
    pprim = p - L3 * sin(phi)
    D = sqrt(r**2 + p**2)
    theta3 = arccos((D**2 - L1**2 -L2**2)/(2*L1*L2))
    beta = arctan2(pprim, rprim)
    alpha = arccos((L1**2 + D**2 - L2**2)/(2*L1*D))
    theta2 = beta - alpha


    
    return theta1, theta2, theta3


def inverseKinematics4(x,y,z):
    maxreach = L1 + L2
    p = sqrt(x**2 + y**2)
    R = sqrt(p**2 + z**2)
    base = arctan(y/x)
    M = (R**2 - L1**2 * L2**2)/(2*L1*L2)
    print(maxreach, R, M)
    elbow = arccos(M)
    shoulder = arcsin(z/R) + arctan((L2*sin(elbow))/(L1 + L2*cos(elbow)))
    return base, shoulder,elbow 
    
def transformFrame(xt, yt, zt):
    theta1 = arctan2(yt,xt)
    xtf = L1 * sin(phi) - zt # target x relative forearm as base
    zs = L1 * cos(phi) * cos(theta1)
    ys = L1 * cos(phi) * sin(theta1)
    ztf = xt - zs # target z relative forearm as base
    ytf = yt - ys # target y relative formarm as base
    
   
    return xtf, ytf, ztf

def 


def main():
    global L1, L2, L3, phi
    L1 = 101/1000 # Length of shoulder arm  (m)
    L2 = 94/1000  # Length of forearm       (m)
    L3 = 169/1000 # Length of gripper/wrist (m)
    phi = np.radians(45) # Shoulder angle (locked in place)
    # Target points in base frame:
    xt = 0.1
    yt = 0.1
    zt = 0.1 
    print(sqrt(xt**2+yt**2+zt**2))

    #xtf,ytf,ztf = transformFrame(xt, yt, zt) # Shift perspective/frame to beginning of forearm
    #theta1, theta2, theta3 = inverseKinematics(xtf,ytf,ztf) 
    
    theta1, theta2, theta3 = inverseKinematics2(xt,yt,zt) 
    theta1 = rad2deg(theta1)
    theta2 = rad2deg(theta2)
    theta3 = rad2deg(theta3)
    print(theta1, theta2, theta3)



if __name__ == '__main__':
    main()