import numpy as np
from numpy import rad2deg
from scipy.optimize import minimize
import time
from geometry_msgs.msg import PoseStamped, PointStamped

# Example robot parameters (replace with your actual dimensions)
L1 = 101/1000  # Length of upper arm
L2 = 94/1000  # Length of forearm
#Lw = 169/1000  # Length from wrist to end-effector
L_w = 142/1000 # Length from wrist to end-effector

# Fixed angle for locked wrist (in radians)
#phi = np.radians(90)

def compute_penalty(joint_angles):
    """ arm upright angles:
        servo1:  26
        servo2:  120
        servo3:  120
        servo4:  120
        servo5:  120
        servo6:  120  """
    
    """ Joint limits for servos not considering workspace:
        servo1:  0-90     (top)
        servo2:  0-240
        servo3:  30-210
        servo4:  30-210
        servo5:  60-180 
        servo6:  0-240    (bottom/base)"""
    joint_limits = [[-90,90],
                    [10,90],
                    [0,90],
                    [0,90],
    ]


    penalty = 0
    for i, angle in enumerate(joint_angles):
        lower, upper = joint_limits[i]
        if angle < lower:
            penalty += (lower - angle) ** 2
        elif angle > upper:
            penalty += (angle - upper) ** 2

    # Experimental penalty for wrist orientation
    """ desired_orientation_deg = np.radians(-90)
    ee_angle = np.radians(joint_angles[1] - joint_angles[2] - joint_angles[3])
    print('ee_angle',ee_angle)
    print(ee_angle - desired_orientation_deg)
    # Penalize deviation from desired orientation
    penalty += (ee_angle - desired_orientation_deg) ** 2 """
    return penalty

def compute_fk(joint_angles):
    z_offset = (2.7+4.4+7+1.8+10.3)/100
    theta1, theta2, theta3, phi = np.radians(joint_angles)
    #theta1 = np.radians(theta1)  # Base rotation
    #theta2 = np.radians(theta2)  # Shoulder
    #theta3 = np.radians(theta3)  # Elbow

    # Compute wrist position in 3D space
    x1 = L1 * np.cos(theta2)
    z1 = L1 * np.sin(theta2)    

    x2 = x1 + L2 * np.cos(theta2 - theta3)
    z2 = z1 + L2 * np.sin(theta2 - theta3)

    # Wrist endpoint (full length of the wrist)
    #L_w = 169/1000  # Wrist length (adjust as needed)
    #phi = np.radians(90)  # Wrist angle relative to forearm (0 = aligned)
    x3 = x2 + L_w * np.cos(theta2 - theta3 - phi)
    z3 = z2 + L_w * np.sin(theta2 - theta3 - phi) #+ z_offset

    # Apply base rotation (θ1) in XY plane
    x0, y0, z0 = 0, 0, 0  # Base position
    x1_rot = x1 * np.cos(theta1)
    y1_rot = x1 * np.sin(theta1)

    x2_rot = x2 * np.cos(theta1)
    y2_rot = x2 * np.sin(theta1)

    x3_rot = x3 * np.cos(theta1)
    y3_rot = x3 * np.sin(theta1)


    #print('yo', x3_rot,y3_rot,z3)
    theta11 = rad2deg(theta1)
    theta22 = rad2deg(theta2)
    theta33 = rad2deg(theta3)
    #print(theta11, theta22, theta33)
    #print(np.rad2deg(theta2-theta3-phi))

    return x3_rot, y3_rot, z3

def ik_error(joint_angles, target):
    """
    Error function for inverse kinematics.
    Calculates the Euclidean distance between the 
    forward kinematics result and the target position.
    """
    pos = compute_fk(joint_angles)
    error = np.linalg.norm(pos - target)
    penalty = compute_penalty(joint_angles)
    return error + penalty

def inverse_kinematics(target, initial_guess=[0, 0, 0, 0]):
    """
    Solve the inverse kinematics using a numerical solver.
    
    Parameters:
      target: (x, y, z) target position for the end-effector.
      initial_guess: starting guess for the joint angles (in degrees).
      
    Returns:
      The joint angles [theta1, theta2, theta3] in degrees that minimize the error.
    """
    
    #targetx = target.pose.position.x
    #targety = target.pose.position.y
    #targetz = target.pose.position.z 
    targetx = target.point.x
    targety = target.point.y
    targetz = target.point.z 

    """ targetrotx = target.pose.orientation.x
    targetrotx = target.pose.orientation.y
    targetrotx = target.pose.orientation.z
    targetrotx = target.pose.orientation.w """

    target = np.array([targetx, targety, targetz])
    print(target)


    result = minimize(ik_error, x0=initial_guess, args=(target,), method='L-BFGS-B', options={'gtol': 1e-6, 'disp': False})
    if result.success:
        #return result.x
        angles = result.x 
        anglesCorrected = translate_to_servo(angles)
        return result.x, anglesCorrected
    else:
        raise ValueError("IK solver did not converge: " + result.message)
    
def translate_to_servo(solution_angles):
    """ arm upright angles:
        servo1:  26     (gripper)
        servo2:  120
        servo3:  120
        servo4:  120    (theta3)
        servo5:  120    (theta2)
        servo6:  120    (theta1)
        """
    theta1, theta2, theta3, phi = solution_angles
    theta1_servo = theta1 + 120
    theta2_servo = theta2 + 30
    theta3_servo = theta3 + 120
    phi_servo = -phi + 120 # not final
    return theta1_servo, theta2_servo, theta3_servo, phi_servo

def main():
    # Example usage:
    #target_position = np.array([-0.1, -0.1, 0.1])  # Set your desired target
    target = PointStamped()

    target.point.x = 0.2
    target.point.y = 0.0
    target.point.z = -0.13
    """ target.pose.position.x = 0.2
    target.pose.position.y = 0.0
    target.pose.position.z = -0.13

    target.pose.orientation.x = 0
    target.pose.orientation.y = 0
    target.pose.orientation.z = 0
    target.pose.orientation.w = 0 """

    start = time.time()
    try:
        solution_angles, servo_angles = inverse_kinematics(target)
        print("Solution joint angles (degrees):", int(solution_angles[0]),int(solution_angles[1]),int(solution_angles[2]),int(solution_angles[3]))
        
        print('Servo angles:', servo_angles)
        print("End-effector position:", compute_fk(solution_angles))
    except ValueError as e:
        print(e)
    end =  time.time()
    total = end - start
    #print(total)

if __name__ == '__main__':
    main()


