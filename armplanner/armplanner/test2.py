import numpy as np
from numpy import rad2deg
from scipy.optimize import minimize
import time

# Example robot parameters (replace with your actual dimensions)
L1 = 101/1000  # Length of upper arm
L2 = 94/1000  # Length of forearm
Lw = 169/1000  # Length from wrist to end-effector

# Fixed angle for locked wrist (in radians)
phi = np.radians(90)

def forward_kinematics(joint_angles):
    """
    Compute the forward kinematics for a robot arm with:
      - joint_angles[0]: base rotation (theta1)
      - joint_angles[1]: shoulder (theta2)
      - joint_angles[2]: elbow (theta3)
      
    The wrist is locked (with fixed offset phi).
    
    Returns the (x, y, z) position of the end-effector.
    """
    theta1, theta2, theta3 = np.radians(joint_angles)
    
    # Compute the cumulative angles for each segment
    # x and y depend on base rotation theta1
    # z is vertical (using theta2 and theta3)
    
    # Horizontal projection (r)
    r = (L1 * np.sin(theta2) +
         L2 * np.sin(theta2 + theta3) +
         Lw * np.sin(theta2 + theta3 + phi))
    
    x = r * np.cos(theta1)
    y = r * np.sin(theta1)
    
    # Vertical (z) position
    z = (L1 * np.cos(theta2) +
         L2 * np.cos(theta2 + theta3) +
         Lw * np.cos(theta2 + theta3 + phi))
    
    print(x,y,z)
    theta11 = rad2deg(theta1)
    theta22 = rad2deg(theta2)
    theta33 = rad2deg(theta3)
    print(theta11, theta22, theta33)
    return np.array([x, y, z])

def compute_fk(joint_angles):

    theta1, theta2, theta3 = np.radians(joint_angles)
    #theta1 = np.radians(theta1)  # Base rotation
    #theta2 = np.radians(theta2)  # Shoulder
    #theta3 = np.radians(theta3)  # Elbow

    # Compute wrist position in 3D space
    x1 = L1 * np.cos(theta2)
    z1 = L1 * np.sin(theta2)    

    x2 = x1 + L2 * np.cos(theta2 - theta3)
    z2 = z1 + L2 * np.sin(theta2 - theta3)

    # Wrist endpoint (full length of the wrist)
    L_w = 169/1000  # Wrist length (adjust as needed)
    phi = np.radians(90)  # Wrist angle relative to forearm (0 = aligned)
    x3 = x2 + L_w * np.cos(theta2 - theta3 - phi)
    z3 = z2 + L_w * np.sin(theta2 - theta3 - phi)

    # Apply base rotation (θ1) in XY plane
    x0, y0, z0 = 0, 0, 0  # Base position
    x1_rot = x1 * np.cos(theta1)
    y1_rot = x1 * np.sin(theta1)

    x2_rot = x2 * np.cos(theta1)
    y2_rot = x2 * np.sin(theta1)

    x3_rot = x3 * np.cos(theta1)
    y3_rot = x3 * np.sin(theta1)


    print('yo', x3_rot,y3_rot,z3)
    theta11 = rad2deg(theta1)
    theta22 = rad2deg(theta2)
    theta33 = rad2deg(theta3)
    print(theta11, theta22, theta33)

    return x3_rot, y3_rot, z3

def ik_error(joint_angles, target):
    """
    Error function for inverse kinematics.
    Calculates the Euclidean distance between the 
    forward kinematics result and the target position.
    """
    pos = compute_fk(joint_angles)
    error = np.linalg.norm(pos - target)
    return error

def inverse_kinematics(target, initial_guess=[0, 0, 0]):
    """
    Solve the inverse kinematics using a numerical solver.
    
    Parameters:
      target: (x, y, z) target position for the end-effector.
      initial_guess: starting guess for the joint angles (in degrees).
      
    Returns:
      The joint angles [theta1, theta2, theta3] in degrees that minimize the error.
    """
    result = minimize(ik_error, x0=initial_guess, args=(target,), method='L-BFGS-B', options={'gtol': 1e-6, 'disp': True})
    if result.success:
        return result.x
    else:
        raise ValueError("IK solver did not converge: " + result.message)

# Example usage:
target_position = np.array([0.1, 0.1, 0.1])  # Set your desired target
start = time.time()
try:
    solution_angles = inverse_kinematics(target_position)
    print("Solution joint angles (degrees):", solution_angles)
    print("End-effector position:", compute_fk(solution_angles))
except ValueError as e:
    print(e)
end =  time.time()
total = end - start
print(total*100)

""" 

def main():
    global L1, L2, Lw, phi
    L1 = 101/1000  # Length of first link
    L2 = 94/1000  # Length of second link
    Lw = 0.05  # Wrist offset length
    phi = -90  # Fixed wrist angle

   # Desired end-effector position (x, y, z)
    target_position = np.array([0.1, 0.1, 0.1])

    # Initial guess for joint angles (theta1, theta2, theta3)
    initial_guess = np.array([45.0, 20.0, 40.0])

    # Solve IK
    joint_angles = numerical_ik_3d(target_position, initial_guess)
    print("Joint angles (degrees):", np.degrees(joint_angles))

if __name__ == '__main__':
    main() """