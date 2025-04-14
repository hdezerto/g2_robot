import matplotlib.pyplot as plt
import numpy as np

def plot_robot_arm(phi_values):
    fig, axes = plt.subplots(1, len(phi_values), figsize=(12, 4))
    L1, L2 = 2, 1.5  # Length of upper and lower arm
    shoulder = np.array([0, 0])  # Shoulder joint position
    
    elbow = shoulder + np.array([L1, 0])  # Elbow joint
    wrist = elbow + np.array([L2, 0])  # Wrist joint without wrist rotation
    
    for ax, phi in zip(axes, phi_values):
        ax.set_xlim(-1, 4)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f'phi = {phi}°')
        
        # Compute wrist orientation
        phi_rad = np.radians(phi)
        wrist_end = wrist + np.array([np.cos(phi_rad), np.sin(phi_rad)])  # End-effector position
        
        # Plot arm
        ax.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'bo-', lw=3, label="Upper Arm")
        ax.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'go-', lw=3, label="Lower Arm")
        ax.plot([wrist[0], wrist_end[0]], [wrist[1], wrist_end[1]], 'ro-', lw=3, label="Wrist")
        
        # Indicate joints
        ax.plot(shoulder[0], shoulder[1], 'ko', markersize=8)
        ax.plot(elbow[0], elbow[1], 'ko', markersize=8)
        ax.plot(wrist[0], wrist[1], 'ko', markersize=8)
    
    plt.show()

# Example wrist angles phi_values = [0, -90, 90]
plot_robot_arm(phi_values=[0, -90, 90])
