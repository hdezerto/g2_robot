import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from matplotlib.patches import Arc

# Link lengths
L1 = 101/1000  # Upper arm length
L2 = 94/1000  # Forearm length

# Function to compute forward kinematics
def compute_fk(theta1, theta2, theta3):
    theta1 = np.radians(theta1)  # Base rotation
    theta2 = np.radians(theta2)  # Shoulder
    theta3 = np.radians(theta3)  # Elbow

    # Compute wrist position in 3D space
    x1 = L1 * np.cos(theta2)
    z1 = L1 * np.sin(theta2)    

    x2 = x1 + L2 * np.cos(theta2 - theta3)
    z2 = z1 + L2 * np.sin(theta2 - theta3)

    # Wrist endpoint (full length of the wrist)
    #L_w = 169/1000  # Wrist length (adjust as needed)
    L_w = 135/1000 # Wrist length (adjust as needed)
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

    return [(x0, x1_rot, x2_rot, x3_rot), (y0, y1_rot, y2_rot, y3_rot), (z0, z1, z2, z3)]





# Initial joint angles
theta1_init = 0  # Base rotation
theta2_init = 90 # Shoulder angle
theta3_init = 90 # Elbow angle

# Create figure and 3D plot
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(0, 0.3)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.view_init(elev=20, azim=45)

# Plot the arm with actual segments
x_vals, y_vals, z_vals = compute_fk(theta1_init, theta2_init, theta3_init)
arm_line, = ax.plot(x_vals, y_vals, z_vals, 'o-', markersize=8, linewidth=5, color='blue')

wrist_line, = ax.plot([x_vals[2], x_vals[3]], [y_vals[2], y_vals[3]], [z_vals[2], z_vals[3]], 'o-', markersize=8, linewidth=5, color='red')
#wrist_line2, = ax.plot([x_vals[2], x_vals[3]], [y_vals[2], y_vals[3]], [z_vals[2], z_vals[3]], 'o-', markersize=8, linewidth=5, color='red')



# Adjust the main plot to make room for sliders
plt.subplots_adjust(left=0.25, bottom=0.25)

plt.show()

