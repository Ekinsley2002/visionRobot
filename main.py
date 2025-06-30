import numpy as np
import matplotlib.pyplot as plt

"""
# --- LINKAGE PARAMETERS (converted to meters) ---
L1 = 44.79 / 1000      # Link 1 (upper motor horn to joint B)
L2 = 119.73 / 1000     # Link 2 (joint B to foot)
L3 = 119.73 / 1000     # Link 3 (joint C to foot)
L4 = 150.08 / 1000     # Link 4 (lower motor horn to joint C)

# Motor positions (in meters)
motor1_pos = np.array([0.0, 0.0])                # Motor 1 anchor point
motor2_pos = np.array([0.0351, -0.0283])         # Motor 2 anchor point

# Servo zero angles (degrees)
servo1_zero = 0      # Motor 1: increasing raises robot
servo2_zero = 180    # Motor 2: decreasing applies backward force

# Input angles (in degrees, relative to servo zero)
input_theta1 = 60    # You can change this for testing
input_theta2 = 140

# Compute actual motor angles (in radians)
theta1 = np.radians(servo1_zero + input_theta1)
theta2 = np.radians(servo2_zero - input_theta2)

# --- FORWARD KINEMATICS FUNCTION ---
def solve_leg_kinematics(theta1, theta2):
    # Point B: end of Link 1
    B = motor1_pos + np.array([
        L1 * np.cos(theta1),
        L1 * np.sin(theta1)
    ])

        # Point C: end of Link 4
    C = motor2_pos + np.array([
        L4 * np.cos(theta2),
    -L4 * np.sin(theta2)  # <- flipped
    ])


    # Vector from B to C
    foot_vector = C - B

    # Normalize to compute foot midpoint (just for debug or center)
    foot_mid = (B + C) / 2

    return motor1_pos, B, C, motor2_pos

# --- GET JOINTS ---
A, B, C, D = solve_leg_kinematics(theta1, theta2)

# --- PLOT ---
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect('equal')
ax.set_title("2D Quadruped Leg Linkage - Step 2")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.grid(True)

# Plot links
ax.plot([A[0], B[0]], [A[1], B[1]], 'r-', label='Link 1')
ax.plot([B[0], C[0]], [B[1], C[1]], 'g-', label='Link 2 + 3 (Foot)')
ax.plot([C[0], D[0]], [C[1], D[1]], 'b-', label='Link 4')
ax.plot([A[0], D[0]], [A[1], D[1]], 'k--', label='Base')

# Plot joints
for pt, name, color in zip([A, B, C, D], ['A', 'B', 'C', 'D'], ['black', 'red', 'green', 'blue']):
    ax.plot(pt[0], pt[1], 'o', color=color)
    ax.text(pt[0] + 0.005, pt[1] + 0.005, name, fontsize=10, color=color)

ax.legend()
plt.show()
"""


"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- Linkage Parameters (meters) ---
L1 = 44.79 / 1000
L2 = 119.73 / 1000
L3 = 119.73 / 1000
L4 = 150.08 / 1000

motor1_pos = np.array([0.0, 0.0])
motor2_pos = np.array([0.0351, -0.0283])

servo1_zero = 0     # Motor 1 zero angle (degrees)
servo2_zero = 180   # Motor 2 zero angle (degrees)

# --- Kinematic Solver ---
def solve_leg_kinematics(theta1_deg, theta2_deg):
    # Convert motor-relative input to absolute angles
    theta1 = np.radians(servo1_zero + theta1_deg)
    theta2 = np.radians(servo2_zero - theta2_deg)

    # Joint B
    B = motor1_pos + np.array([
        L1 * np.cos(theta1),
        L1 * np.sin(theta1)
    ])

    # Joint C (flipped Y direction)
    C = motor2_pos + np.array([
        L4 * np.cos(theta2),
       -L4 * np.sin(theta2)
    ])

    return motor1_pos, B, C, motor2_pos

# --- Plot Setup ---
fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(bottom=0.25)
ax.set_aspect('equal')
ax.set_title("2D Quadruped Leg Interactive Simulation")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_xlim(-0.05, 0.2)
ax.set_ylim(-0.15, 0.1)
ax.grid(True)

# Initialize lines and points
(link1_line,) = ax.plot([], [], 'r-', label='Link 1')
(foot_line,) = ax.plot([], [], 'g-', label='Foot (Link 2+3)')
(link4_line,) = ax.plot([], [], 'b-', label='Link 4')
(base_line,) = ax.plot([], [], 'k--', label='Base')

# Marker points
(joint_dots,) = ax.plot([], [], 'ko', markersize=5)

# --- Slider Axes ---
ax_theta1 = plt.axes([0.15, 0.15, 0.65, 0.03])
ax_theta2 = plt.axes([0.15, 0.1, 0.65, 0.03])

slider_theta1 = Slider(ax_theta1, 'Motor 1 Angle', 0, 140, valinit=60)
slider_theta2 = Slider(ax_theta2, 'Motor 2 Angle', 0, 140, valinit=80)

# --- Update Function ---
def update(val):
    theta1 = slider_theta1.val
    theta2 = slider_theta2.val

    A, B, C, D = solve_leg_kinematics(theta1, theta2)

    # Update lines
    link1_line.set_data([A[0], B[0]], [A[1], B[1]])
    foot_line.set_data([B[0], C[0]], [B[1], C[1]])
    link4_line.set_data([D[0], C[0]], [D[1], C[1]])
    base_line.set_data([A[0], D[0]], [A[1], D[1]])

    # Update joint points
    joint_x = [A[0], B[0], C[0], D[0]]
    joint_y = [A[1], B[1], C[1], D[1]]
    joint_dots.set_data(joint_x, joint_y)

    fig.canvas.draw_idle()

# Attach update function
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)

# Initial draw
update(None)
ax.legend()
plt.show()
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ------------------------------
# Constants and Link Dimensions
# ------------------------------
L1 = 44.79 / 1000     # Link 1 (m)
L2 = 119.73 / 1000    # Link 2 (m)
L3 = 119.73 / 1000    # Link 3 (m)
L4 = 150.08 / 1000    # Link 4 (m)
R_motor = 33.5 / 1000 # Radius of circular motor linkages (m)

# Motor anchor positions
motor1_pos = np.array([0.0, 0.0])
motor2_pos = np.array([35.1 / 1000, -28.3 / 1000])

# Zero angles (degrees)
motor1_zero = 0
motor2_zero = 180

# Angle limits
motor1_min, motor1_max = 0, 140
motor2_min, motor2_max = 40, 180

# ------------------------------
# Plot Setup
# ------------------------------
fig, ax = plt.subplots(figsize=(8, 6))
plt.subplots_adjust(left=0.1, bottom=0.25)

link_lines = {
    'motor1_arm': ax.plot([], [], 'orange', label='Motor 1 Arm')[0],
    'link1': ax.plot([], [], 'red', label='Link 1')[0],
    'foot': ax.plot([], [], 'green', label='Foot (Link 2+3)')[0],
    'link4': ax.plot([], [], 'blue', label='Link 4')[0],
    'motor2_arm': ax.plot([], [], 'purple', label='Motor 2 Arm')[0],
    'base': ax.plot([], [], 'k--', label='Base')[0],
}

points_plot, = ax.plot([], [], 'ko')  # joints

ax.set_xlim(-0.08, 0.2)
ax.set_ylim(-0.16, 0.1)
ax.set_aspect('equal')
ax.set_title("2D Quadruped Leg Interactive Simulation")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.grid(True)
ax.legend()

# ------------------------------
# Slider Setup
# ------------------------------
ax_slider1 = plt.axes([0.1, 0.15, 0.8, 0.03])
ax_slider2 = plt.axes([0.1, 0.1, 0.8, 0.03])
slider1 = Slider(ax_slider1, 'Motor 1 Angle', motor1_min, motor1_max, valinit=70)
slider2 = Slider(ax_slider2, 'Motor 2 Angle', motor2_min, motor2_max, valinit=110)

# ------------------------------
# Forward Kinematics Update
# ------------------------------
def update(val):
    theta1_deg = slider1.val
    theta2_deg = slider2.val

    theta1_rad = np.radians(motor1_zero + theta1_deg)
    theta2_rad = np.radians(motor2_zero - theta2_deg)

    # Motor 1 arm (rotates from motor1_pos)
    A = motor1_pos + R_motor * np.array([np.cos(theta1_rad), np.sin(theta1_rad)])
    B = A + L1 * np.array([np.cos(theta1_rad), np.sin(theta1_rad)])
    C = B + (L2 + L3) * np.array([np.cos(theta1_rad), np.sin(theta1_rad)])

    # Motor 2 arm
    D = motor2_pos + R_motor * np.array([np.cos(theta2_rad), np.sin(theta2_rad)])

    # Link 4 connects D to C
    link_lines['motor1_arm'].set_data([motor1_pos[0], A[0]], [motor1_pos[1], A[1]])
    link_lines['link1'].set_data([A[0], B[0]], [A[1], B[1]])
    link_lines['foot'].set_data([B[0], C[0]], [B[1], C[1]])
    link_lines['motor2_arm'].set_data([motor2_pos[0], D[0]], [motor2_pos[1], D[1]])
    link_lines['link4'].set_data([D[0], C[0]], [D[1], C[1]])
    link_lines['base'].set_data([motor1_pos[0], motor2_pos[0]], [motor1_pos[1], motor2_pos[1]])

    # Joints
    points = np.array([motor1_pos, A, B, C, D, motor2_pos])
    points_plot.set_data(points[:, 0], points[:, 1])
    fig.canvas.draw_idle()

# Initial draw
update(None)
slider1.on_changed(update)
slider2.on_changed(update)
plt.show()
