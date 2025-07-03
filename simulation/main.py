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
