import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants
radius = 33.5 / 1000  # 33.5 mm in meters
link1_length = 44.79 / 1000  # mm to meters
link2_length = 150.08 / 1000  # mm to meters
link2_offset = link2_length * (1 / 3)  # 1/3 in from left

# Anchors
motor1_anchor = np.array([0.0, 0.0])
motor2_anchor = np.array([0.07, -0.03])
link2_right_anchor = np.array([0.10, -0.05])

# Setup figure
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_aspect('equal')
ax.set_xlim(-0.1, 0.15)
ax.set_ylim(-0.1, 0.1)
ax.set_title('2-Link System with Motor Swivels')
ax.axis('off')

# Draw fixed anchors
ax.plot(*motor1_anchor, 'ko')
ax.plot(*motor2_anchor, 'ko')

# Link lines and markers
link1_line, = ax.plot([], [], 'r-', lw=3)
link2_line, = ax.plot([], [], 'b-', lw=3)
tip1_marker, = ax.plot([], [], 'go')  # Motor 1 tip
tip2_marker, = ax.plot([], [], 'go')  # Motor 2 tip

# Sliders
ax_slider1 = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_slider2 = plt.axes([0.2, 0.05, 0.65, 0.03])
slider1 = Slider(ax_slider1, 'Motor 1 Angle', 0, 40, valinit=0)
slider2 = Slider(ax_slider2, 'Motor 2 Angle', 0, 40, valinit=0)

def update(val):
    # Swivel angles
    angle1 = np.radians(135 + slider1.val)
    angle2 = np.radians(-45 - slider2.val)

    # Motor tips
    tip1 = motor1_anchor + radius * np.array([np.cos(angle1), np.sin(angle1)])
    tip2 = motor2_anchor + radius * np.array([np.cos(angle2), np.sin(angle2)])

    # Link 1: from motor1 tip to bar joint
    joint1 = tip1 + link1_length * np.array([np.cos(angle1), np.sin(angle1)])
    link1_line.set_data([tip1[0], joint1[0]], [tip1[1], joint1[1]])

    # Link 2: from fixed right anchor to joint1 (1/3 offset from right)
    joint2 = link2_right_anchor
    joint1_adjusted = joint2 - link2_offset * np.array([1, 0])
    link2_line.set_data([joint1_adjusted[0], joint2[0]], [joint1_adjusted[1], joint2[1]])

    # Update tips
    tip1_marker.set_data([tip1[0]], [tip1[1]])
    tip2_marker.set_data([tip2[0]], [tip2[1]])
    fig.canvas.draw_idle()

slider1.on_changed(update)
slider2.on_changed(update)
update(None)
plt.show()
