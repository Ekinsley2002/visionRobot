import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants (in meters)
radius = 38 / 1000
link1_length = 30.7 / 1000
link2_length = 106.2 / 1000
joint_ratio = 0.32  # red connects 1/3 from left on blue

# Anchors
motor1_anchor = np.array([0.0, 0.0])
motor2_anchor = np.array([0.07, -0.03])
link2_right_anchor = np.array([0.085, 0.005])  # fixed right anchor (purple dot)

# Setup figure
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_aspect('equal')
ax.set_xlim(-0.1, 0.15)
ax.set_ylim(-0.1, 0.1)
ax.set_title('2-Link System with Motor Swivels')
ax.axis('off')

# Draw static anchors
ax.plot(*motor1_anchor, 'ko')
ax.plot(*motor2_anchor, 'ko')
ax.plot(*link2_right_anchor, 'mo')

# Lines and markers
link1_line, = ax.plot([], [], 'r-', lw=3)
link2_line, = ax.plot([], [], 'b-', lw=3)
tip1_marker, = ax.plot([], [], 'go')
tip2_marker, = ax.plot([], [], 'go')

# Sliders
ax_slider1 = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_slider2 = plt.axes([0.2, 0.05, 0.65, 0.03])
slider1 = Slider(ax_slider1, 'Motor 1 Angle', 0, 40, valinit=0)
slider2 = Slider(ax_slider2, 'Motor 2 Angle', 0, 40, valinit=0)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants (in meters)
radiusUpper =  38 / 1000
radiusLower = 29.8 / 1000
link1_length = 44.79 / 1000
link2_length = 150.08 / 1000
joint_ratio = 0.24  # red connects ~1/4 from left on blue

# Anchors
motor1_anchor = np.array([0.0, 0.0])
motor2_anchor = np.array([0.07, -0.03])
link2_right_anchor = motor2_anchor + np.array([4.0, 11.3]) / 1000  # 4mm right, 11.3mm up from motor2

# Setup figure
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_aspect('equal')
ax.set_xlim(-0.1, 0.15)
ax.set_ylim(-0.1, 0.1)
ax.set_title('2-Link System with Motor Swivels')
ax.axis('off')

# Draw static anchors
ax.plot(*motor1_anchor, 'ko')
ax.plot(*motor2_anchor, 'ko')
ax.plot(*link2_right_anchor, 'mo')  # purple anchor

# Lines and markers
link1_line, = ax.plot([], [], 'r-', lw=3)
link2_line, = ax.plot([], [], 'b-', lw=3)
tip1_marker, = ax.plot([], [], 'go')
tip2_marker, = ax.plot([], [], 'go')

# Sliders
ax_slider1 = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_slider2 = plt.axes([0.2, 0.05, 0.65, 0.03])
slider1 = Slider(ax_slider1, 'Motor 1 Angle', 0, 40, valinit=0)
slider2 = Slider(ax_slider2, 'Motor 2 Angle', 0, 40, valinit=0)

def update(val):
    angle1 = np.radians(135 + slider1.val)
    angle2 = np.radians(-45 - slider2.val)

    tip1 = motor1_anchor + radiusUpper * np.array([np.cos(angle1), np.sin(angle1)])
    tip2 = motor2_anchor + radiusLower * np.array([np.cos(angle2), np.sin(angle2)])

    r = link1_length
    d = link2_length * (1 - joint_ratio)
    P1 = tip1
    P2 = link2_right_anchor
    D = P2 - P1
    dist = np.linalg.norm(D)

    if dist > r + d or dist < abs(r - d) or (dist == 0 and r == d):
        return  # No solution or infinite solutions

    a = (r**2 - d**2 + dist**2) / (2 * dist)
    h = np.sqrt(abs(r**2 - a**2))
    midpoint = P1 + a * D / dist
    perp = np.array([-D[1], D[0]]) / dist
    joint1 = midpoint - h * perp  # mirrored solution to keep initial red bar vertical

    full_vec = (link2_right_anchor - joint1) / (1 - joint_ratio)
    full_vec = full_vec / np.linalg.norm(full_vec) * link2_length
    left_joint = link2_right_anchor - full_vec

    link1_line.set_data([tip1[0], joint1[0]], [tip1[1], joint1[1]])
    link2_line.set_data([left_joint[0], link2_right_anchor[0]], [left_joint[1], link2_right_anchor[1]])
    tip1_marker.set_data([tip1[0]], [tip1[1]])
    tip2_marker.set_data([tip2[0]], [tip2[1]])
    fig.canvas.draw_idle()

slider1.on_changed(update)
slider2.on_changed(update)
update(None)
plt.show()



slider1.on_changed(update)
slider2.on_changed(update)
update(None)
plt.show()
