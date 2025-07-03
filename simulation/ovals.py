import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants
radius = 33.5 / 1000  # 33.5 mm in meters

# Fixed motor pivot positions
motor1_anchor = np.array([0.0, 0.0])
motor2_anchor = np.array([0.07, -0.03])

# Setup figure
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_aspect('equal')
ax.set_xlim(-0.1, 0.15)
ax.set_ylim(-0.1, 0.1)
ax.set_title('Swiveling Points with Motor Anchors')
ax.axis('off')

# Draw fixed pivot markers
ax.plot(motor1_anchor[0], motor1_anchor[1], 'ko')  # Motor 1 anchor (black)
ax.plot(motor2_anchor[0], motor2_anchor[1], 'ko')  # Motor 2 anchor (black)

# Swiveling tip markers
tip1_marker, = ax.plot([], [], 'go')  # Motor 1 tip (green)
tip2_marker, = ax.plot([], [], 'go')  # Motor 2 tip (green)

# Sliders
ax_slider1 = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_slider2 = plt.axes([0.2, 0.05, 0.65, 0.03])
slider1 = Slider(ax_slider1, 'Motor 1 Angle', 0, 40, valinit=0)  # from 135° to 175°
slider2 = Slider(ax_slider2, 'Motor 2 Angle', 0, 40, valinit=0)  # from -45° to -85°

def update(val):
    # Motor 1: from 135° (top-left) to 175° (clockwise)
    angle1_deg = 135 + slider1.val
    angle1 = np.radians(angle1_deg)
    tip1 = motor1_anchor + radius * np.array([np.cos(angle1), np.sin(angle1)])

    # Motor 2: from -45° (bottom-right) to -85° (counter-clockwise)
    angle2_deg = -45 - slider2.val
    angle2 = np.radians(angle2_deg)
    tip2 = motor2_anchor + radius * np.array([np.cos(angle2), np.sin(angle2)])

    # Update tip positions
    tip1_marker.set_data([tip1[0]], [tip1[1]])
    tip2_marker.set_data([tip2[0]], [tip2[1]])
    fig.canvas.draw_idle()

slider1.on_changed(update)
slider2.on_changed(update)
update(None)
plt.show()

