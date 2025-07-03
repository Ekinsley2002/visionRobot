import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ─────────────── CONSTANTS (metres) ───────────────
radiusUpper   = 38.0  / 1000
radiusLower   = 30.0  / 1000
link1_length  = 31 / 1000        # red
link2_length  = 104 / 1000       # blue
bar_length    = 124  / 1000       # orange
attach_dist   = 48.0  / 1000        # green attaches 48 mm from orange-top
joint_ratio   = 0.24                # blue joint fraction

# ─────────────── FIXED ANCHORS ────────────────────
motor1_anchor      = np.array([0.0, 0.0])                    # upper motor
motor2_anchor      = motor1_anchor + np.array([0.0386, -0.0268])  # lower motor
link2_right_anchor = motor2_anchor + np.array([4.0, 11.3]) / 1000

# ─────────────── FIGURE SET-UP ────────────────────
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.25)
ax.set_aspect('equal')
ax.set_xlim(-0.10, 0.22)           # a bit wider to fit new spacing
ax.set_ylim(-0.22, 0.15)
ax.set_title('2-Link System with Motor Swivels')
ax.axis('off')

ax.plot(*motor1_anchor, 'ko')
ax.plot(*motor2_anchor, 'ko')
ax.plot(*link2_right_anchor, 'mo')

link1_line, = ax.plot([], [], 'r-',  lw=3)          # red
link2_line, = ax.plot([], [], 'b-',  lw=3)          # blue
link3_line, = ax.plot([], [], 'g-',  lw=3)          # green
link4_line, = ax.plot([], [], color='orange', lw=3) # orange
tip1_marker, = ax.plot([], [], 'go')
tip2_marker, = ax.plot([], [], 'go')

ax_slider1 = plt.axes([0.20, 0.10, 0.65, 0.03])
ax_slider2 = plt.axes([0.20, 0.05, 0.65, 0.03])
slider1 = Slider(ax_slider1, 'Motor 1 Angle', 0, 40, valinit=0)
slider2 = Slider(ax_slider2, 'Motor 2 Angle', 0, 40, valinit=0)

# ─────────────── UPDATE FUNCTION ─────────────────
def update(_):
    th1 = np.radians(135 + slider1.val)
    th2 = np.radians(-45 - slider2.val)
    tip1 = motor1_anchor + radiusUpper * np.array([np.cos(th1), np.sin(th1)])
    tip2 = motor2_anchor + radiusLower * np.array([np.cos(th2), np.sin(th2)])

    # ---- red & blue linkage ---------------------------------------
    r, d = link1_length, link2_length * (1 - joint_ratio)
    D     = link2_right_anchor - tip1
    dist  = np.linalg.norm(D)
    if dist > r + d or dist < abs(r - d) or (dist == 0 and r == d):
        return

    a = (r**2 - d**2 + dist**2) / (2 * dist)
    h = np.sqrt(abs(r**2 - a**2))
    midpoint = tip1 + a * D / dist
    perp     = np.array([-D[1], D[0]]) / dist
    joint1   = midpoint - h * perp                      # red/blue hinge

    full_vec   = (link2_right_anchor - joint1) / (1 - joint_ratio)
    full_vec   = full_vec / np.linalg.norm(full_vec) * link2_length
    left_joint = link2_right_anchor - full_vec          # orange top

    # ---- orange bottom via circle–circle intersection -------------
    vec_lo = tip2 - left_joint
    d_lo   = np.linalg.norm(vec_lo)
    R = bar_length
    if d_lo == 0 or d_lo > 2 * R:
        return
    a2   = d_lo / 2
    h2   = np.sqrt(R**2 - a2**2)
    base = left_joint + a2 * vec_lo / d_lo
    perp2 = np.array([-vec_lo[1], vec_lo[0]]) / d_lo
    cand1, cand2 = base + h2 * perp2, base - h2 * perp2
    orange_bottom = cand1 if cand1[1] < cand2[1] else cand2

    # ---- fixed attach point 48 mm down orange ---------------------
    orange_axis = orange_bottom - left_joint
    axis_len    = np.linalg.norm(orange_axis)
    if axis_len == 0 or attach_dist > axis_len:
        return
    orange_dir  = orange_axis / axis_len
    green_start = left_joint + orange_dir * attach_dist
    green_end   = tip2

    # ---- draw ------------------------------------------------------
    link1_line.set_data([tip1[0], joint1[0]], [tip1[1], joint1[1]])
    link2_line.set_data([left_joint[0], link2_right_anchor[0]],
                        [left_joint[1], link2_right_anchor[1]])
    link3_line.set_data([green_start[0], green_end[0]],
                        [green_start[1], green_end[1]])
    link4_line.set_data([left_joint[0], orange_bottom[0]],
                        [left_joint[1], orange_bottom[1]])
    tip1_marker.set_data([tip1[0]], [tip1[1]])
    tip2_marker.set_data([tip2[0]], [tip2[1]])
    fig.canvas.draw_idle()

slider1.on_changed(update)
slider2.on_changed(update)
update(None)
plt.show()
