import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import random
import time

# Robot Dimensions (in meters)
robot_length = 0.25  # 25 cm
robot_width  = 0.20  # 20 cm

# Grid dimensions (1×1 m)
grid_size = 1.0

# Initialize robot state
x_pos       = random.uniform(0, grid_size)
y_pos       = random.uniform(0, grid_size)
orientation = random.uniform(0, 2*np.pi)

def move_robot():
    global x_pos, y_pos, orientation
    move = random.choice(['forward','backward','left','right','rotate'])
    speed = 0.05         # m per step
    rot_spd = np.pi/20   # rad per step

    if move=='forward':
        x_pos += speed*np.cos(orientation)
        y_pos += speed*np.sin(orientation)
    elif move=='backward':
        x_pos -= speed*np.cos(orientation)
        y_pos -= speed*np.sin(orientation)
    elif move=='left':    # strafe left
        x_pos -= speed*np.sin(orientation)
        y_pos += speed*np.cos(orientation)
    elif move=='right':   # strafe right
        x_pos += speed*np.sin(orientation)
        y_pos -= speed*np.cos(orientation)
    else:                 # rotate
        orientation += rot_spd

    # keep inside [0,1]
    x_pos = np.clip(x_pos, 0, grid_size)
    y_pos = np.clip(y_pos, 0, grid_size)
    orientation %= 2*np.pi

def sensor_distance(ox, oy, angle):
    """
    Cast a ray from (ox,oy) in direction 'angle' and return distance to the first
    intersection with the square border [0,grid_size]×[0,grid_size].
    """
    ca, sa = np.cos(angle), np.sin(angle)
    dists = []

    # vertical walls x=0 and x=grid_size
    if ca != 0:
        t = (0 - ox)/ca
        if t>0: dists.append(t)
        t = (grid_size - ox)/ca
        if t>0: dists.append(t)
    # horizontal walls y=0 and y=grid_size
    if sa != 0:
        t = (0 - oy)/sa
        if t>0: dists.append(t)
        t = (grid_size - oy)/sa
        if t>0: dists.append(t)

    return min(dists) if dists else 0

def plot_robot():
    plt.clf()
    ax = plt.gca()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_xticks(np.arange(0, grid_size+1e-6, 0.1))
    ax.set_yticks(np.arange(0, grid_size+1e-6, 0.1))
    ax.grid(True)

    # Draw robot rectangle about its center, then rotate
    rect = patches.Rectangle(
        (x_pos - robot_length/2, y_pos - robot_width/2),
        robot_length, robot_width,
        facecolor='blue', alpha=0.6
    )
    t = transforms.Affine2D().rotate_deg_around(
        x_pos, y_pos, np.degrees(orientation)
    ) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

    # Draw heading arrow from center
    arrow_len = max(robot_length, robot_width)/2
    ax.arrow(
        x_pos, y_pos,
        arrow_len*np.cos(orientation),
        arrow_len*np.sin(orientation),
        head_width=0.03, head_length=0.03,
        fc='red', ec='red'
    )

    # Sensor specs: (name, offset in body frame, cast angle)
    sensors = [
      ("front", ( robot_length/2,  0.0         ), orientation          ),
      ("right", ( 0.0,             -robot_width/2), orientation - np.pi/2 ),
      ("back",  (-robot_length/2,  0.0         ), orientation + np.pi   ),
      ("left",  ( 0.0,              robot_width/2), orientation + np.pi/2 )
    ]

    # For each sensor: compute origin, distance, draw ray & label
    for name, (dx_b, dy_b), angle in sensors:
        # transform body offset into world coords
        ox = x_pos + dx_b*np.cos(orientation) - dy_b*np.sin(orientation)
        oy = y_pos + dx_b*np.sin(orientation) + dy_b*np.cos(orientation)
        d = sensor_distance(ox, oy, angle)

        # draw dashed green ray
        ax.plot(
            [ox, ox + d*np.cos(angle)],
            [oy, oy + d*np.sin(angle)],
            linestyle='--', color='green'
        )
        # label distance just beyond sensor origin
        ax.text(
            ox + 0.02*np.cos(angle),
            oy + 0.02*np.sin(angle),
            f"{d:.2f} m",
            color='green', fontsize=8, ha='center', va='center'
        )

    ax.set_title(
      f"Pos: ({x_pos:.2f}, {y_pos:.2f}) m, θ={np.degrees(orientation):.0f}°"
    )
    plt.pause(0.1)

# === run the sim ===
plt.ion()
for _ in range(200):
    move_robot()
    plot_robot()
    time.sleep(0.1)
plt.ioff()
plt.show()
