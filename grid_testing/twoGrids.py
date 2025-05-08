import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import random
import time

# Robot & grid parameters
robot_length = 0.25  # m
robot_width  = 0.20  # m
grid_size    = 1.0   # m

# Initialize robot state
x_pos       = random.uniform(0, grid_size)
y_pos       = random.uniform(0, grid_size)
orientation = random.uniform(0, 2*np.pi)

def move_robot():
    global x_pos, y_pos, orientation
    move    = random.choice(['forward','backward','left','right','rotate'])
    speed   = 0.05
    rot_spd = np.pi/20

    if move=='forward':
        x_pos += speed*np.cos(orientation)
        y_pos += speed*np.sin(orientation)
    elif move=='backward':
        x_pos -= speed*np.cos(orientation)
        y_pos -= speed*np.sin(orientation)
    elif move=='left':   # strafe left
        x_pos -= speed*np.sin(orientation)
        y_pos += speed*np.cos(orientation)
    elif move=='right':  # strafe right
        x_pos += speed*np.sin(orientation)
        y_pos -= speed*np.cos(orientation)
    else:                # rotate
        orientation += rot_spd

    x_pos = np.clip(x_pos, 0, grid_size)
    y_pos = np.clip(y_pos, 0, grid_size)
    orientation %= 2*np.pi

def sensor_hit(ox, oy, angle):
    """
    Returns (d, hx, hy) where d is distance to border hit,
    and (hx, hy) is the global hit point.
    """
    ca, sa = np.cos(angle), np.sin(angle)
    dists = []
    # x = 0 or x = grid_size
    if ca != 0:
        for xw in (0, grid_size):
            t = (xw - ox)/ca
            if t>0: dists.append((t, ox + t*ca, oy + t*sa))
    # y = 0 or y = grid_size
    if sa != 0:
        for yw in (0, grid_size):
            t = (yw - oy)/sa
            if t>0: dists.append((t, ox + t*ca, oy + t*sa))
    return min(dists, key=lambda x: x[0])  # smallest positive t

# set up figure + subplots
plt.ion()
fig, (ax1, ax2) = plt.subplots(1,2, figsize=(10,5))

for _ in range(200):
    move_robot()
    ax1.cla(); ax2.cla()

    # --- 1) global map on ax1 ---
    ax1.set_title("Global Map")
    ax1.set_xlim(0, grid_size); ax1.set_ylim(0, grid_size)
    ax1.set_xticks(np.arange(0,1.01,0.1)); ax1.set_yticks(np.arange(0,1.01,0.1))
    ax1.grid(True)

    # robot body
    rect = patches.Rectangle(
        (x_pos - robot_length/2, y_pos - robot_width/2),
        robot_length, robot_width,
        facecolor='blue', alpha=0.6
    )
    t = transforms.Affine2D().rotate_deg_around(
        x_pos, y_pos, np.degrees(orientation)
    ) + ax1.transData
    rect.set_transform(t); ax1.add_patch(rect)

    # heading arrow
    arrow_len = max(robot_length, robot_width)/2
    ax1.arrow(
        x_pos, y_pos,
        arrow_len*np.cos(orientation),
        arrow_len*np.sin(orientation),
        head_width=0.03, head_length=0.03, fc='red', ec='red'
    )

    # sensors: front, right, back, left
    sensors = [
      ( robot_length/2,  0.0,          orientation    ),  # front
      ( 0.0,            -robot_width/2, orientation-np.pi/2 ),
      (-robot_length/2,  0.0,          orientation+np.pi ),
      ( 0.0,             robot_width/2, orientation+np.pi/2 )
    ]
    hits = []
    for dx_b, dy_b, ang in sensors:
        # sensor origin in global coords
        ox = x_pos + dx_b*np.cos(orientation) - dy_b*np.sin(orientation)
        oy = y_pos + dx_b*np.sin(orientation) + dy_b*np.cos(orientation)
        d, hx, hy = sensor_hit(ox, oy, ang)
        hits.append((ox,oy,ang,d,hx,hy))
        # draw ray
        ax1.plot([ox, hx], [oy, hy], '--g')
        ax1.text(ox + 0.02*np.cos(ang), oy + 0.02*np.sin(ang),
                 f"{d:.2f}", color='green', fontsize=8)

    # --- 2) robot‑centric view on ax2 ---
    ax2.set_title("Robot‑Centric View")
    L = grid_size  # show full range
    ax2.set_xlim(-L, L); ax2.set_ylim(-L, L)
    ax2.set_aspect('equal', 'box')
    ax2.grid(True)

    # draw robot at origin
    rb = patches.Rectangle(
        (-robot_length/2, -robot_width/2),
        robot_length, robot_width,
        facecolor='blue', alpha=0.6
    )
    ax2.add_patch(rb)
    ax2.arrow(0,0, arrow_len, 0, head_width=0.03, head_length=0.03,
              fc='red', ec='red')

    # plot each hit in robot coords
    for ox,oy,ang,d,hx,hy in hits:
        # compute local coords: rotate global delta by –orientation
        dx, dy = hx - x_pos, hy - y_pos
        x_l =  dx*np.cos(orientation) + dy*np.sin(orientation)
        y_l = -dx*np.sin(orientation) + dy*np.cos(orientation)
        # draw local ray & point
        ax2.plot([0, x_l], [0, y_l], '--g')
        ax2.plot(x_l, y_l, 'go')
        ax2.text(x_l*1.05, y_l*1.05, f"{d:.2f}", color='green', fontsize=8,
                 ha='center', va='center')

    plt.pause(0.1)
    time.sleep(0.1)

plt.ioff()
plt.show()
