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
    speed = 0.05
    rot_spd = np.pi/20

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

    x_pos = np.clip(x_pos, 0, grid_size)
    y_pos = np.clip(y_pos, 0, grid_size)
    orientation %= 2*np.pi

def plot_robot():
    plt.clf()
    ax = plt.gca()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_xticks(np.arange(0, grid_size+1e-6, 0.1))
    ax.set_yticks(np.arange(0, grid_size+1e-6, 0.1))
    ax.grid(True)

    # Create the robot rectangle (unrotated, centered at (x_pos,y_pos))
    rect = patches.Rectangle(
        (x_pos - robot_length/2, y_pos - robot_width/2),
        robot_length, robot_width,
        facecolor='blue', alpha=0.6
    )
    # Rotate it about its center
    t = transforms.Affine2D().rotate_deg_around(
        x_pos, y_pos, np.degrees(orientation)
    ) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

    # Draw the heading arrow from the exact center
    arrow_len = max(robot_length, robot_width)/2
    ax.arrow(
        x_pos, y_pos,
        arrow_len*np.cos(orientation),
        arrow_len*np.sin(orientation),
        head_width=0.03, head_length=0.03,
        fc='red', ec='red'
    )

    ax.set_title(f"Pos: ({x_pos:.2f}, {y_pos:.2f}) m, θ={np.degrees(orientation):.0f}°")
    plt.pause(0.1)

# Run the simulation
plt.ion()
for _ in range(200):
    move_robot()
    plot_robot()
    time.sleep(0.1)
plt.ioff()
plt.show()
