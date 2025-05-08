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

# State
x_pos       = random.uniform(robot_length/2, grid_size - robot_length/2)
y_pos       = random.uniform(robot_width/2,  grid_size - robot_width/2)
orientation = random.uniform(0, 2*np.pi)

# Accumulate sonar hits
mapped_points = []

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

    # --- clamp so that the entire robot stays inside [0,1] ---
    x_pos = np.clip(x_pos, robot_length/2, grid_size - robot_length/2)
    y_pos = np.clip(y_pos, robot_width/2,  grid_size - robot_width/2)
    orientation %= 2*np.pi

def sensor_hit(ox, oy, angle):
    ca, sa = np.cos(angle), np.sin(angle)
    candidates = []
    # vertical walls
    if ca != 0:
        for xw in (0, grid_size):
            t = (xw - ox)/ca
            if t>0:
                candidates.append((t, ox+t*ca, oy+t*sa))
    # horizontal walls
    if sa != 0:
        for yw in (0, grid_size):
            t = (yw - oy)/sa
            if t>0:
                candidates.append((t, ox+t*ca, oy+t*sa))
    return min(candidates, key=lambda c: c[0])

def convex_hull(points):
    pts = sorted(set(points))
    if len(pts)<3: return pts
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower=[]
    for p in pts:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:
            lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(pts):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

# Prepare 3‑panel figure
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(1,3, figsize=(15,5))

for _ in range(200):
    move_robot()
    ax1.cla(); ax2.cla(); ax3.cla()

    # 1) Global Map
    ax1.set_title("Global Map")
    ax1.set_xlim(0,1); ax1.set_ylim(0,1)
    ax1.set_xticks(np.arange(0,1.01,0.1)); ax1.set_yticks(np.arange(0,1.01,0.1))
    ax1.grid(True)

    # draw robot
    rect=patches.Rectangle(
        (x_pos-robot_length/2, y_pos-robot_width/2),
        robot_length, robot_width, facecolor='blue', alpha=0.6
    )
    t=transforms.Affine2D().rotate_deg_around(
        x_pos, y_pos, np.degrees(orientation)
    )+ax1.transData
    rect.set_transform(t); ax1.add_patch(rect)
    arrow_len=max(robot_length,robot_width)/2
    ax1.arrow(x_pos,y_pos, arrow_len*np.cos(orientation), arrow_len*np.sin(orientation),
              head_width=0.03, head_length=0.03, fc='red', ec='red')

    # four sonars
    sensors = [
      ( robot_length/2,  0.0,          orientation    ),
      ( 0.0,            -robot_width/2, orientation-np.pi/2 ),
      (-robot_length/2,  0.0,          orientation+np.pi ),
      ( 0.0,             robot_width/2, orientation+np.pi/2 )
    ]
    hits=[]
    for dx_b,dy_b,ang in sensors:
        ox = x_pos + dx_b*np.cos(orientation) - dy_b*np.sin(orientation)
        oy = y_pos + dx_b*np.sin(orientation) + dy_b*np.cos(orientation)
        # clamp sensor origin into [0,1]
        ox = np.clip(ox, 0, 1)
        oy = np.clip(oy, 0, 1)
        t,hx,hy = sensor_hit(ox,oy,ang)
        d = t
        hits.append((ox,oy,ang,d,hx,hy))
        mapped_points.append((hx,hy))
        ax1.plot([ox,hx],[oy,hy],'--g')
        ax1.text(ox+0.02*np.cos(ang), oy+0.02*np.sin(ang),
                 f"{d:.2f}", color='green', fontsize=8)

    # 2) Robot‑Centric View
    ax2.set_title("Robot‑Centric View")
    L = 1.0
    ax2.set_xlim(-L,L); ax2.set_ylim(-L,L); ax2.set_aspect('equal')
    ax2.grid(True)
    rb=patches.Rectangle((-robot_length/2,-robot_width/2),
                         robot_length,robot_width,
                         facecolor='blue',alpha=0.6)
    ax2.add_patch(rb)
    ax2.arrow(0,0, arrow_len,0, head_width=0.03,head_length=0.03,fc='red',ec='red')
    for ox,oy,ang,d,hx,hy in hits:
        dx, dy = hx-x_pos, hy-y_pos
        x_l =  dx*np.cos(orientation) + dy*np.sin(orientation)
        y_l = -dx*np.sin(orientation) + dy*np.cos(orientation)
        ax2.plot([0,x_l],[0,y_l],'--g')
        ax2.plot(x_l,y_l,'go')
        ax2.text(x_l*1.05,y_l*1.05, f"{d:.2f}",
                 color='green', fontsize=8, ha='center', va='center')

    # 3) Mapped Border
    ax3.set_title("Mapped Border")
    pad=0.1
    ax3.set_xlim(-pad,1+pad); ax3.set_ylim(-pad,1+pad)
    ax3.set_xticks(np.arange(0,1.01,0.1)); ax3.set_yticks(np.arange(0,1.01,0.1))
    ax3.grid(True)
    hull = convex_hull(mapped_points)
    if len(hull)>1:
        xs,ys = zip(*hull+[hull[0]])
        ax3.plot(xs,ys,'-r',linewidth=2,label="Border hull")
    ax3.plot(x_pos,y_pos,'bo',label="Robot")
    ax3.legend(loc='upper right',fontsize=8)

    plt.pause(0.1)
    time.sleep(0.1)

plt.ioff()
plt.show()

