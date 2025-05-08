import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import matplotlib.transforms as transforms
import numpy as np
import random
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap

# ── CONFIG ───────────────────────────────────────────────
gridSize        = 2.0    # meters
gridDiv         = 50     # divisions per axis
robot_length    = 0.25   # m
robot_width     = 0.20   # m
update_interval = 1     # ms/frame
num_obstacles   = 15     # how many random static blocks

cellSize = gridSize / gridDiv
half_len = robot_length / 2
half_wid = robot_width  / 2

# ── STATE ────────────────────────────────────────────────
x_pos       = random.uniform(half_len, gridSize - half_len)
y_pos       = random.uniform(half_wid, gridSize - half_wid)
orientation = random.uniform(0, 2*np.pi)

all_interior = [(i,j) for i in range(1,gridDiv-1) for j in range(1,gridDiv-1)]
obs_cells    = random.sample(all_interior, num_obstacles)

# ── OCCUPANCY GRID ───────────────────────────────────────
occupancy = np.ones((gridDiv, gridDiv), dtype=int)

# carve out the initial robot footprint
ci0 = int((x_pos-half_len)/cellSize)
ci1 = int((x_pos+half_len)/cellSize)
cj0 = int((y_pos-half_wid)/cellSize)
cj1 = int((y_pos+half_wid)/cellSize)
for ci in range(ci0, ci1+1):
    for cj in range(cj0, cj1+1):
        if 0 <= ci < gridDiv and 0 <= cj < gridDiv:
            occupancy[cj,ci] = 0

# ensure static obstacles stay occupied
for oi,oj in obs_cells:
    occupancy[oj,oi] = 1

# ── HELPERS ──────────────────────────────────────────────
def bresenham(x0, y0, x1, y1):
    pts, dx, dy = [], abs(x1-x0), abs(y1-y0)
    x, y = x0, y0
    sx, sy = (1 if x1>x0 else -1), (1 if y1>y0 else -1)
    err = dx - dy
    while True:
        pts.append((x, y))
        if x==x1 and y==y1: break
        e2 = 2*err
        if e2 > -dy:
            err -= dy; x   += sx
        if e2 < dx:
            err += dx; y   += sy
    return pts

def can_place(nx, ny):
    ci0 = int((nx - half_len) / cellSize)
    ci1 = int((nx + half_len) / cellSize)
    cj0 = int((ny - half_wid) / cellSize)
    cj1 = int((ny + half_wid) / cellSize)
    block = occupancy[cj0:cj1+1, ci0:ci1+1]
    return not np.any(block == 1)

def move_robot():
    global x_pos, y_pos, orientation
    for cmd in random.sample(
        ['forward','backward','left','right','rotate'], 5
    ):
        sp, rp = cellSize*0.5, np.pi/(2*gridDiv)
        dx = dy = da = 0
        if   cmd=='forward':  dx, dy = sp*np.cos(orientation),  sp*np.sin(orientation)
        elif cmd=='backward': dx, dy = -sp*np.cos(orientation), -sp*np.sin(orientation)
        elif cmd=='left':     dx, dy = -sp*np.sin(orientation),  sp*np.cos(orientation)
        elif cmd=='right':    dx, dy =  sp*np.sin(orientation), -sp*np.cos(orientation)
        else:                 da    = rp

        nx = np.clip(x_pos+dx, half_len, gridSize-half_len)
        ny = np.clip(y_pos+dy, half_wid,  gridSize-half_wid)
        no = (orientation + da) % (2*np.pi)

        if can_place(nx, ny):
            x_pos, y_pos, orientation = nx, ny, no
            return

def sensor_hit(ox, oy, ang):
    ca, sa = np.cos(ang), np.sin(ang)
    cands = []
    # map borders
    if ca:
        for xw in (0, gridSize):
            t = (xw-ox)/ca
            if t>0:
                y_hit = oy + t*sa
                if 0<=y_hit<=gridSize:
                    cands.append((t, xw, y_hit))
    if sa:
        for yw in (0, gridSize):
            t = (yw-oy)/sa
            if t>0:
                x_hit = ox + t*ca
                if 0<=x_hit<=gridSize:
                    cands.append((t, x_hit, yw))
    # obstacles
    for oi,oj in obs_cells:
        x0, x1 = oi*cellSize, (oi+1)*cellSize
        y0, y1 = oj*cellSize, (oj+1)*cellSize
        if ca:
            for xw in (x0, x1):
                t = (xw-ox)/ca
                if t>0:
                    y_hit = oy + t*sa
                    if y0<=y_hit<=y1:
                        cands.append((t, xw, y_hit))
        if sa:
            for yw in (y0, y1):
                t = (yw-oy)/sa
                if t>0:
                    x_hit = ox + t*ca
                    if x0<=x_hit<=x1:
                        cands.append((t, x_hit, yw))
    return min(cands, key=lambda v:v[0])

# pre‐compute sensor offsets in robot frame
sensor_defs = [
    ( half_len,  0.0,      0.0      ),
    ( 0.0,      -half_wid, -np.pi/2 ),
    (-half_len,  0.0,       np.pi   ),
    ( 0.0,       half_wid,  np.pi/2 )
]

# ── FIGURE SETUP ────────────────────────────────────────
fig, (ax1, ax2, ax3) = plt.subplots(1,3, figsize=(15,5))

# Global Map
ax1.set_title("Global Map")
ax1.set_xlim(0,gridSize); ax1.set_ylim(0,gridSize)
ax1.set_xticks(np.linspace(0,gridSize,gridDiv+1))
ax1.set_yticks(np.linspace(0,gridSize,gridDiv+1))
ax1.grid(True)

body = patches.Rectangle((0,0), robot_length, robot_width,
                         facecolor='blue', alpha=0.6)
ax1.add_patch(body)

arrow = FancyArrowPatch(
    posA=(x_pos, y_pos),
    posB=(x_pos + half_len*np.cos(orientation),
          y_pos + half_len*np.sin(orientation)),
    arrowstyle='-|>',
    mutation_scale=10,
    color='red',
    linewidth=1
)
ax1.add_patch(arrow)

for oi,oj in obs_cells:
    ax1.add_patch(patches.Rectangle(
        (oi*cellSize, oj*cellSize), cellSize, cellSize,
        facecolor='black', edgecolor='black'
    ))

sensor_lines = [ax1.plot([],[], '--', color='green')[0]
                for _ in sensor_defs]

# Robot‑Centric View
ax2.set_title("Robot‑Centric View")
ax2.set_xlim(-gridSize, gridSize); ax2.set_ylim(-gridSize, gridSize)
ax2.set_aspect('equal'); ax2.grid(True)

rb = patches.Rectangle((-half_len, -half_wid),
                       robot_length, robot_width,
                       facecolor='blue', alpha=0.6)
ax2.add_patch(rb)

heading = FancyArrowPatch(
    posA=(0,0),
    posB=(robot_length/2, 0),
    arrowstyle='-|>',
    mutation_scale=10,
    color='red',
    linewidth=1
)
ax2.add_patch(heading)

rc_lines = [ax2.plot([],[], '--', color='green')[0]
            for _ in sensor_defs]
rc_dots  = [ax2.plot([],[], 'go')[0]
            for _ in sensor_defs]

# Occupancy Grid
ax3.set_title("Occupancy Grid")
ax3.set_xlim(0,gridSize); ax3.set_ylim(0,gridSize)
ax3.set_xticks(np.linspace(0,gridSize,gridDiv+1))
ax3.set_yticks(np.linspace(0,gridSize,gridDiv+1))
ax3.grid(True)

occ_cmap = ListedColormap(['#00FF00','#FF0000'])
occ_im = ax3.imshow(
    occupancy,
    cmap=occ_cmap,
    origin='lower',
    extent=[0,gridSize,0,gridSize],
    vmin=0, vmax=1,
    interpolation='none'
)
robot_dot, = ax3.plot([],[], 'bo')

# ── ANIMATION UPDATE ────────────────────────────────────
def update(frame):
    # 1) ensure static obstacles remain red
    for oi,oj in obs_cells:
        occupancy[oj,oi] = 1

    # 2) move robot
    move_robot()

    # 3) carve out free & mark hits per sensor
    for (dx, dy, dth), line in zip(sensor_defs, sensor_lines):
        ang = orientation + dth
        ox = x_pos + dx*np.cos(orientation) - dy*np.sin(orientation)
        oy = y_pos + dx*np.sin(orientation) + dy*np.cos(orientation)
        ox, oy = np.clip(ox,0,gridSize), np.clip(oy,0,gridSize)

        t, hx, hy = sensor_hit(ox, oy, ang)
        ci0_s = min(int(ox/cellSize), gridDiv-1)
        cj0_s = min(int(oy/cellSize), gridDiv-1)
        ci1_s = min(int(hx/cellSize), gridDiv-1)
        cj1_s = min(int(hy/cellSize), gridDiv-1)

        # carve free
        for ci, cj in bresenham(ci0_s, cj0_s, ci1_s, cj1_s)[1:-1]:
            occupancy[cj,ci] = 0
        # mark hit
        occupancy[cj1_s,ci1_s] = 1

        line.set_data([ox, hx], [oy, hy])

    # 4) update Global Map robot body & arrow
    tr = (transforms.Affine2D()
          .rotate_deg_around(x_pos, y_pos, np.degrees(orientation))
          + ax1.transData)
    body.set_xy((x_pos-half_len, y_pos-half_wid))
    body.set_transform(tr)

    x2 = x_pos + half_len*np.cos(orientation)
    y2 = y_pos + half_len*np.sin(orientation)
    arrow.set_positions((x_pos, y_pos), (x2, y2))

    # 5) update Robot‑Centric rays & dots
    for line, dot, (dx, dy, dth) in zip(rc_lines, rc_dots, sensor_defs):
        ang = orientation + dth
        ox = x_pos + dx*np.cos(orientation) - dy*np.sin(orientation)
        oy = y_pos + dx*np.sin(orientation) + dy*np.cos(orientation)
        t, hx, hy = sensor_hit(ox, oy, ang)
        dx0, dy0 = hx - x_pos, hy - y_pos
        xl =  dx0*np.cos(orientation) + dy0*np.sin(orientation)
        yl = -dx0*np.sin(orientation) + dy0*np.cos(orientation)
        line.set_data([0, xl], [0, yl])
        dot.set_data([xl], [yl])

    # 6) update occupancy image & robot dot
    occ_im.set_data(occupancy)
    robot_dot.set_data([x_pos], [y_pos])

    return [body, arrow] + sensor_lines + rc_lines + rc_dots + [occ_im, robot_dot]

anim = FuncAnimation(fig, update, blit=True, interval=update_interval)
plt.show()
