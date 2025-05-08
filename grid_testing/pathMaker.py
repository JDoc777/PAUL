import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import matplotlib.transforms as transforms
import numpy as np
import random
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap
from collections import deque

# ── CONFIG ───────────────────────────────────────────────
gridSize        = 2.0    # meters
gridDiv         = 50     # divisions per axis
robot_length    = 0.25   # m
robot_width     = 0.20   # m
update_interval = 10     # ms/frame
num_obstacles   = 5      # random static blocks

cellSize       = gridSize / gridDiv
half_len       = robot_length / 2
half_wid       = robot_width  / 2
scan_increment = np.pi / (2 * gridDiv)

# ── STATE & FLAGS ───────────────────────────────────────
x_pos, y_pos    = (random.uniform(half_len, gridSize-half_len),
                   random.uniform(half_wid, gridSize-half_wid))
orientation     = 0.0  # will be set after initial scan

# occupancy: 1=blocked, 0=free
occupancy       = np.ones((gridDiv, gridDiv), dtype=int)
visited         = set()

scanning        = True
rotation_accum  = 0.0

# random static obstacles
all_cells = [(i,j) for i in range(1,gridDiv-1)
                    for j in range(1,gridDiv-1)]
obs_cells = random.sample(all_cells, num_obstacles)

# sensor offsets in robot frame
sensor_defs = [
    ( half_len,  0.0,       0.0     ),
    ( 0.0,      -half_wid, -np.pi/2 ),
    (-half_len,  0.0,       np.pi   ),
    ( 0.0,       half_wid,  np.pi/2 )
]

# ── INITIAL OCCUPANCY & VISIT ────────────────────────────
# carve out robot footprint
ci0 = int((x_pos-half_len)/cellSize)
ci1 = int((x_pos+half_len)/cellSize)
cj0 = int((y_pos-half_wid)/cellSize)
cj1 = int((y_pos+half_wid)/cellSize)
occupancy[cj0:cj1+1, ci0:ci1+1] = 0

# place static obstacles
for oi,oj in obs_cells:
    occupancy[oj,oi] = 1

def mark_visited(x, y):
    ci = min(int(x/cellSize), gridDiv-1)
    cj = min(int(y/cellSize), gridDiv-1)
    visited.add((ci, cj))
    occupancy[cj,ci] = 0

mark_visited(x_pos, y_pos)

# ── BRESENHAM & SENSOR HIT ──────────────────────────────
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
            err -= dy; x += sx
        if e2 < dx:
            err += dx; y += sy
    return pts

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
    # static obstacles
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
    if not cands:
        # no intersection: cast to boundary
        t = gridSize
        return (t, ox+ca*t, oy+sa*t)
    return min(cands, key=lambda v:v[0])

# ── FIND FARTHEST REACHABLE CELL ────────────────────────
def find_farthest_cell():
    start = ( min(int(x_pos/cellSize),gridDiv-1),
              min(int(y_pos/cellSize),gridDiv-1) )
    q = deque([(start, 0)])
    seen = {start}
    far, far_dist = start, 0
    while q:
        (ci, cj), d = q.popleft()
        if d > far_dist:
            far, far_dist = (ci, cj), d
        for di, dj in [(1,0),(-1,0),(0,1),(0,-1)]:
            ni, nj = ci+di, cj+dj
            if (0 <= ni < gridDiv and 0 <= nj < gridDiv
                and (ni,nj) not in seen
                and occupancy[nj,ni] == 0):
                seen.add((ni,nj))
                q.append(((ni,nj), d+1))
    return far

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

arrow = FancyArrowPatch(posA=(0,0), posB=(half_len,0),
                       arrowstyle='-|>', mutation_scale=10,
                       color='red', linewidth=1,
                       transform=transforms.Affine2D())
ax1.add_patch(arrow)

for oi,oj in obs_cells:
    ax1.add_patch(patches.Rectangle(
        (oi*cellSize, oj*cellSize),
        cellSize, cellSize,
        facecolor='black', edgecolor='black'
    ))

# Robot‑Centric View
ax2.set_title("Robot‑Centric View")
ax2.set_xlim(-gridSize, gridSize); ax2.set_ylim(-gridSize, gridSize)
ax2.set_aspect('equal'); ax2.grid(True)

rb = patches.Rectangle((-half_len, -half_wid),
                       robot_length, robot_width,
                       facecolor='blue', alpha=0.6)
ax2.add_patch(rb)

heading = FancyArrowPatch((0,0), (half_len,0),
                         arrowstyle='-|>', mutation_scale=10,
                         color='red', linewidth=1)
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

occ_im = ax3.imshow(occupancy,
                    cmap=ListedColormap(['#00FF00','#FF0000']),
                    origin='lower',
                    extent=[0,gridSize,0,gridSize],
                    vmin=0, vmax=1,
                    interpolation='none')

robot_dot, = ax3.plot([],[], 'bo')

# ── ANIMATION UPDATE ────────────────────────────────────
def update(frame):
    global x_pos, y_pos, orientation, scanning, rotation_accum

    # 1) carve free space with sensors
    for i, (dx, dy, dth) in enumerate(sensor_defs):
        ang = orientation + dth
        ox = x_pos + dx*np.cos(orientation) - dy*np.sin(orientation)
        oy = y_pos + dx*np.sin(orientation) + dy*np.cos(orientation)
        ox, oy = np.clip(ox,0,gridSize), np.clip(oy,0,gridSize)

        t, hx, hy = sensor_hit(ox, oy, ang)
        ci0 = min(int(ox/cellSize), gridDiv-1)
        cj0 = min(int(oy/cellSize), gridDiv-1)
        ci1 = min(int(hx/cellSize), gridDiv-1)
        cj1 = min(int(hy/cellSize), gridDiv-1)

        for ci, cj in bresenham(ci0, cj0, ci1, cj1)[1:-1]:
            occupancy[cj,ci] = 0
        occupancy[cj1,ci1] = 1

        # robot‑centric view
        dx0, dy0 = hx - x_pos, hy - y_pos
        xl = dx0*np.cos(-orientation) - dy0*np.sin(-orientation)
        yl = dx0*np.sin(-orientation) + dy0*np.cos(-orientation)
        rc_lines[i].set_data([0, xl], [0, yl])
        rc_dots[i].set_data([xl], [yl])

    # 2) scanning phase: rotate in place
    if scanning:
        orientation = (orientation + scan_increment) % (2*np.pi)
        rotation_accum += scan_increment

        if rotation_accum >= 2*np.pi:
            scanning       = False
            rotation_accum = 0.0
            mark_visited(x_pos, y_pos)

            # pick farthest reachable cell using BFS
            far_ci, far_cj = find_farthest_cell()
            # compute orientation toward that cell's center
            tx = (far_ci + 0.5) * cellSize
            ty = (far_cj + 0.5) * cellSize
            orientation = np.arctan2(ty - y_pos, tx - x_pos)

    else:
        # 3) move straight forward until obstacle
        sp = cellSize * 0.5
        nx = np.clip(x_pos + sp*np.cos(orientation),
                     half_len, gridSize-half_len)
        ny = np.clip(y_pos + sp*np.sin(orientation),
                     half_wid,  gridSize-half_wid)

        if occupancy[int(ny/cellSize), int(nx/cellSize)] == 0:
            x_pos, y_pos = nx, ny
            mark_visited(x_pos, y_pos)
        else:
            # hit obstacle → re-enter scanning
            scanning = True

    # 4) update robot on global map
    tr = (transforms.Affine2D()
          .rotate_deg_around(x_pos, y_pos, np.degrees(orientation))
          + ax1.transData)
    body.set_xy((x_pos-half_len, y_pos-half_wid))
    body.set_transform(tr)
    arrow.set_transform(tr)

    # 5) update occupancy & robot dot
    occ_im.set_data(occupancy)
    robot_dot.set_data([x_pos], [y_pos])

    return [body, arrow, occ_im, robot_dot] + rc_lines + rc_dots

anim = FuncAnimation(fig, update, blit=True, interval=update_interval,
                     cache_frame_data=False)

plt.show()
