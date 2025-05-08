import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import random
import time

# ── CONFIG ───────────────────────────────────────────────
gridSize        = 1.0    # meters
gridDiv         = 20     # divisions per axis
robot_length    = 0.25   # m
robot_width     = 0.20   # m
update_interval = 0.01   # s/frame
num_obstacles   = 3      # how many random static blocks

cellSize = gridSize / gridDiv
arrow_hw = cellSize * 0.3
arrow_hl = cellSize * 0.3

# ── STATE ────────────────────────────────────────────────
x_pos       = random.uniform(robot_length/2, gridSize - robot_length/2)
y_pos       = random.uniform(robot_width/2,  gridSize - robot_width/2)
orientation = random.uniform(0, 2*np.pi)

# pick multiple distinct interior obstacle cells
all_interior = [(i,j) for i in range(1,gridDiv-1) for j in range(1,gridDiv-1)]
obs_cells = random.sample(all_interior, num_obstacles)

# occupancy grid: 0=free, 1=occupied
occupancy = np.ones((gridDiv, gridDiv), dtype=int)

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

def can_place(nx, ny, no):
    """Return False if robot footprint at (nx,ny,no) overlaps any occupied cell."""
    ci0 = int((nx - robot_length/2) / cellSize)
    ci1 = int((nx + robot_length/2) / cellSize)
    cj0 = int((ny - robot_width/2)  / cellSize)
    cj1 = int((ny + robot_width/2)  / cellSize)
    for ci in range(ci0, ci1+1):
        for cj in range(cj0, cj1+1):
            if 0 <= ci < gridDiv and 0 <= cj < gridDiv and occupancy[cj,ci] == 1:
                return False
    return True

def move_robot():
    """Try random move that doesn’t collide; else stay."""
    global x_pos, y_pos, orientation
    choices = ['forward','backward','left','right','rotate']
    random.shuffle(choices)
    ox, oy, oo = x_pos, y_pos, orientation
    for cmd in choices:
        nx, ny, no = ox, oy, oo
        sp = cellSize*0.5
        rp = np.pi/(2*gridDiv)
        if   cmd=='forward':
            nx += sp*np.cos(no); ny += sp*np.sin(no)
        elif cmd=='backward':
            nx -= sp*np.cos(no); ny -= sp*np.sin(no)
        elif cmd=='left':
            nx -= sp*np.sin(no); ny += sp*np.cos(no)
        elif cmd=='right':
            nx += sp*np.sin(no); ny -= sp*np.cos(no)
        else:  # rotate
            no += rp

        # clamp inside map
        nx = np.clip(nx, robot_length/2, gridSize-robot_length/2)
        ny = np.clip(ny, robot_width/2,  gridSize-robot_width/2)
        no %= 2*np.pi

        if can_place(nx, ny, no):
            x_pos, y_pos, orientation = nx, ny, no
            return

def sensor_hit(ox, oy, ang):
    """
    Cast a ray from (ox,oy) along ang. Returns the nearest intersection
    with either the map border or any static obstacle cell.
    """
    ca, sa = np.cos(ang), np.sin(ang)
    candidates = []

    # map borders
    if ca != 0:
        for xw in (0, gridSize):
            t = (xw - ox)/ca
            if t > 0:
                y_hit = oy + t*sa
                if 0 <= y_hit <= gridSize:
                    candidates.append((t, xw, y_hit))
    if sa != 0:
        for yw in (0, gridSize):
            t = (yw - oy)/sa
            if t > 0:
                x_hit = ox + t*ca
                if 0 <= x_hit <= gridSize:
                    candidates.append((t, x_hit, yw))

    # each static obstacle
    for oi, oj in obs_cells:
        x0, x1 = oi*cellSize, (oi+1)*cellSize
        y0, y1 = oj*cellSize, (oj+1)*cellSize
        # vertical sides
        if ca != 0:
            for xw in (x0, x1):
                t = (xw - ox)/ca
                if t > 0:
                    y_hit = oy + t*sa
                    if y0 <= y_hit <= y1:
                        candidates.append((t, xw, y_hit))
        # horizontal sides
        if sa != 0:
            for yw in (y0, y1):
                t = (yw - oy)/sa
                if t > 0:
                    x_hit = ox + t*ca
                    if x0 <= x_hit <= x1:
                        candidates.append((t, x_hit, yw))

    # pick nearest
    t_min, hx, hy = min(candidates, key=lambda v: v[0])
    return t_min, hx, hy

# ── PLOTTING LOOP ───────────────────────────────────────
plt.ion()
fig, (ax1,ax2,ax3) = plt.subplots(1,3, figsize=(15,5))

try:
    while True:
        # carve robot footprint free (interior only)
        ci0 = int((x_pos-robot_length/2)/cellSize)
        ci1 = int((x_pos+robot_length/2)/cellSize)
        cj0 = int((y_pos-robot_width/2)/cellSize)
        cj1 = int((y_pos+robot_width/2)/cellSize)
        for ci in range(ci0,ci1+1):
            for cj in range(cj0,cj1+1):
                if 1<=ci<gridDiv-1 and 1<=cj<gridDiv-1:
                    occupancy[cj,ci] = 0

        # keep all static obstacles occupied
        for oi,oj in obs_cells:
            occupancy[oj,oi] = 1

        # move robot
        move_robot()

        # clear axes
        ax1.cla(); ax2.cla(); ax3.cla()

        # — Global Map —
        ax1.set_title("Global Map")
        ax1.set_xlim(0,gridSize); ax1.set_ylim(0,gridSize)
        ax1.set_xticks(np.linspace(0,gridSize,gridDiv+1))
        ax1.set_yticks(np.linspace(0,gridSize,gridDiv+1))
        ax1.grid(True)

        # draw robot
        body = patches.Rectangle(
            (x_pos-robot_length/2, y_pos-robot_width/2),
            robot_length, robot_width,
            facecolor='blue', alpha=0.6
        )
        tr = transforms.Affine2D().rotate_deg_around(
            x_pos, y_pos, np.degrees(orientation)
        ) + ax1.transData
        body.set_transform(tr); ax1.add_patch(body)
        ax1.arrow(
            x_pos, y_pos,
            (robot_length/2)*np.cos(orientation),
            (robot_length/2)*np.sin(orientation),
            head_width=arrow_hw, head_length=arrow_hl,
            fc='red', ec='red'
        )

        # draw static obstacles
        for oi,oj in obs_cells:
            ax1.add_patch(patches.Rectangle(
                (oi*cellSize, oj*cellSize),
                cellSize, cellSize,
                facecolor='black', edgecolor='black'
            ))

        # sonar carve-out
        sensors = [
            ( robot_length/2,  0.0,            orientation     ),
            ( 0.0,            -robot_width/2, orientation-np.pi/2 ),
            (-robot_length/2,  0.0,            orientation+np.pi   ),
            ( 0.0,             robot_width/2, orientation+np.pi/2 )
        ]
        for dx, dy, ang in sensors:
            ox = x_pos + dx*np.cos(orientation) - dy*np.sin(orientation)
            oy = y_pos + dx*np.sin(orientation) + dy*np.cos(orientation)
            ox, oy = np.clip(ox,0,gridSize), np.clip(oy,0,gridSize)

            t, hx, hy = sensor_hit(ox, oy, ang)
            ci0 = min(int(ox/cellSize), gridDiv-1)
            cj0 = min(int(oy/cellSize), gridDiv-1)
            ci1 = min(int(hx/cellSize), gridDiv-1)
            cj1 = min(int(hy/cellSize), gridDiv-1)

            ray = bresenham(ci0, cj0, ci1, cj1)
            # carve free up to (but not including) the hit cell
            for ci, cj in ray[1:-1]:
                occupancy[cj,ci] = 0
            # keep hit cell occupied
            occupancy[cj1,ci1] = 1

            ax1.plot([ox, hx], [oy, hy], '--g')
            ax1.text(
                ox + 0.02*np.cos(ang),
                oy + 0.02*np.sin(ang),
                f"{t:.2f}", color='green', fontsize=8
            )

        # — Robot‑Centric View —
        ax2.set_title("Robot‑Centric View")
        ax2.set_xlim(-gridSize, gridSize)
        ax2.set_ylim(-gridSize, gridSize)
        ax2.set_aspect('equal'); ax2.grid(True)
        rb = patches.Rectangle(
            (-robot_length/2, -robot_width/2),
            robot_length, robot_width,
            facecolor='blue', alpha=0.6
        )
        ax2.add_patch(rb)
        ax2.arrow(
            0,0, robot_length/2, 0,
            head_width=arrow_hw, head_length=arrow_hl,
            fc='red', ec='red'
        )
        for dx, dy, ang in sensors:
            ox = x_pos + dx*np.cos(orientation) - dy*np.sin(orientation)
            oy = y_pos + dx*np.sin(orientation) + dy*np.cos(orientation)
            ox, oy = np.clip(ox,0,gridSize), np.clip(oy,0,gridSize)

            t, hx, hy = sensor_hit(ox, oy, ang)
            dx0, dy0 = hx - x_pos, hy - y_pos
            xl = dx0*np.cos(orientation) + dy0*np.sin(orientation)
            yl = -dx0*np.sin(orientation) + dy0*np.cos(orientation)

            ax2.plot([0, xl], [0, yl], '--g')
            ax2.plot(xl, yl, 'go')

        # — Occupancy Grid —
        ax3.set_title("Occupancy Grid")
        ax3.set_xlim(0,gridSize); ax3.set_ylim(0,gridSize)
        ax3.set_xticks(np.linspace(0,gridSize,gridDiv+1))
        ax3.set_yticks(np.linspace(0,gridSize,gridDiv+1))
        ax3.grid(True)
        for i in range(gridDiv):
            for j in range(gridDiv):
                clr = '#00FF00' if occupancy[j,i]==0 else '#FF0000'
                ax3.add_patch(patches.Rectangle(
                    (i*cellSize, j*cellSize),
                    cellSize, cellSize,
                    facecolor=clr, edgecolor='black', linewidth=0.5
                ))
        ax3.plot(x_pos, y_pos, 'bo')

        plt.pause(update_interval)
        if not plt.fignum_exists(fig.number):
            break

finally:
    plt.close(fig)
