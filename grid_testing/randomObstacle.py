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
update_interval = 0.05   # s/frame

cellSize = gridSize / gridDiv
arrow_hw = cellSize * 0.3
arrow_hl = cellSize * 0.3

# ── STATE ────────────────────────────────────────────────
x_pos       = random.uniform(robot_length/2, gridSize - robot_length/2)
y_pos       = random.uniform(robot_width/2,  gridSize - robot_width/2)
orientation = random.uniform(0, 2*np.pi)

# one static obstacle
obs_i = random.randint(1, gridDiv-2)
obs_j = random.randint(1, gridDiv-2)

# 0=free, 1=occupied
occupancy = np.ones((gridDiv, gridDiv), dtype=int)

# ── HELPERS ──────────────────────────────────────────────
def bresenham(x0,y0,x1,y1):
    pts, dx, dy = [], abs(x1-x0), abs(y1-y0)
    x,y, sx,sy = x0,y0, (1 if x1>x0 else -1), (1 if y1>y0 else -1)
    err = dx-dy
    while True:
        pts.append((x,y))
        if x==x1 and y==y1: break
        e2 = 2*err
        if e2>-dy: err, x = err-dy, x+sx
        if e2< dx: err, y = err+dx, y+sy
    return pts

def cell_under_robot(ci,cj):
    cx, cy = (ci+0.5)*cellSize, (cj+0.5)*cellSize
    dx, dy = cx-x_pos, cy-y_pos
    xb =  dx*np.cos(orientation)+dy*np.sin(orientation)
    yb = -dx*np.sin(orientation)+dy*np.cos(orientation)
    return abs(xb)<=robot_length/2 and abs(yb)<=robot_width/2

def can_place(nx,ny,no):
    ci_min = int((nx-robot_length/2)/cellSize)
    ci_max = int((nx+robot_length/2)/cellSize)
    cj_min = int((ny-robot_width/2)/cellSize)
    cj_max = int((ny+robot_width/2)/cellSize)
    for ci in range(ci_min,ci_max+1):
        for cj in range(cj_min,cj_max+1):
            if 0<=ci<gridDiv and 0<=cj<gridDiv and occupancy[cj,ci]==1:
                return False
    return True

def move_robot():
    global x_pos,y_pos,orientation
    choices = ['forward','backward','left','right','rotate']
    random.shuffle(choices)
    ox,oy,oo = x_pos,y_pos,orientation
    for cmd in choices:
        nx,ny,no = ox,oy,oo
        sp = cellSize*0.5; rp = np.pi/(2*gridDiv)
        if   cmd=='forward':  nx+=sp*np.cos(no); ny+=sp*np.sin(no)
        elif cmd=='backward': nx-=sp*np.cos(no); ny-=sp*np.sin(no)
        elif cmd=='left':     nx-=sp*np.sin(no); ny+=sp*np.cos(no)
        elif cmd=='right':    nx+=sp*np.sin(no); ny-=sp*np.cos(no)
        else:                 no+=rp
        nx = np.clip(nx,robot_length/2,gridSize-robot_length/2)
        ny = np.clip(ny,robot_width/2, gridSize-robot_width/2)
        no %= 2*np.pi
        if can_place(nx,ny,no):
            x_pos,y_pos,orientation = nx,ny,no
            return

def sensor_hit(ox,oy,ang):
    ca,sa = np.cos(ang),np.sin(ang)
    cands = []
    if ca!=0:
        for xw in (0,gridSize):
            t=(xw-ox)/ca
            if t>0: cands.append((t,ox+t*ca,oy+t*sa))
    if sa!=0:
        for yw in (0,gridSize):
            t=(yw-oy)/sa
            if t>0: cands.append((t,ox+t*ca,oy+t*sa))
    return min(cands, key=lambda v:v[0])

# ── PLOTTING ─────────────────────────────────────────────
plt.ion()
fig,(ax1,ax2,ax3)=plt.subplots(1,3,figsize=(15,5))

try:
    while True:
        # 1) carve robot footprint free (interior only)
        ci0 = int((x_pos-robot_length/2)/cellSize)
        ci1 = int((x_pos+robot_length/2)/cellSize)
        cj0 = int((y_pos-robot_width/2)/cellSize)
        cj1 = int((y_pos+robot_width/2)/cellSize)
        for ci in range(ci0,ci1+1):
            for cj in range(cj0,cj1+1):
                if 1<=ci<gridDiv-1 and 1<=cj<gridDiv-1:
                    occupancy[cj,ci]=0
        # always keep static obstacle occupied
        occupancy[obs_j,obs_i]=1

        # 2) attempt move
        move_robot()

        # 3) redraw
        ax1.cla(); ax2.cla(); ax3.cla()
        # -- global map
        ax1.set_title("Global Map")
        ax1.set_xlim(0,gridSize); ax1.set_ylim(0,gridSize)
        ax1.set_xticks(np.linspace(0,gridSize,gridDiv+1))
        ax1.set_yticks(np.linspace(0,gridSize,gridDiv+1))
        ax1.grid(True)
        # robot
        body=patches.Rectangle((x_pos-robot_length/2,y_pos-robot_width/2),
                                robot_length,robot_width,facecolor='blue',alpha=0.6)
        tr=transforms.Affine2D().rotate_deg_around(
            x_pos,y_pos,np.degrees(orientation)
        )+ax1.transData
        body.set_transform(tr); ax1.add_patch(body)
        ax1.arrow(x_pos,y_pos,
                  (robot_length/2)*np.cos(orientation),
                  (robot_length/2)*np.sin(orientation),
                  head_width=arrow_hw,head_length=arrow_hl,fc='red',ec='red')
        # static obstacle
        ax1.add_patch(patches.Rectangle((obs_i*cellSize,obs_j*cellSize),
                                        cellSize,cellSize,facecolor='black'))

        # sonar carve-out
        sensors=[(robot_length/2,0,orientation),
                 (0,-robot_width/2,orientation-np.pi/2),
                 (-robot_length/2,0,orientation+np.pi),
                 (0, robot_width/2,orientation+np.pi/2)]
        for dx,dy,ang in sensors:
            ox=x_pos+dx*np.cos(orientation)-dy*np.sin(orientation)
            oy=y_pos+dx*np.sin(orientation)+dy*np.cos(orientation)
            ox,oy=np.clip(ox,0,gridSize),np.clip(oy,0,gridSize)
            t,hx,hy=sensor_hit(ox,oy,ang)
            ci0=min(int(ox/cellSize),gridDiv-1)
            cj0=min(int(oy/cellSize),gridDiv-1)
            ci1=min(int(hx/cellSize),gridDiv-1)
            cj1=min(int(hy/cellSize),gridDiv-1)
            ray=bresenham(ci0,cj0,ci1,cj1)
            for ci,cj in ray[1:-1]:
                if (1<=ci<gridDiv-1 and 1<=cj<gridDiv-1
                    and not (ci==obs_i and cj==obs_j)):
                    occupancy[cj,ci]=0
            occupancy[cj1,ci1]=1
            ax1.plot([ox,hx],[oy,hy],'--g')
            ax1.text(ox+0.02*np.cos(ang),oy+0.02*np.sin(ang),
                     f"{t:.2f}",color='green',fontsize=8)

        # robot-centric
        ax2.set_title("Robot‑Centric View")
        L=gridSize
        ax2.set_xlim(-L,L); ax2.set_ylim(-L,L)
        ax2.set_aspect('equal'); ax2.grid(True)
        rb=patches.Rectangle((-robot_length/2,-robot_width/2),
                              robot_length,robot_width,
                              facecolor='blue',alpha=0.6)
        ax2.add_patch(rb)
        ax2.arrow(0,0,robot_length/2,0,
                  head_width=arrow_hw,head_length=arrow_hl,
                  fc='red',ec='red')
        for dx,dy,ang in sensors:
            ox=x_pos+dx*np.cos(orientation)-dy*np.sin(orientation)
            oy=y_pos+dx*np.sin(orientation)+dy*np.cos(orientation)
            ox,oy=np.clip(ox,0,gridSize),np.clip(oy,0,gridSize)
            t,hx,hy=sensor_hit(ox,oy,ang)
            dx0,dy0=hx-x_pos,hy-y_pos
            xl=dx0*np.cos(orientation)+dy0*np.sin(orientation)
            yl=-dx0*np.sin(orientation)+dy0*np.cos(orientation)
            ax2.plot([0,xl],[0,yl],'--g')
            ax2.plot(xl,yl,'go')
            ax2.text(xl*1.05,yl*1.05,f"{t:.2f}",
                     color='green',fontsize=8,ha='center',va='center')

        # occupancy grid
        ax3.set_title("Occupancy Grid")
        ax3.set_xlim(0,gridSize); ax3.set_ylim(0,gridSize)
        ax3.set_xticks(np.linspace(0,gridSize,gridDiv+1))
        ax3.set_yticks(np.linspace(0,gridSize,gridDiv+1))
        ax3.grid(True)
        for i in range(gridDiv):
            for j in range(gridDiv):
                clr='#00FF00' if occupancy[j,i]==0 else '#FF0000'
                ax3.add_patch(patches.Rectangle((i*cellSize,j*cellSize),
                                               cellSize,cellSize,
                                               facecolor=clr,edgecolor='black',linewidth=0.5))
        ax3.plot(x_pos,y_pos,'bo')

        plt.pause(update_interval)
        if not plt.fignum_exists(fig.number):
            break

finally:
    plt.close(fig)
