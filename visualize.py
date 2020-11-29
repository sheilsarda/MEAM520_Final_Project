import numpy as np
from calculateFK import calculateFK

def animate(i, path, line):
    fk = calculateFK()
    pos = fk.forward(path[i])[0]
    line.set_data_3d(pos[:, 0], pos[:, 1], pos[:, 2])

def plot_obstacle(ax, obstacle):
    x = np.linspace(obstacle[0], obstacle[3], num=10)
    y = np.linspace(obstacle[1], obstacle[4], num=10)
    z = np.linspace(obstacle[2], obstacle[5], num=10)

    x1, z1 = np.meshgrid(x, z)
    y11 = np.ones_like(x1) * obstacle[1]
    y12 = np.ones_like(x1) * obstacle[4]

    x2, y2 = np.meshgrid(x, y)
    z21 = np.ones_like(x2) * obstacle[2]
    z22 = np.ones_like(x2) * obstacle[5]

    y3, z3 = np.meshgrid(y, z)
    x31 = np.ones_like(y3) * obstacle[0]
    x32 = np.ones_like(y3) * obstacle[3]

    ax.plot_wireframe(x1, y11, z1, color='b', rstride=3, cstride=3, alpha=0.6)

    ax.plot_wireframe(x1, y12, z1,
                      color='b',
                      rstride=3,
                      cstride=3,
                      alpha=0.6)

    ax.plot_wireframe(x2, y2, z21,
                      color='b',
                      rstride=3,
                      cstride=3,
                      alpha=0.6)

    ax.plot_wireframe(x2, y2, z22,
                      color='b',
                      rstride=3,
                      cstride=3,
                      alpha=0.6)

    ax.plot_wireframe(x31, y3, z3,
                      color='b',
                      rstride=3,
                      cstride=3,
                      alpha=0.6)

    ax.plot_wireframe(x32, y3, z3,
                      color='b',
                      rstride=3,
                      cstride=3,
                      alpha=0.6)

def plot_obstacles(ax, obstacles):
    for o in obstacles:
        plot_obstacle(ax, o)
    
