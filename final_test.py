#!/usr/bin/python2
import visualize
from matplotlib import pyplot as plt
from matplotlib import animation as ani
from calculateFK import calculateFK
from loadmap import loadmap
from copy import deepcopy
from rrt import rrt
from astar import Astar

from mpl_toolkits.mplot3d import Axes3D

from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand

fig = plt.figure(figsize=(20, 20))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim3d([-600, 600])
ax.set_ylim3d([-600, 600])
ax.set_zlim3d([-600, 600])

map1 = loadmap("maps/final.txt")
obs = map1.obstacles

start = np.array([0, 0, 0, 0, 0, 0])
goal  = np.array([-1, 0, 1, 0, 0, 0])

path = Astar(deepcopy(map1), deepcopy(start), deepcopy(goal))
    
print("Astar path : ")
print(path)

f = calculateFK()
p_start = f.forward(path[0])[0]
line = ax.plot(p_start[:, 0], p_start[:, 1], p_start[:, 2])[0]

visualize.plot_obstacles(ax, obs)
arm = ani.FuncAnimation(fig, visualize.animate, fargs=(path, line), interval=1, blit=False)

plt.show()
