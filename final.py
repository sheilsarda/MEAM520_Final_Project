#!/usr/bin/python2
from arm_controller import ArmController
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
from gripStatic import calcNewQ4
from calcIK import inverse

import sys
from os import getcwd
sys.path.append(getcwd() + "/../Core")

# Function definition to set paths
def setpath(finq):
    ispos = False
    lynx.command(finq)
    while ispos == False:
        sleep(3)
        curq, _ = lynx.get_state()
        print(curq)
        if abs(curq[0]-finq[0]) < 0.05:
            if abs(curq[1]-finq[1]) < 0.05:
                if abs(curq[2]-finq[2]) < 0.05:
                    if abs(curq[3]-finq[3]) < 0.05:
                        if abs(curq[4]-finq[4]) < 0.05:
                            ispos = True

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = sys.argv[1]
    lynx = ArmController(color)

    sleep(1)  # wait for setup

	# lynx.wait_for_start() # Wait for Start Gun to be fired

    # get state of your robot
    [q, qd] = lynx.get_state()
    print("Lynx position", q)
    print("Lynx velocity", qd)

    # get state of scoreable objects
    [name, pose, twist] = lynx.get_object_state()
    print("object name:", name)
    print("Object pose:", pose)
    print("Object twist:", twist)

    # get state of your opponent's robot
    [q, qd] = lynx.get_opponent_state()
    print("Opponent position:", q)
    print("Opponent velocity:", qd)

  
    numBlocks = len(name)
    dynamicBlocks = []
    staticBlocks = []  # Maybe sort indices by distance to goal

    # Parse name array and separate by block type
    for n in range(numBlocks):
        if name[n][5] == 'd':
            dynamicBlocks.append(n)

        elif name[n][5] == 's':
            # Choose statics by which side of the world they are on (Y position)
            if pose[n][1, 3] < 0 and color == "red":
                staticBlocks.append(n)
            elif color == "blue":
                staticBlocks.append(n)

    print("Dynamic Blocks: " + str(dynamicBlocks))
    print("Static Blocks:  " + str(staticBlocks))

    platform1 = []
    platform2 = []
    for r in range(0, len(staticBlocks)):
        if pose[staticBlocks[r]][2, -1] < 10 and pose[staticBlocks[r]][2, -1] > 5:
            platform1.append(staticBlocks[r])
        elif pose[staticBlocks[r]][2, -1] < 30 and pose[staticBlocks[r]][2, -1] > 5:
            platform2.append(staticBlocks[r])
    print("platform1:", platform1)
    print("platform2:", platform2)

    for i in range(0, len(platform2)):
        # approaching first object in platform 1
        Tf = calcNewQ4(q, pose[platform2[i]], color, 60)
        newq = inverse(Tf, 30)
        setpath(newq)
        
        # lower to pick up block from platform 1
        Tf = calcNewQ4(q, pose[platform2[i]], color, 10)
        newq = inverse(Tf, 30)
        setpath(newq)
        
        # grasp
        Tf = calcNewQ4(q, pose[platform2[i]], color, 10)
        newq = inverse(Tf, 0)
        setpath(newq)
        
        # pick up 120mm above
        Tf = calcNewQ4(q, pose[platform2[i]], color, 60)
        newq = inverse(Tf, 0)
        setpath(newq)
        
        # Going towards goal position
        setpath([-1.4, 0.3, -0.3, 1.2456, -2, 0])
        
        # Path to first block position on goal
        setpath([-1.4, 1.2, -1.5, np.pi/2, -np.pi/2, 0])
        
        # Releasing the block at the first position
        setpath([-1.4, 1.2, -1.5, np.pi/2, -np.pi/2, 30])
        print("end of platform 2")
    
    # Getting right to the platform 1
    for i in range(0, len(platform1)):
        
        # approaching first object in platform 1
        Tf = calcNewQ4(q, pose[platform1[i]], color, 60)
        newq = inverse(Tf, 30)
        setpath(newq)
        
        # lower to pick up block from platform 1
        Tf = calcNewQ4(q, pose[platform1[i]], color, 10)
        newq = inverse(Tf, 30)
        setpath(newq)
        
        # grasp
        Tf = calcNewQ4(q, pose[platform1[i]], color, 10)
        newq = inverse(Tf, 0)
        setpath(newq)
        
        # pick up 120mm above
        Tf = calcNewQ4(q, pose[platform1[i]], color, 60)
        newq = inverse(Tf, 0)
        setpath(newq)

        # Going towards goal position
        setpath([-1.1, 0.3, -0.3, 1.2456, -2, 0])

        # Path to first block position on goal
        setpath([-1.2, 1.2, -1.5, np.pi/2, -np.pi/2, 0])

        # Releasing the block at the first position
        setpath([-1.2, 1.2, -1.5, np.pi/2, -np.pi/2, 30])
        print("end of platform 1")

    lynx.stop()
