#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand

import sys
from os import getcwd
sys.path.append(getcwd() + "/../Core")

from arm_controller import ArmController

if __name__=='__main__':

    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = sys.argv[1]
    lynx = ArmController(color)

    sleep(1) # wait for setup

    # interact with simulator, such as...

    # get state of your robot
    [q, qd]  = lynx.get_state()
    print(q)
    print(qd)

    # get state of scoreable objects
    [name, pose, twist] = lynx.get_object_state()
    print(name)
    print(pose)
    print(twist)

    # Separate based on dynamic vs static (red v blue)
    numBlocks = len(name)
    dynamicBlocks = []
    staticRedBlocks = [] # Maybe sort indices by distance to goal
    staticBlueBlocks = []

    ## Parse name array and separate by block type
    for n in range(numBlocks):
        if name[n][5] == 'd':
            dynamicBlocks.append(n)

        elif name[n][5] == 's':
            # Sort statics by which side of the world they are on (Y position)
            if pose[n][1, 3] < 0:
                staticRedBlocks.append(n)
            else:
                staticBlueBlocks.append(n)

    print("Purple: " + str(dynamicBlocks))
    print("Red:    " + str(staticRedBlocks))
    print("Blue:   " + str(staticBlueBlocks)) # Check if I got the static colors right!

    # Have a check on whether we are the red or blue robot?

    # get state of your opponent's robot
    [q, qd]  = lynx.get_opponent_state()
    print(q)
    print(qd)

    lynx.stop()
