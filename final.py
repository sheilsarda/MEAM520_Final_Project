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

    # get state of your opponent's robot
    [q, qd]  = lynx.get_opponent_state()
    print(q)
    print(qd)

    lynx.stop()
