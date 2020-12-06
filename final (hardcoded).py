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
#Function definition to set paths
def setpath(finq):
    ispos=False
    lynx.set_pos(finq)
    while ispos==False:
        sleep(5)
        curq,_=lynx.get_state()
        print(curq)
        if abs(curq[0]-finq[0])<0.05:
            if abs(curq[1]-finq[1])<0.05:
                if abs(curq[2]-finq[2])<0.05:
                    if abs(curq[3]-finq[3])<0.05:
                        if abs(curq[4]-finq[4])<0.05:
                            ispos=True
                            # if abs(curq[5]-finq[5])<0.05:

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
    print("Lynx position",q)
    print("Lynx velocity",qd)

    # get state of scoreable objects
    [name, pose, twist] = lynx.get_object_state()
    print("object name:",name)
    print("Object pose:",pose)
    print("Object twist:",twist)

    # get state of your opponent's robot
    [q, qd]  = lynx.get_opponent_state()
    print("Opponent position:",q)
    print("Opponent velocity:",qd)

    # finq=[np.pi/4,np.pi/5,-np.pi/5,np.pi/2,0,35]

#STAGE 1: Dynamic blocks
    #lynx.set_pos([1.2,0,0,1.2456,-2,0]) #Fix this
    #FIGURE OUT HOW TO PICK UP DYNAMIC block

#Stage 2: Static blocks
    #lynx.set_pos([1.2,0,0,1.2456,-2,0]) #Fix this
    #Find out closest block to edge and find its T0e
    #Input its T0e to CALCIK Function
    #Figure out proper theta 5 and grasp
    #Send to goal point
    #repeat for next block on platform

    #Platform 1: Closest to the roundtable
    #Getting right to the platform
    setpath([0,0.5,-0.2,np.pi/2,0,30])
    #going 30mm above block of interest
    setpath([-0.1031,0.89596,-0.57086,1.2456,-2,30])
    #Decending on the block
    setpath([-0.1031,0.953,-0.55943,1.1697,-2,30])
    #Grasping the block
    setpath([-0.1031,0.953,-0.55943,1.1697,-2,0])
    #picking up the block 60 mm vertically
    setpath([-0.1031,0.83142,-0.6326,1.37198,-2,0])
    #Getting out of the platform area
    setpath([0,0,0,0,-2,0])
    #Going towards goal position
    setpath([-1.2,0.5,-0.3,1.2456,-2,0])
    #Path to first block position on goal
    setpath([-1.2,1.3,-1.5,np.pi/2,-np.pi/2,0])
    #Releasing the block at the first position
    setpath([-1.2,1.3,-1.5,np.pi/2,-np.pi/2,30])

    # setpath([0.9,0.05,np.pi/5,0,1.5,30])

    lynx.stop()
