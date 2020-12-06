#!/usr/bin/python2
from arm_controller import ArmController
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
from gripStatic import calcNewQ4
from calcIK import inverse
from IK_velocity import IK_velocity
import rospy
import sys
from os import getcwd
from copy import deepcopy
sys.path.append(getcwd() + "/../Core")

# Function definition to set paths
def setpath(finq):
    ispos = False
    lynx.command(finq)
    while ispos == False:
        sleep(3)
        curq, _ = lynx.get_state()
        print(curq)
<<<<<<< Updated upstream
        if abs(curq[0]-finq[0]) < 0.05:
            if abs(curq[1]-finq[1]) < 0.05:
                if abs(curq[2]-finq[2]) < 0.05:
                    if abs(curq[3]-finq[3]) < 0.05:
                        if abs(curq[4]-finq[4]) < 0.05:
                            ispos = True
=======
        if abs(curq[0]-finq[0])<0.05:
            if abs(curq[1]-finq[1])<0.05:
                if abs(curq[2]-finq[2])<0.05:
                    if abs(curq[3]-finq[3])<0.05:
                        if abs(curq[4]-finq[4])<0.05:
                            ispos=True
                            # if abs(curq[5]-finq[5])<0.05:
#Finding the closest block to track within the dynamic blocks
def closestblock():
    [name, pose, twist] = lynx.get_object_state()
    block_ang=np.empty(len(dynamicBlocks))
    for r in range(0,len(dynamicBlocks)):
        block_ang[r]=(3*np.pi/2-0.1)-np.arctan2(pose[dynamicBlocks[r]][1,-1],pose[dynamicBlocks[r]][0,-1])
        if block_ang[r]<0:
            block_ang[r]=block_ang[r]+1000
    print("block angles:",block_ang)
    i=np.argmin(block_ang)
    #return index of dynamicBlocks to pickup
    print("index:",i)
    return i
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
    [q, qd] = lynx.get_opponent_state()
    print("Opponent position:", q)
    print("Opponent velocity:", qd)

  
=======
    [q, qd]  = lynx.get_opponent_state()
    print("Opponent position:",q)
    print("Opponent velocity:",qd)

    #Dynamic block test
    # setpath([0.9,0.05,np.pi/5,0,1.5,30])
        # Separate based on dynamic vs static (red v blue)
>>>>>>> Stashed changes
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
        elif pose[staticBlocks[r]][2,-1]<30 and pose[staticBlocks[r]][2,-1]>5:
            platform2.append(staticBlocks[r])
    print("platform1:",platform1)
    print("platform2:",platform2)

<<<<<<< Updated upstream
    setpath([0,0,0,0,0,0])

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

=======
    #Making an array to

    #Dynamic test:
    #STARTING POSE FOR DYNAMIC
    Tf=np.array([[-0.5,0,0.5,110],[0.5,0,0.5,110],[0,1,0,85],[0,0,0,1]])
    newq=inverse(Tf,30)
    setpath(newq)
    # [name, pose, twist] = lynx.get_object_state()
    i=closestblock()
    print("index:",i)
    chosen_block=dynamicBlocks[i]
    rost=rospy.get_time()
    print("Twist:",twist[chosen_block])
    angv=twist[chosen_block][-1]
    print("Pose:",pose[chosen_block])
    block_r=np.sqrt(pose[chosen_block][0,-1]**2+pose[chosen_block][1,-1]**2)
    #Setting position within 30 mm to blocks
    hyp=200*np.sqrt(2)-(block_r+30)
    block_offset=np.sin(np.pi/4)*hyp
    print("block offset:",block_offset)
    print("block+30:",block_r+30)
    # block_offset=200-(block_r+30)/np.sqrt(2)
    Tf_bl=deepcopy(Tf)
    Tf_bl[0,-1]=block_offset
    Tf_bl[1,-1]=block_offset
    newq=inverse(Tf_bl,30)
    setpath(newq)
    t0e=pose[chosen_block]
    btheta=np.arctan2(t0e[1,-1],t0e[0,-1])
    setpost=((3*np.pi/2)-btheta)/angv
    # print(setpost+rost)
    dq=IK_velocity(newq,np.array([50,50,0]),np.array([0,0,0]),6)
    dq[-1]=30
    while rospy.get_time() < setpost+rost-5:
        print("waiting for block")
        pass
    bset=rospy.get_time()
    # while rospy.get_time()<bset+1:
    #     lynx.set_vel(dq)
    #     curq,_= lynx.get_state()
    #     dq=IK_velocity(curq,np.array([50,50,0]),np.array([0,0,0]),6)
    #     print("getting block")

    curq,_= lynx.get_state()
    curq[-1]=0
    lynx.set_pos(curq)
    sleep(2)
    setpath([-1.4,0.3,-0.3,1.2456,-2,0])

    # for i in range(0,len(platform2)):
    #     #approaching first object in platform 1
    #     Tf=calcNewQ4(q, pose[platform2[i]], color,60)
    #     newq=inverse(Tf,30)
    #     setpath(newq)
    #     #lower to pick up block from platform 1
    #     Tf=calcNewQ4(q, pose[platform2[i]], color,10)
    #     newq=inverse(Tf,30)
    #     setpath(newq)
    #     #grasp
    #     Tf=calcNewQ4(q, pose[platform2[i]], color,10)
    #     newq=inverse(Tf,0)
    #     setpath(newq)
    #     #pick up 120mm above
    #     Tf=calcNewQ4(q, pose[platform2[i]], color,60)
    #     newq=inverse(Tf,0)
    #     setpath(newq)
    #     # #Getting out of the platform area
    #     # setpath([0,0,0,0,-2,0])
    #     #Going towards goal position
    #     setpath([-1.4,0.3,-0.3,1.2456,-2,0])
    #     #Path to first block position on goal
    #     setpath([-1.4,1.2,-1.5,np.pi/2,-np.pi/2,0])
    #     #Releasing the block at the first position
    #     setpath([-1.4,1.2,-1.5,np.pi/2,-np.pi/2,30])
    #     print("end of platform 2")
    # #Getting right to the platform 1
    # # setpath([0,0.5,-0.2,np.pi/2,0,30])
    # # q=[0,0.5,-0.2,np.pi/2,0,30]
    # for i in range(0,len(platform1)):
    #     #approaching first object in platform 1
    #     Tf=calcNewQ4(q, pose[platform1[i]], color,60)
    #     newq=inverse(Tf,30)
    #     setpath(newq)
    #     #lower to pick up block from platform 1
    #     Tf=calcNewQ4(q, pose[platform1[i]], color,10)
    #     newq=inverse(Tf,30)
    #     setpath(newq)
    #     #grasp
    #     Tf=calcNewQ4(q, pose[platform1[i]], color,10)
    #     newq=inverse(Tf,0)
    #     setpath(newq)
    #     #pick up 120mm above
    #     Tf=calcNewQ4(q, pose[platform1[i]], color,60)
    #     newq=inverse(Tf,0)
    #     setpath(newq)
    #     # #Getting out of the platform area
    #     # setpath([0,0,0,0,-2,0])
    #     #Going towards goal position
    #     setpath([-1.1,0.3,-0.3,1.2456,-2,0])
    #     #Path to first block position on goal
    #     setpath([-1.2,1.2,-1.5,np.pi/2,-np.pi/2,0])
    #     #Releasing the block at the first position
    #     setpath([-1.2,1.2,-1.5,np.pi/2,-np.pi/2,30])
    #     print("end of platform 1")


>>>>>>> Stashed changes
    lynx.stop()
