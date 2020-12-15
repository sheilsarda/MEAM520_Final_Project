#!/usr/bin/python2

from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
from gripStatic import calcNewQ4
from gripStatic import horizontalAngle
from calcIK import inverse
import rospy
import sys
from os import getcwd
from copy import deepcopy
sys.path.append(getcwd() + "/../Core")
from arm_controller import ArmController


# Function definition to set paths using command

def setpath(finq):
    ispos = False
    lynx.command(finq)
    print("setting path")
    while ispos == False:
        sleep(1)
        curq, _ = lynx.get_state()
        # print(curq)
        if abs(curq[0]-finq[0]) < 0.05:
            if abs(curq[1]-finq[1]) < 0.05:
                if abs(curq[2]-finq[2]) < 0.05:
                    if abs(curq[3]-finq[3]) < 0.05:
                        if abs(curq[4]-finq[4]) < 0.05:
                            ispos = True


# Finding the closest block to track within the dynamic blocks
def closestblock(color):
    [name, pose, twist] = lynx.get_object_state()
    avail_block=avail_dyn()

    if(len(avail_block) == 0): return -1

    block_ang=np.empty(len(avail_block))
    ang_diff=np.empty(len(avail_block))
    for r in range(0,len(avail_block)):
        n=dynamicBlocks.index(avail_block[r])
        # if np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])<0:
        #     block_ang[r]=np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])+2*np.pi
        # else:
        #     block_ang[r]=np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])
        if color == "red":
            if np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])<0:
                block_ang[r]=np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])+2*np.pi
            else:
                block_ang[r]=np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])
            ang_diff[r]=((5*np.pi)/4-0.05)-block_ang[r]
            pos_vals=np.array([b for b in ang_diff if b>=0])
        else:
            # if block_ang[r]>np.pi/4:
            #     ang_diff[r]=(2*np.pi)-block_ang[r]+(np.pi/4)
            # else:
            #     ang_diff[r]=(np.pi/4)-block_ang[r]
            block_ang[r]=np.arctan2(pose[dynamicBlocks[n]][1,-1],pose[dynamicBlocks[n]][0,-1])
            if block_ang[r]<0:
                ang_diff[r]=-block_ang[r]+(np.pi/4)
            elif block_ang[r]>np.pi/4:
                ang_diff[r]=(np.pi-block_ang[r])+np.pi+np.pi/4
            else:
                 ang_diff[r]=np.pi/4-block_ang[r]
    if color == "red":
        i=np.argmin(ang_diff)
        if len(pos_vals)>0:
            i=np.argmin([b for b in ang_diff if b>=0])
        else:
            i=np.argmax(ang_diff)
    else:
        i=np.argmin(ang_diff)
    print("available blocks",ang_diff)
    # i=np.argmin([b for b in ang_diff if b>=0])
    print("chosen",ang_diff[i])
    f=dynamicBlocks.index(avail_block[i])
    print("confirmation",pose[dynamicBlocks[f]])
    return f

def avail_dyn():
    _, pose, _ = lynx.get_object_state()
    # list of dynamic blocks on platform
    dyn_platform = []
    for n in range(len(dynamicBlocks)):
        if pose[dynamicBlocks[n]][0, -1] <= 100 and pose[dynamicBlocks[n]][0, -1] >= -100:
            if pose[dynamicBlocks[n]][1, -1] <= 100 and pose[dynamicBlocks[n]][1, -1] >= -100:
                if pose[dynamicBlocks[n]][2, -1] <= 80 and pose[dynamicBlocks[n]][2, -1] >= 40:
                    dyn_platform.append(dynamicBlocks[n])
    print("blocks available", dyn_platform)
    return dyn_platform

def plat_blocks(staticBlocks):
    name,pose,_= lynx.get_object_state()
    platform1 = []
    platform2 = []
    goalPlatform=[]
    for r in range(0, len(staticBlocks)):
        if pose[staticBlocks[r]][2, -1] < 10 and pose[staticBlocks[r]][2, -1] > 5:
            platform1.append(staticBlocks[r])
        elif pose[staticBlocks[r]][2, -1] < 30 and pose[staticBlocks[r]][2, -1] > 5:
            platform2.append(staticBlocks[r])
        elif pose[staticBlocks[r]][2, -1]>40 and abs(pose[staticBlocks[r]][1, -1])>200:
            goalPlatform.append(staticBlocks[r])

    print("platform1   :", platform1)
    print("platform2   :", platform2)
    print("goalPlatform:", goalPlatform)
    return platform1, platform2, goalPlatform

    
if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = sys.argv[1]
    lynx = ArmController(color)

    sleep(1)  # wait for setup

    #lynx.wait_for_start() # Wait for Start Gun to be fired

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

    # Dynamic block test - separate based
    # on dynamic vs static (red v blue)

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
            elif pose[n][1, 3] > 0 and color == "blue":
                staticBlocks.append(n)

    print("Dynamic Blocks: " + str(dynamicBlocks))
    print("Static Blocks:  " + str(staticBlocks))
    platform1,platform2,goalPlatform_stat=plat_blocks(staticBlocks)

    # tf test:
    if color == "red":
        goalPos = np.array([-110.0, -510.0, 50.5])
    else:
        goalPos = np.array([110.0, 510.0, 50.5])
    currStackDyn = 0
    # setpath(newq)
    #Making an array to
    #
    #Dynamic test:
    while len(avail_dyn())>0:
        Tf=np.array([[-0.5,0,0.5,110],[0.5,0,0.5,110],[0,1,0,75],[0,0,0,1]])
        newq=inverse(Tf,30)
        setpath(newq)
        # [name, pose, twist] = lynx.get_object_state()
        i=closestblock(color)
        print("index:",i)
        chosen_block=dynamicBlocks[i]
        rost=rospy.get_time()
        A=np.sqrt(2*(200**2))
        block_r=np.sqrt(pose[chosen_block][0,-1]**2+pose[chosen_block][1,-1]**2)
        #Setting position within 50 mm to blocks
        block_offset=(A-(block_r+50))/np.sqrt(2)
        Tf_bl=deepcopy(Tf)
        Tf_bl[0,-1]=block_offset
        Tf_bl[1,-1]=block_offset
        newq=inverse(Tf_bl,30)
        setpath(newq)
        t0e=pose[chosen_block]
        
      
        # btheta=np.arctan2(t0e[1,-1],t0e[0,-1])
        # Setting position to scoop up and go 10 mm ahead
        block_offset=(A-(block_r-5))/np.sqrt(2)
        Tf_bl1=deepcopy(Tf)
        Tf_bl1[0,-1]=block_offset
        Tf_bl1[1,-1]=block_offset
        Tf_bl1[2,-1]=70
        newq=inverse(Tf_bl1,30)
        cur_block_ang=np.arctan2(pose[chosen_block][1,-1],pose[chosen_block][0,-1])

        if color == "red":
            if cur_block_ang<0:
                cur_block_ang+=np.pi
            while cur_block_ang<=((5*np.pi)/4)-0.05:
                _,pose,_=lynx.get_object_state()
                if (abs(pose[dynamicBlocks[i]][1,3])  < 100):
                    # print(np.arctan2(pose[chosen_block][1,-1],pose[chosen_block][0,-1]))
                    cur_block_ang=np.arctan2(pose[chosen_block][1,-1],pose[chosen_block][0,-1])
                    if cur_block_ang<0:
                        cur_block_ang+=2*np.pi
                    print("waiting for block")
                    print("red")
        else:
            # if cur_block_ang<0:
            #     ang_dif=-np.pi-cur_block_ang+(np.pi/4)
            # elif cur_block_ang>(np.pi)/4:
            #     ang_dif=(2*np.pi)-cur_block_ang+(np.pi/4)
            # else:
            #     ang_dif=(np.pi/4)-cur_block_ang
            if cur_block_ang<0:
                ang_dif=-cur_block_ang+(np.pi/4)
            elif cur_block_ang>(np.pi)/4:
                ang_dif=(np.pi-cur_block_ang)+(np.pi)+(np.pi/4)
            else:
                ang_dif=(np.pi/4)-cur_block_ang
            # print("current block angle:",cur_block_ang)
            while ang_dif>0.04:
                _,pose,_=lynx.get_object_state()
                    # print("block position:",np.arctan2(pose[chosen_block][1,-1],pose[chosen_block][0,-1]))
                if (abs(pose[dynamicBlocks[i]][1,3])  < 100):
                    cur_block_ang=np.arctan2(pose[chosen_block][1,-1],pose[chosen_block][0,-1])
                    if cur_block_ang<0:
                        ang_dif=-cur_block_ang+(np.pi/4)
                    elif cur_block_ang>(np.pi)/4:
                        ang_dif=(np.pi-cur_block_ang)+(np.pi)+(np.pi/4)
                    else:
                        ang_dif=(np.pi/4)-cur_block_ang
                    print("current block angle:",cur_block_ang)
                    print("current angle diff:",ang_dif)
                    print("waiting for block")
        setpath(newq)
        newq=inverse(Tf_bl1,-15)
        setpath(newq)
        Tf_bl2=deepcopy(Tf_bl1)
        Tf_bl2[2,-1]=110
        newq=inverse(Tf_bl2,-15)
        setpath(newq)
        setpath([0,0,0,0,0,-15])
        # # setpath([-1.3,0,-1,0,0,0])
        # # q_reach=deepcopy(q_drop)
        # # q_release[]=
        # setpath(q_drop)
        # q_release=deepcopy(q_drop)
        # q_release[-1]=30
        # setpath(q_release)

        # Going towards goal position
        TgA, TgB = horizontalAngle(np.array([goalPos[0], goalPos[1], goalPos[2] + currStackDyn * 20.0]), color, [0.0, 120.0], angled=True)
        qUp = inverse(TgB, -15)
        
        setpath(qUp)
        newq = inverse(TgA, 0)
        setpath(newq)

        # Releasing the block at the position
        newq[-1] = 30.0
        setpath(newq)
        newq = inverse(TgB, 30)
        setpath(newq)

        # TODO Raise the bot in some way
        newq[1] -= 0.2
        setpath(newq)
        currStackDyn += 1
    
        setpath([0,0,0,0,0,0])

    platform1,platform2,goalPlatform_stat=plat_blocks(staticBlocks)

    if color == "red":
        goalPos = np.array([-75.0, -475.0, 50.5]) #[-110.0, -510.0, 50.5])
    else:
        goalPos = np.array([75.0, 475.0, 50.5]) # [100.0, 450.0, 50.5]
    
    prevGoalLen = len(goalPlatform_stat) 
    currStack = 0

    #for i in range(0, len(platform2)):
    while len(platform2) > 0:
        # print("PLATFORM 2 - Block " + str(i))
        TfA, TfB = calcNewQ4(q, pose[platform2.pop(0)], color, [10.0, 60.0])
        print(TfB)
        print(TfA)
        
        # approaching first object in platform 2
        newq = inverse(TfB, 30)
        setpath(newq)

        # lower to pick up block from platform 2
        newq = inverse(TfA, 30)
        setpath(newq)

        # grasp
        newq = inverse(TfA, 0)
        setpath(newq)

        # pick up 120mm above or side
        newq = inverse(TfB, 0)
        setpath(newq)

        # Going towards goal position
        TgA, TgB = horizontalAngle(np.array([goalPos[0], goalPos[1], goalPos[2] + currStack * 20.0]), color, [0.0, 120.0], angled=True)
        qUp = inverse(TgB, 0)
        tempq0 = qUp[0]
        qUp[0] = newq[0]
        qUp[1] -= 0.15
        setpath(qUp)
        qUp[0] = tempq0
        qUp[1] += 0.15
        setpath(qUp)
        newq = inverse(TgA, 0)
        setpath(newq)

        # Releasing the block at the position
        newq[-1] = 30.0
        setpath(newq)
        newq = inverse(TgB, 30)
        setpath(newq)

        # TODO Raise the bot in some way
        newq[1] -= 0.2
        setpath(newq)

        q = newq
        platform1,platform2,goalPlatform_stat = plat_blocks(staticBlocks)
        if prevGoalLen > len(goalPlatform_stat):
            prevGoalLen = len(goalPlatform_stat)
            currStack += 1
    print("end of platform 2")

    

    # Getting right to the platform 1
    # for i in range(0, len(platform1)):
    while len(platform1) > 0:
        print("PLATFORM 1 - Block " + str(i))
        TfA, TfB = calcNewQ4(q, pose[platform1.pop(0)], color, [10.0, 60.0])
        print(TfB)
        print(TfA)
        
        # approaching first object in platform 1
        newq = inverse(TfB, 30)
        setpath(newq)

        # lower to pick up block from platform 1
        newq = inverse(TfA, 30)
        setpath(newq)

        # grasp
        newq = inverse(TfA, 0)
        setpath(newq)

        # pick up 120mm above or side
        newq = inverse(TfB, 0)
        setpath(newq)

        # Going towards goal position
        TgA, TgB = horizontalAngle(np.array([goalPos[0], goalPos[1], goalPos[2] + currStack * 20.0]), color, [0.0, 120.0], angled=True)
        qUp = inverse(TgB, 0)
        tempq0 = qUp[0]
        qUp[0] = newq[0]
        qUp[1] -= 0.15
        setpath(qUp)
        qUp[0] = tempq0
        qUp[1] += 0.15
        setpath(qUp)
        newq = inverse(TgA, 0)
        setpath(newq)

        # Releasing the block at the position
        newq[-1] = 30.0
        setpath(newq)
        newq = inverse(TgB, 30)
        setpath(newq)

        # TODO Raise the bot in some way
        newq[1] -= 0.2
        setpath(newq)
        
        q = newq
        platform1,platform2,goalPlatform_stat = plat_blocks(staticBlocks)
        if prevGoalLen > len(goalPlatform_stat):
            prevGoalLen = len(goalPlatform_stat)
            currStack += 1
    print("end of platform 1")

    lynx.stop()
