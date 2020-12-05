import numpy as np
import math
import copy

"""
This is the dimension of the Lynx Robot stated as global variable

"""
# Lynx ADL5 constants in mm
d1 = 76.2                      # Distance between joint 1 and joint 2
a2 = 146.05                    # Distance between joint 2 and joint 3
a3 = 187.325                   # Distance between joint 3 and joint 4
d4 = 34                        # Distance between joint 4 and joint 5
d5 = 68                        # Distance between joint 4 and end effector

# Joint limits
lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

def inverse(T0e,opening):
    """
    INPUT:
    T - 4 x 4 homogeneous transformation matrix, representing
       the end effector frame expressed in the base (0) frame
       (position in mm)

    OUTPUT:
    q - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad)
       which are required for the Lynx robot to reach the given
       transformation matrix T. Each row represents a single
       solution to the IK problem. If the transform is
       infeasible, q should be all zeros.
    isPos - a boolean set to true if the provided
         transformation T is achievable by the Lynx robot as given,
         ignoring joint limits
    """
    isPos = 1
    q = np.zeros((1, 6))
    # Your code starts from here

    # End effector position
    e_pos = T0e[0:3,3]

    # Wrist position
    wrist_pos = e_pos - d5*T0e[0:3,2]
    print("wrist position: ", wrist_pos)
    print("end_effector position: ", e_pos)

    # Theta_1
    # First and Fourth quadrant
    theta1 = np.arctan(wrist_pos[1] / wrist_pos[0])

    # Second quadrant and Third quadrant yield the same solutions
    # as First and Fourth

    # Check for feasible orientation of end-effector
    theta_check = np.arctan(e_pos[1] / e_pos[0])
    if(abs(theta_check-theta1) <  0.000001):
        isPos = True
    else: isPos = False

    # Theta_3-- first solution
    theta3 = -np.pi-(-np.pi/2 -np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - d1)**2 - a2**2 - a3**2) / (2*a2*a3)))
#     print("theta3 preprocessed:",theta3)
    if theta3>2*np.pi:
        theta3=theta3-2*np.pi
    elif theta3<-2*np.pi:
        theta3=theta3+2*np.pi
    elif theta3>upperLim[0,2]:
        theta3=theta3-np.pi
    elif theta3<lowerLim[0,2]:
        theta3=theta3+np.pi

    theta2 =  np.pi/2 - np.arctan2((wrist_pos[2] - d1) , (np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2))) + np.arctan2((a3*np.sin(-np.pi/2 - theta3)) , (a2 + a3*np.cos(-np.pi/2 - theta3)))
    wpx=np.cos(theta1)*(a3*np.cos(theta2+theta3)+a2*np.sin(theta2))
    wpy=np.sin(theta1)*(a3*np.cos(theta2+theta3)+a2*np.sin(theta2))
    wpz=a2*np.cos(theta2)+d1-a3*np.sin(theta2+theta3)
    print("simulated wrist position X:",wpx)
    print("simulated wrist position Y:",wpy)
    print("simulated wrist position Z:",wpz)

    # Rotation from frame 0 to end effector
    R = np.zeros((3, 3))
    for i in range(0, 3):
        for j in range(0, 3):
            R[i,j] = T0e[i,j]

    # Rotation matrix from frame 0 to frame 1
    R_01 = np.zeros((3, 3))
    R_01[2,1]= -1
    R_01[1,2]= np.cos(theta1)
    R_01[1,0]= np.sin(theta1)
    R_01[0,2]= -np.sin(theta1)
    R_01[0,0]= np.cos(theta1)

    t4sol=[]
    t5sol=[]

    # Rotation matrix from frame 1 to frame 2
    R_12 = np.zeros((3, 3))
    R_12[2,2]= 1
    R_12[1,1]= np.cos(theta2-np.pi/2)
    R_12[1,0]= np.sin(theta2-np.pi/2)
    R_12[0,1]= -np.sin(theta2-np.pi/2)
    R_12[0,0]= np.cos(theta2-np.pi/2)

    # Rotation matrix from frame 2 to frame 3
    R_23 = np.zeros((3, 3))
    R_23[2,2]= 1
    R_23[1,1]= np.cos(theta3+np.pi/2)
    R_23[1,0]= np.sin(theta3+np.pi/2)
    R_23[0,1]= -np.sin(theta3+np.pi/2)
    R_23[0,0]= np.cos(theta3+np.pi/2)

    R_03 = np.matmul(np.matmul(R_01, R_12), R_23)
    R_3e = np.matmul(np.transpose(R_03) , R)

    theta5 = np.arctan2(-R_3e[2,0] , -R_3e[2,1])
    t5sol.append(theta5)
    theta4 = np.arctan2(-R_3e[0,2] , R_3e[1,2]) + np.pi/2
    t4sol.append(theta4)


#     print("theta 1:",theta1)
#     print("theta 2:",theta)
#     print("theta 3:",t3sol)
#     print("theta 4:",t4sol)
#     print("theta 5:",t5sol)

    finalc=np.zeros(6)
    finalc[0]=theta1
    finalc[1]=theta2
    finalc[2]=theta3
    finalc[3]=theta4
    finalc[4]=theta5
    finalc[5]=opening
    print("coordinates:",finalc)
    return finalc
