"""
MATLAB version AUTHOR:
    Dr. Cynthia Sung (crsung@seas.upenn.edu)
    Modified by Gedaliah Knizhnik (knizhnik@seas.upenn.edu) 08/28/19
Python version transformed AUTHOR:
    Zichen Lao (lao0910@seas.upenn.edu) 06/05/20
"""

import numpy as np

class calculateFK():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx Dimensions in mm
        self.L1 = 76.2    # distance between joint 0 and joint 1
        self.L2 = 146.05  # distance between joint 1 and joint 2
        self.L3 = 187.325 # distance between joint 2 and joint 3
        self.L4 = 34      # distance between joint 3 and joint 4
        self.L5 = 34      # distance between joint 4 and center of gripper

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def forward(self, q):
        """
        INPUT:
        q - 1x6 vector of joint inputs [q0,q1,q2,q3,q4,lg]

        OUTPUTS:
        jointPositions - 6 x 3 matrix, where each row represents one
                  joint along the robot. Each row contains the [x,y,z]
                  coordinates of the respective joint's center (mm). For
                  consistency, the first joint should be located at
                  [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  base (0) frame
        """
        # Your code starts from here
        # Frame 1 w.r.t Frame 0
        T1 = np.array([[np.cos(q[0]), -np.sin(q[0])*np.cos(-np.pi/2), np.sin(q[0])*np.sin(-np.pi/2), 0],
                       [np.sin(q[0]), np.cos(q[0])*np.cos(-np.pi/2), -np.cos(q[0])*np.sin(-np.pi/2), 0],
                       [0, np.sin(-np.pi/2), np.cos(-np.pi/2), self.L1],
                       [0, 0, 0, 1]])

        # Frame 2 w.r.t Frame 1
        T2 = np.array([[np.cos(q[1]-(np.pi/2)), -np.sin(q[1]-(np.pi/2)), 0, self.L2*np.cos(q[1]-(np.pi/2))],
                       [np.sin(q[1]-(np.pi/2)), np.cos(q[1]-(np.pi/2)), 0, self.L2*np.sin(q[1]-(np.pi/2))],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        # Frame 3 w.r.t Frame 2
        T3 = np.array([[np.cos(q[2]+(np.pi/2)), -np.sin(q[2]+(np.pi/2)), 0, self.L3*np.cos(q[2]+(np.pi/2))],
                       [np.sin(q[2]+(np.pi/2)), np.cos(q[2]+(np.pi/2)), 0, self.L3*np.sin(q[2]+(np.pi/2))],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        # Frame 4 w.r.t Frame 3
        T4 = np.array([[np.cos(q[3]-(np.pi/2)), -np.sin(q[3]-(np.pi/2))*np.cos(-np.pi/2), np.sin(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                       [np.sin(q[3]-(np.pi/2)), np.cos(q[3]-(np.pi/2))*np.cos(-np.pi/2), -np.cos(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                       [0, np.sin(-np.pi/2), np.cos(-np.pi/2), 0],
                       [0, 0, 0, 1]])
        # Frame 5 w.r.t Frame 4
        T5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, 0],
                       [np.sin(q[4]), np.cos(q[4]), 0, 0],
                       [0, 0, 1, self.L4 + self.L5],
                       [0, 0, 0, 1]])

        x = np.empty((6, 4)).reshape((6, 4))
        zeroPos = np.array([0, 0, 0, 1]).reshape((1, 4))
        zeroPos_trans = np.transpose(zeroPos)

        # Position of First Joint (Base Revolute)
        x[0, :] = zeroPos

        # Position of Second Joint (Shoulder Revolute)
        x[1, :] = np.transpose(T1.dot(zeroPos_trans))

        # Position of Third Joint (Elbow Revolute)
        x[2, :] = np.transpose((T1.dot(T2)).dot(zeroPos_trans))

        # Position of Fourth Joint (1st Wrist)
        x[3, :] = np.transpose(((T1.dot(T2)).dot(T3)).dot(zeroPos_trans))

        # Position of Fifth Joint (2nd Wrist)
        x[4, :] = np.transpose((((T1.dot(T2)).dot(T3)).dot(T4)).dot(np.array([0, 0, self.L4, 1]).reshape((4, 1))))

        # Position of Gripper (Base of the Gripper)
        x[5, :] = np.transpose(((((T1.dot(T2)).dot(T3)).dot(T4)).dot(T5)).dot(zeroPos_trans))
        # Outputs the 6x3 of the locations of each joint in the Base Frame
        jointPositions = x[0:6,0:3]

        T0e = ((((T1.dot(T2)).dot(T3)).dot(T4)).dot(T5))
        # Your code ends here

        return jointPositions, T0e

    def forwardjoint(self, q,joint):
        # Your code starts from here
        # Frame 1 w.r.t Frame 0
        T1 = np.array([[np.cos(q[0]), -np.sin(q[0])*np.cos(-np.pi/2), np.sin(q[0])*np.sin(-np.pi/2), 0],
                       [np.sin(q[0]), np.cos(q[0])*np.cos(-np.pi/2), -np.cos(q[0])*np.sin(-np.pi/2), 0],
                       [0, np.sin(-np.pi/2), np.cos(-np.pi/2), self.L1],
                       [0, 0, 0, 1]])

        # Frame 2 w.r.t Frame 1
        T2 = np.array([[np.cos(q[1]-(np.pi/2)), -np.sin(q[1]-(np.pi/2)), 0, self.L2*np.cos(q[1]-(np.pi/2))],
                       [np.sin(q[1]-(np.pi/2)), np.cos(q[1]-(np.pi/2)), 0, self.L2*np.sin(q[1]-(np.pi/2))],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        # Frame 3 w.r.t Frame 2
        T3 = np.array([[np.cos(q[2]+(np.pi/2)), -np.sin(q[2]+(np.pi/2)), 0, self.L3*np.cos(q[2]+(np.pi/2))],
                       [np.sin(q[2]+(np.pi/2)), np.cos(q[2]+(np.pi/2)), 0, self.L3*np.sin(q[2]+(np.pi/2))],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        # Frame 4 w.r.t Frame 3
        T4 = np.array([[np.cos(q[3]-(np.pi/2)), -np.sin(q[3]-(np.pi/2))*np.cos(-np.pi/2), np.sin(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                       [np.sin(q[3]-(np.pi/2)), np.cos(q[3]-(np.pi/2))*np.cos(-np.pi/2), -np.cos(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                       [0, np.sin(-np.pi/2), np.cos(-np.pi/2), 0],
                       [0, 0, 0, 1]])
        # Frame 5 w.r.t Frame 4
        T5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, 0],
                       [np.sin(q[4]), np.cos(q[4]), 0, 0],
                       [0, 0, 1, self.L4 + self.L5],
                       [0, 0, 0, 1]])

        x = np.empty((6, 4)).reshape((6, 4))
        zeroPos = np.array([0, 0, 0, 1]).reshape((1, 4))
        zeroPos_trans = np.transpose(zeroPos)

        # Position of First Joint (Base Revolute)
        x[0, :] = zeroPos

        # Position of Second Joint (Shoulder Revolute)
        x[1, :] = np.transpose(T1.dot(zeroPos_trans))

        # Position of Third Joint (Elbow Revolute)
        x[2, :] = np.transpose((T1.dot(T2)).dot(zeroPos_trans))

        # Position of Fourth Joint (1st Wrist)
        x[3, :] = np.transpose(((T1.dot(T2)).dot(T3)).dot(zeroPos_trans))

        # Position of Fifth Joint (2nd Wrist)
        x[4, :] = np.transpose((((T1.dot(T2)).dot(T3)).dot(T4)).dot(np.array([0, 0, self.L4, 1]).reshape((4, 1))))

        # Position of Gripper (Base of the Gripper)
        x[5, :] = np.transpose(((((T1.dot(T2)).dot(T3)).dot(T4)).dot(T5)).dot(zeroPos_trans))
        # Outputs the 6x3 of the locations of each joint in the Base Frame
        jointPositions = x[0:6,0:3]

        if joint==1:
            T0i=T1
        elif joint==2:
            T0i=(T1.dot(T2))
        elif joint==3
            T0i=(T1.dot(T2)).dot(T3)
        elif joint==4:
            T0i=((T1.dot(T2)).dot(T3)).dot(T4)
        elif joint==5:
            T0i=(((T1.dot(T2)).dot(T3)).dot(T4)).dot(T5)
        # Your code ends here

        return T0i
