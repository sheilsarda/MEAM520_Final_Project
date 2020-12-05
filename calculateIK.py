import numpy as np
import math
import copy

class Main():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx ADL5 constants in mm
        self.d1 = 76.2                      # Distance between joint 1 and joint 2
        self.a2 = 146.05                    # Distance between joint 2 and joint 3
        self.a3 = 187.325                   # Distance between joint 3 and joint 4
        self.d4 = 34                        # Distance between joint 4 and joint 5
        self.d5 = 68                        # Distance between joint 4 and end effector

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def inverse(self, T0e):
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
        wrist_pos = e_pos - self.d5*T0e[0:3,2]
        print("wrist position: ", wrist_pos)
        print("end_effector position: ", e_pos)

        print("Target T0e: ", T0e)

        # Theta_1
        # First and Fourth quadrant
        theta1 = np.arctan(wrist_pos[1] / wrist_pos[0])
        print('Theta_1 = ', theta1, ' rads')
        q[0,0] = np.around(theta1, 5)

        # Second quadrant and Third quadrant yield the same solutions
        # as First and Fourth

        # Check for feasible orientation of end-effector
        theta_check = np.arctan(e_pos[1] / e_pos[0])
        if(abs(theta_check-theta1) <  0.000001):
            isPos = True
        else: isPos = False

        # Hardcoded
        print("isPossible: " , isPos)


        # Theta_3-- first solution
        theta3 = np.pi/2 -np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - self.d1)**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3))
        q[0,2] = np.around(theta3, 5)
        print('Theta_3 = ', q[0,2], ' rads')

        # Theta_3-- second solution
        temp = np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - self.d1)**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3))
        if(temp != None):
            temp += -3*np.pi/2
            new_row = np.zeros((1,6))
            new_row[0,0] = theta1
            new_row[0,2] = temp
            q = np.concatenate((q, new_row))
            print('Theta_3 = ', temp, ' rads')

        # Theta_2
        for i in range(q.shape[0]):
            theta3 = q[i, 2]
            theta2 =  np.pi/2 - np.arctan2((wrist_pos[2] - self.d1) , (np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2))) + np.arctan2((self.a3*np.sin(-np.pi/2 - theta3)) , (self.a2 + self.a3*np.cos(-np.pi/2 - theta3)))
            q[i,1] = np.around(theta2, 5)
            print('Theta_2 = ', q[i,1], ' rads')

        print('q before populating every theta: ', q)

        # row index of q array for rotational matrices to reference
        i = 0

        # Rotation matrix from frame 0 to frame 1
        R_01 = np.zeros((3, 3))
        R_01[2,1]= -1
        R_01[1,2]= np.cos(q[i,0])
        R_01[1,0]= np.sin(q[i,0])
        R_01[0,2]= -np.sin(q[i,0])
        R_01[0,0]= np.cos(q[i,0])

        # Rotation matrix from frame 1 to frame 2
        R_12 = np.zeros((3, 3))
        R_12[2,2]= 1
        R_12[1,1]= np.cos(q[i,1]-np.pi/2)
        R_12[1,0]= np.sin(q[i,1]-np.pi/2)
        R_12[0,1]= -np.sin(q[i,1]-np.pi/2)
        R_12[0,0]= np.cos(q[i,1]-np.pi/2)

        # Rotation matrix from frame 2 to frame 3
        R_23 = np.zeros((3, 3))git add .
        git
        R_23[2,2]= 1
        R_23[1,1]= np.cos(q[i,2]+np.pi/2)
        R_23[1,0]= np.sin(q[i,2]+np.pi/2)
        R_23[0,1]= -np.sin(q[i,2]+np.pi/2)
        R_23[0,0]= np.cos(q[i,2]+np.pi/2)

        # Rotation matrix from frame 3 to frame 4
        R_34 = np.zeros((3, 3))
        R_34[2,1] = -1
        R_34[1,2] = np.cos(q[i,3]-np.pi/2)
        R_34[1,0] = np.sin(q[i,3]-np.pi/2)
        R_34[0,2] = -np.sin(q[i,3]-np.pi/2)
        R_34[0,0] = np.cos(q[i,3]-np.pi/2)

        # Rotation matrix from frame 4 to frame 5
        R_45 = np.zeros((3, 3))
        R_45[2,2] = 1
        R_45[1,1] = np.cos(q[i,4])
        R_45[1,0] = np.sin(q[i,4])
        R_45[0,1] = -np.sin(q[i,4])
        R_45[0,0] = np.cos(q[i,4])

        # Rotation from frame 0 to end effector
        R = np.zeros((3, 3))
        for i in range(0, 3):
            for j in range(0, 3):
                R[i,j] = T0e[i,j]

        print('R_0e = ', R)

        if(isPos):
            for row_idx in range(q.shape[0]):
                # Which row from q to reference for matrix angles
                print("Calculating theta4 and theta5 for row ", i, " of ", q.shape[0])
                i = row_idx

                R_03 = np.matmul(np.matmul(R_01, R_12), R_23)
                print("R_03 ")
                print(R_03)

                R_3e = np.matmul(np.transpose(R_03) , R)
                print("R_3e ")
                print(R_3e)

                theta5 = np.arctan2(-R_3e[2,0] , -R_3e[2,1])
                theta4 = np.arctan2(-R_3e[0,2] , R_3e[1,2]) + np.pi/2

                print('Theta_4 = ', theta4, ' rads')
                print('Theta_5 = ', theta5, ' rads')

                q[i,3] = theta4
                q[i,4] = theta5

                R_check = np.matmul(np.matmul(R_03, R_34), R_45)
                print("R_check", R_check)

            print('q after populating every theta: ', q)

        else:
            T0e_feasible = copy.deepcopy(T0e)
            # Orientation of end effector
            # is unreachable. Goal is to
            # project z-orientation

            print("end effector", e_pos)
            print("wrist ", wrist_pos)
            theta1 = np.arctan(e_pos[1] / e_pos[0])
            print("theta1", theta1)
            theta1_w = np.arctan(wrist_pos[1] / wrist_pos[0])
            print("theta1_w", theta1_w)

            normal_vec = [-np.sin(theta1), np.cos(theta1), 0]
            z_prime = T0e[0:3, 2]
            z_prime_norm = np.linalg.norm(z_prime)

            dot_value = np.dot(z_prime, normal_vec)
            normal_dot = [normal_vec[i] * dot_value for i in range(len(normal_vec))]
            norm_normal_dot = np.linalg.norm(normal_vec)**2

            z_e_1 = (copy.deepcopy(z_prime) / z_prime_norm) - (normal_dot / norm_normal_dot)
            z_e_norm = np.linalg.norm(z_e_1)
            z_e = z_e_1 / z_e_norm

            print("normal_vec", normal_vec)
            print("z_prime", z_prime)
            print("dot prod norm", norm_normal_dot)
            print("normalized dot product", (normal_dot / norm_normal_dot))
            print("z_prime", z_prime)
            print("normal_dot", normal_dot)
            print("dot_value", dot_value)

            print("normal_vec", normal_vec)
            print("Feasible z (non-normalized): ", z_e)

            print("Feasible z (1): ", z_e)

            # Start projecting y-axis
            y_prime = T0e[0:3, 1]
            print("Intended y: ", y_prime)
            normal_vec_z = z_e
            y_prime_norm = np.linalg.norm(y_prime)
            print("normal_vec_z")
            print(normal_vec_z)
            print("y_prime")
            print(y_prime)
            print("dot prod")
            print(np.dot(y_prime, normal_vec_z))
            y_e = copy.deepcopy(y_prime )
            dot_value = np.dot(y_prime, normal_vec_z)
            normal_dot = [y_prime[i] * dot_value for i in range(len(normal_vec_z))]
            y_e -= normal_dot
            y_e_norm = np.linalg.norm(y_e)
            y_e /= y_e_norm

            print("Feasible y: ", y_e)

            # Start projecting x-axis
            x_prime = T0e[0:3, 0]
            print("Intended x: ", x_prime)

            x_e = np.cross(y_e, z_e)
            x_norm = np.linalg.norm(x_e)
            x_e /= x_norm

            print("Feasible x: ", x_e)

            T0e_feasible[0:3, 0]  = x_e
            T0e_feasible[0:3, 1]  = y_e
            T0e_feasible[0:3, 2]  = z_e

            print("Feasible T0e: ", T0e_feasible)

            # Recursive call to the inverse func
            return self.inverse(T0e_feasible)

        # Account for joint limits
        exceeds_limits = [False for row in q]

        for row_idx in range(q.shape[0]):
            for theta_idx in range(q.shape[1]):
                if (q[row_idx, theta_idx] > self.upperLim[0, theta_idx]) or (q[row_idx, theta_idx] < self.lowerLim[0, theta_idx]):
                    exceeds_limits[row_idx] = (exceeds_limits[row_idx] | True)

        print("Joint limits exceeded: ", exceeds_limits)
        # q = q[np.invert(exceeds_limits),:]
        # print("q after filtering: ", q)

        # Your code ends here

        return q, isPos
