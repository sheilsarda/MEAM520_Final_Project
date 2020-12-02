import numpy as np
import math

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

        isPos_val = 0
        lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)


        x = 0.0
        y = 0.0
        z = 0.0
        d1 = 76.2                      # Distance between joint 1 and joint 2
        a2 = 146.05                    # Distance between joint 2 and joint 3
        a3 = 187.325                   # Distance between joint 3 and joint 4
        d4 = 34                        # Distance between joint 4 and joint 5
        d5 = 68 
        q1 = np.zeros((1,6))
        q2 = np.zeros((1,6))
        q3 = np.zeros((1,6))
        q4 = np.zeros((1,6))
        T = T0e
        
        xyz = np.array((T[:3,3]))
        
       
        for i in range(xyz.shape[0]):
            if i == 0:
                #location of the end effector x,y,z.
                x = xyz[i]
            elif i == 1:
                y = xyz[i]
            elif i == 2:
                z = xyz[i]

        r3 = np.array((T[:3,2]))
      
      
        for a in range(r3.shape[0]):
            if a == 0:
                #the location of the wrist centers
                xc = x - (d5)*(r3[a])
                r13 = r3[a]
            elif a == 1:
                yc = y - (d5)*(r3[a])
                r23 = r3[a]
            elif a == 2:
                zc = z - (d5)*(r3[a])
                r33 = r3[a]

        r2 = np.array((T[:3,1]))
       
        for s in range(r2.shape[0]):
            if s == 0:
                r12 = r2[s]
            elif s == 1:
                r22 = r2[s]
            else:
                r32 = r2[s] 

       
        r1 = np.array((T[:3,0]))
       
        for b in range(r1.shape[0]):
            if b == 0:
                r11 = r1[b]
            elif b == 1:
                r21 = r1[b]
            else:
                r31 = r1[b]

     


        #Case where position is outside the reachable workspace
        q_det = xc**2 + yc**2 + (zc-d1)**2 - (a2+a3)**2
        if q_det > 0:
            q = []
            isPos = 0


        #Case where position is reachable but orientation is infeasible
        else: 
        
            n = np.array([-np.sin(np.arctan2(yc,xc)),np.cos(np.arctan2(yc,xc)),0])
            print("n: ", n)
            if np.dot(r3, n) != 0:
               
                b = r3 - (np.dot(r3,n))*n
                ze_new = b / np.linalg.norm(b)

                perpn = np.array([1/np.sin(np.arctan2(yc,xc)),-1/np.cos(np.arctan2(yc,xc)), 0 ])
                c = r2 - (np.dot(r2, perpn))*(perpn)
                ye_new = c / np.linalg.norm(c)
                
                xe_new = np.cross(ye_new,ze_new)

                r1 = xe_new
                r2 = ye_new
                r3 = ze_new

                for a in range(r3.shape[0]):
                    if a == 0:
                        xc = x - (d5)*(r3[a])
                        r13 = r3[a]
                    elif a == 1:
                        yc = y - (d5)*(r3[a])
                        r23 = r3[a]
                    else:
                        zc = z - (d5)*(r3[a])
                        r33 = r3[a]

                for s in range(r2.shape[0]):
                    if s == 0:
                        r12 = r2[s]
                    elif s == 1:
                        r22 = r2[s]
                    else:
                        r32 = r2[s] 

                for b in range(r1.shape[0]):
                    if b == 0:
                        r11 = r1[b]
                    elif b == 1:
                        r21 = r1[b]
                    else:
                        r31 = r1[b]

                isPos_val = 1
            

            #calculations for theta1
            theta1 = np.arctan2(yc,xc)

           
        
            #calculations for theta3
            theta3 = (-math.pi/2) - np.arccos((xc**2+yc**2+(zc-d1)**2-a2**2-a3**2)/(2*a2*a3))
            #to check if theta3 is outside pi and so to adjust
            if theta3 >= math.pi:
                theta3 = theta3 - math.pi
            elif theta3 <= -math.pi:
                theta3 = theta3 + math.pi
            else:
                theta3 = theta3
            

            c1 = math.cos(theta1)
            
            s1 = math.sin(theta1)
            

            #calculations for theta2
            theta2_x1 = (zc-d1)
            theta2_x2  = math.sqrt(xc**2+yc**2)
            theta2_y1 = (a3*math.sin((-math.pi/2)-theta3))
            theta2_y2 = (a2+ a3*math.cos((-math.pi/2)- theta3))
            theta2 = (math.pi/2)- np.arctan2(theta2_x1, theta2_x2) + np.arctan2(theta2_y1,theta2_y2)
           
            
            #adjust theta3 again 
            theta3 = theta3 + math.pi
            if theta3 >= math.pi:
                theta3 = theta3 - math.pi
            elif theta3 <= -math.pi:
                theta3 = theta3 + math.pi
            else:
                theta3 = theta3
            

            c3 = math.cos(theta3)
            s3 = math.sin(theta3)

            c2 = math.cos(theta2)
            s2 = math.sin(theta2)

            #calculations for theta4
        
            theta4_cos = ((-c1*c2*s3)+(c1*s2*c3))*r13 + ((-s1*c2*s3)+(s1*s2*c3))*r23 + ((-s2*s3)+(c2*c3))*r33 
            theta4_sin = (((c1*c2*c3)-(c1*s2*s3))*r13 + ((s1*c2*c3)-(s1*s2*s3))*r23 + ((c3*s2)-(c2*s3))*r33)
            theta4 = np.arctan2(theta4_sin,theta4_cos)
                  
            #calculations for theta5
            theta5_sin = ((s1*r11)+(c1*r21))
            theta5_cos = ((s1*r12)+(c1*r22))

            theta5 = np.arctan2(theta5_sin,theta5_cos) #plus math.pi was added to make it 0
            
            if theta5 >= math.pi/2:
                theta5 = theta5 - math.pi/2   
            elif theta5 <= -math.pi/2:
                theta5 = theta5 + math.pi/2
            else:
                theta5 = theta5 

            #adjusting thetas for wrap around

            if theta1 >= math.pi:
                theta1 = theta1 - math.pi
            elif theta1 <= -math.pi:
                theta1 = theta1 + math.pi
            else:
                theta1 = theta1 
           
            if theta4 >= math.pi/2:
                theta4 = theta4 - math.pi/2
            elif theta4 <= -math.pi/2:
                theta4 = theta4 + math.pi/2
            else:
                theta4 = theta4
            
            if theta5 >= math.pi/2:
                theta5 = theta5 - math.pi/2   
            elif theta5 <= -math.pi/2:
                theta5 = theta5 + math.pi/2
            else:
                theta5 = theta5 

            if theta3 >= math.pi:
                theta3 = theta3 - math.pi
            elif theta3 <= -math.pi:
                theta3 = theta3 + math.pi
            else:
                theta3 = theta3 

            #theta3 = theta3 + math.pi



            q1[0,0] = theta1 
            q1[0,1] = theta2 
            q1[0,2] = theta3  
            q1[0,3] = theta4 #+ math.pi/2
            q1[0,4] = theta5
            
            print("q1")
            print(q1)

            if (q1[0,0] < lowerLim[0,0] or q1[0,0] > upperLim[0,0]):
                q1=np.zeros([1,6])
            elif(q1[0,1] < lowerLim[0,1] or q1[0,1] > upperLim[0,1]):
                q1=np.zeros([1,6])
            elif(q1[0,2] < lowerLim[0,2] or q1[0,2] > upperLim[0,2]):
                q1=np.zeros([1,6])
            elif(q1[0,3] < lowerLim[0,3] or q1[0,3] > upperLim[0,3]):
                q1=np.zeros([1,6])
            elif(q1[0,4] < lowerLim[0,4] or q1[0,4] > upperLim[0,4]):
                q1=np.zeros([1,6])
            elif(q1[0,5] < lowerLim[0,5] or q1[0,5] > upperLim[0,5]):
                q1=np.zeros([1,6])
            else:
                q = q1

            q2[0,0] = -1*(theta1)
            q2[0,1] = -1*(theta2) 
            q2[0,2] = -1*(theta3) #initially it was theta3 + pi
            q2[0,3] = -1*(theta4) # it was theta4 + pi/2
            q2[0,4] = -1*(theta5)

            print("q2")
            print(q2)

            if (q2[0,0] < lowerLim[0,0] or q2[0,0] > upperLim[0,0]):
                q2=np.zeros([1,6])
            elif(q2[0,1] < lowerLim[0,1] or q2[0,1] > upperLim[0,1]):
                q2=np.zeros([1,6])
            elif(q2[0,2] < lowerLim[0,2] or q2[0,2] > upperLim[0,2]):
                q2=np.zeros([1,6])
            elif(q2[0,3] < lowerLim[0,3] or q2[0,3] > upperLim[0,3]):
                q2=np.zeros([1,6])
            elif(q2[0,4] < lowerLim[0,4] or q2[0,4] > upperLim[0,4]):
                q2=np.zeros([1,6])
            elif(q2[0,5] < lowerLim[0,5] or q2[0,5] > upperLim[0,5]):
                q2=np.zeros([1,6])
            else:
                q = q2

            q3[0,0] = theta1 + (math.pi)
            q3[0,1] = theta2 + (math.pi)
            q3[0,2] = (theta3) + (math.pi)
            q3[0,3] = (theta4) + (math.pi) 
            q3[0,4] = (theta5) + (math.pi)


            print("q3")
            print(q3)
            if (q3[0,0] < lowerLim[0,0] or q3[0,0] > upperLim[0,0]):
                q3= np.zeros([1,6])
            elif(q3[0,1] < lowerLim[0,1] or q3[0,1] > upperLim[0,1]):
                q3= np.zeros([1,6])
            elif(q3[0,2] < lowerLim[0,2] or q3[0,2] > upperLim[0,2]):
                q3= np.zeros([1,6])
            elif(q3[0,3] < lowerLim[0,3] or q3[0,3] > upperLim[0,3]):
                q3= np.zeros([1,6])
            elif(q3[0,4] < lowerLim[0,4] or q3[0,4] > upperLim[0,4]):
                q3= np.zeros([1,6])
            elif(q3[0,5] < lowerLim[0,5] or q3[0,5] > upperLim[0,5]):
                q3= np.zeros([1,6])
            else:
                q = q3

            q4[0,0] = theta1 - (math.pi)
            q4[0,1] = (theta2) - (math.pi)
            q4[0,2] = (theta3) - (math.pi)
            q4[0,3] = (theta4) - (math.pi) 
            q4[0,4] = (theta5) - (math.pi)

            print("q4")
            print(q4)
            if (q4[0,0] < lowerLim[0,0] or q4[0,0] > upperLim[0,0]):
                q4= np.zeros([1,6])
            elif(q4[0,1] < lowerLim[0,1] or q4[0,1] > upperLim[0,1]):
                q4= np.zeros([1,6])
            elif(q4[0,2] < lowerLim[0,2] or q4[0,2] > upperLim[0,2]):
                q4= np.zeros([1,6])
            elif(q4[0,3] < lowerLim[0,3] or q4[0,3] > upperLim[0,3]):
                q4= np.zeros([1,6])
            elif(q4[0,4] < lowerLim[0,4] or q4[0,4] > upperLim[0,4]):
                q4= np.zeros([1,6])
            elif(q4[0,5] < lowerLim[0,5] or q4[0,5] > upperLim[0,5]):
                q4= np.zeros([1,6])
            else:
                q = q4
            
            
            if isPos_val == 1:
                isPos = 0
            else:
                isPos = 1

        print(q)

        # Your code ends here
        return q, isPos