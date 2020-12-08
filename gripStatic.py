import numpy as np
import math
from calculateFK import calculateFK

def signOf(x):
    """
    Retrieve the sign of scalar input x, considers 0 positive
    """
    return -1.0 if x < 0.0 else 1.0

def normalize(vec):
    """
    Slightly redundant function to normalize vectors (most are already good here)
        Parameter: vec - vector to be normalized
        Output:    vec - but normalized
    """
    mag = np.linalg.norm(vec)
    return vec if (mag == 0.0 or mag == 1.0) else (vec / mag)

def angularDifference(vec1, vec2, axis):
    """
    Get the angle between two vectors and the direction to get from vec1 to vec2.
    Assumes rotation along X, Y, |OR| Z
        Parameters:
            vec1,      1x3 the starting vector
            vec2,      1x3 the vector to compare to
            axis,      scalar, axis to prioritize for cross product
        Output:
            angle      scalar the magnitude of rotation between the vecs (radians)
            sign       scalar - either -1 or 1 denoting the direction of rotation
    """
    # Get the angle of rotation using law of cosin
    angle = abs(np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0)))

    # Get the direction of rotation using the cross product result
    a = (axis + 1) % 3
    b = (axis + 2) % 3
    sign = signOf(vec1[a] * vec2[b] - vec1[b] * vec2[a])

    return angle, sign

def checkCubeAngle(cubeR, rotAxis=2, jointR=np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])):
    """
    Get the angle between a current frame and the world-space rotation of a cube
        Parameters:
            cubeR,     4x4 matrix defining the current cube pos and orientation
            rotAxis,   int, the axis of rotation [0, 1, or 2] = [x, y, or z], default z
            jointR     3x3 matrix defining the space to compare it to, identity matrix by default
                       assume rotAxis (default z-axis) must be facing upwards or downwards
        Output:
            rotZ       float denoting (default = world space) Z angle in radians (mod pi/2)
    """
    # Check which cube axis points up (or down), by checking the maximum z value
    cubeAxis = 0
    maxZ = 0.0

    for n in range(3):
        currVal = abs(cubeR[rotAxis, n])
        if currVal > maxZ:
            maxZ = currVal
            cubeAxis = n

    # If the cube does not have an axis near [0.0, 0.0, +/- 1.0], we know that
    # it has been Jostled and is balancing at an angle to the ground. Notify User
    if maxZ <= 0.99996:
        print("This cube is not flush to the ground.")
        # As of now, we continue to find the best option from above
        # OR - TODO - expand to work in all directions

    # Now get the angle of rotation using law of cosin, comparing non-axis-of-rotation axes
    jointComp = normalize(jointR[:, (rotAxis + 1) % 3])
    cubeComp  = normalize(cubeR[:3, (cubeAxis + 1) % 3])
    angle, direction = angularDifference(jointComp, cubeComp, 2)

    # Use Joint 4's dir to help determine the direction of rotation
    sign = signOf(jointR[rotAxis, rotAxis])

    # Minimize adjustments to be less than 45 degrees
    angle = angle % (np.pi / 2)
    if (angle > np.pi / 4.0): 
        angle = (angle - np.pi / 2.0)

    return sign * direction * angle

def zRot(angle):
    """
    Z-Axis Rotation
        Parameter: angle - scalar (in radians)
        Output:    4x4 Z-Rotation matrx based on the input
    """
    s = math.sin(angle)
    c = math.cos(angle)
    return np.array([[c,   -s,  0.0, 0.0],
                     [s,    c,  0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])

def zRot3x3(angle):
    """
    Z-Axis Rotation
        Parameter: angle - scalar (in radians)
        Output:    3x3 Z-Rotation matrx based on the input
    """
    s = math.sin(angle)
    c = math.cos(angle)
    return np.array([[c,   -s,  0.0],
                     [s,    c,  0.0],
                     [0.0, 0.0, 1.0]])

'''def xRot3x3(angle):
    """
    Z-Axis Rotation
        Parameter: angle - scalar (in radians)
        Output:    3x3 Z-Rotation matrx based on the input
    """
    s = math.sin(angle)
    c = math.cos(angle)
    return np.array([[1.0, 0.0, 0.0]
                     [0.0,  c,   -s],
                     [0.0,  s,    c]])'''

def clampToLims(val, adjust, lowerLim, upperLim):
    """
    Check if a scalar is within a certain range, shifts it by a given adjustment value if not
        Parameters:
            val,        scalar - value to clamp
            adjust,     scalar - amount to shift the value to return to the range (min: pi/2)
            lowerLim,   scalar - the lowest (exclusive) that our value can be
            upperLim,   scalar - the highest (exclusive) that our value can be
        Output:
            val,        scalar - possibly adjusted by 'amount'
    """
    if val >= upperLim: return (val - adjust)
    if val <= lowerLim: return (val + adjust)
    return val

def inRange(FK, origin, p2, blockDir):
    # print(origin)
    # print(p2)
    # print(blockDir)
    p1 = origin.copy()
    p1[2] += FK.L1

    fromAbove = p2.copy()
    fromAbove[2] += (FK.L4 + FK.L5 + 10.0)
    top = (np.linalg.norm(fromAbove - p1) < (FK.L2 + FK.L3 - 10.0))
    # print("Top - p1: " + str(p1) + ", p2: " + str(fromAbove))
    
    p1 += (FK.L2 * math.sin(FK.upperLim[0, 1]) * blockDir)
    p1[2] += (FK.L2 * math.cos(FK.upperLim[0, 1]))

    fromSide = p2 - ((FK.L4 + FK.L5 + 10.0) * blockDir)
    side = (np.linalg.norm(fromSide - p1) < FK.L3)
    # print("Side - p1: " + str(p1) + ", p2: " + str(fromSide))

    if not top and not side:
        return top, side, normalize(p2 - p1)

    return top, side, blockDir

# def calcNewQ4(q, pose, color, a):
#     """
#     Calculates the new joint(4) position to align robot end effector with a cube
#         Parameters:
#             q,       1x6 vector of joint inputs [q0,q1,q2,q3,q4,lg]
#                         MAYBE take in FK's output if that's been calculated outside
#             pose     4x4 matrix defining the current cube pos and orientation
#             color    string, either blue or red
#         Output:
#             4x4 matrix of desired T0e to align robot 30 units above a cube
#     """
#     # REMINDER? - if IK shows this rotation can not be reached, should prob just make
#     # q4 = -np.pi / 2.0 : least liklihood of collision as long as pos can be reached

#      # Test Prints
#     '''print("Cube:")
#     print(pose)'''

#     # FK only used for limits, maybe hardcode it instead
#     FK = calculateFK()

#     # Get projected end effector rotation, using current q4
#     if color == "red":
#         cons = 200
#         origin = np.array([-200.0, -200.0, 0.0])
#         startDir = np.array([1.0, 0.0, 0.0])
#         effectorDown = np.array([[1.0,  0.0,  0.0],
#                                  [0.0, -1.0,  0.0],
#                                  [0.0,  0.0, -1.0]])
#     else:
#         cons = -200
#         origin = np.array([200.0, 200.0, 0.0])
#         startDir = np.array([-1.0, 0.0, 0.0])
#         effectorDown = np.array([[-1.0, 0.0,  0.0],
#                                  [0.0,  1.0,  0.0],
#                                  [0.0,  0.0, -1.0]])
#     # - getting Frame 5 w.r.t. downwards-facing Frame 4 and q0 = 0
#     np.dot(effectorDown, zRot3x3(q[4]), out=effectorDown)
#     # print(effectorDown)

#     # - getting rotation that angles the robot to the block
#     blockO = np.array([pose[0, 3], pose[1, 3], 0.0])
#     blockDir = normalize(np.subtract(blockO, origin))

#     angle, direction = angularDifference(startDir, blockDir, 2)
#     roboRot = zRot3x3(direction * angle)

#     # - and combine to get projectect T pre-consideration of the block
#     T = roboRot.dot(effectorDown)
#     print("origin " + str(origin))

#     top, side, ref = inRange(FK, origin, np.copy(pose[:3, 3]), blockDir)
#     # print("Top: " + str(top) + ",  Side: " + str(side))
#     if not top:
#         y = np.array([0.0, 0.0, 1.0])
        
#         if not side:
#             x = normalize(np.cross(y, ref))
#             y = normalize(np.cross(ref, x))
#             print("Diagonal")
#             print("origin " + str(origin))
#             print("block " + str(blockO))
#             print("blockD " + str(blockDir))
#             print("ref " + str(ref))

#             pAngle = pose[:3, 3] - a * ref
#             pAngle = pose[:3, 3] - a * blockDir
#             p = np.array([pAngle[0] + cons, pAngle[1] + cons, pAngle[2]])
#             if color == "blue":
#                 p = zRot3x3(np.pi).dot(p)
#             return np.array([[x[0], y[0], ref[0], p[0]],
#                              [x[1], y[1], ref[1], p[1]],
#                              [x[2], y[2], ref[2], p[2]],
#                              [0.0,  0.0,     0.0, 1.0]])
        
#         print("Side")
#         x = normalize(np.cross(y, blockDir))
        
#         pAngle = pose[:3, 3] - a * blockDir
#         p = np.array([pAngle[0] + cons, pAngle[1] + cons, pose[2, 3]])
#         if color == "blue":
#             p = zRot3x3(np.pi).dot(p)
#         return np.array([[x[0],  y[0], blockDir[0],  p[0]],
#                          [x[1],  y[1], blockDir[1],  p[1]],
#                          [x[2],  y[2], blockDir[2],  p[2]],
#                          [0.0,    0.0,     0.0,        1.0]])

#     print("Top")


#     # Use projected-T as a comparison to calc the smallest necessary motion
#     #   if we dont care about smallest motion, just use original effectorDown instead
#     dq4 = checkCubeAngle(pose, jointR=T[:3, :3])
#     # print(dq4)

#     q4 = clampToLims(q[4] + dq4, np.pi / 2.0, FK.lowerLim[0, 4], FK.upperLim[0, 4])

#     # SIDEBONUS: Check new transformation of end effector
#     Te = T.dot(zRot3x3(q4 - q[4]))
#     #print(T2)

#     # Want to rotate so that the grips do not overlap the cube's white face...
#     dotX = np.dot(Te[:, 0], pose[:3, 2])

#     # So if the cube's z axis is parallel to the end effector's x axis, shift by 90 degrees
#     if abs(dotX) > 0.8:
#         q4 = clampToLims(q4 - np.pi / 2.0, np.pi, FK.lowerLim[0, 4], FK.upperLim[0, 4])
#         Te = T.dot(zRot3x3(q4 - q[4]))
#         # print("yeah")

#     # Test Prints
#     '''print("Robot")
#     print(Te)'''
#     p = np.array([pose[0,3]+cons, pose[1,3] + cons, pose[2, 3] + a])
#     if color == "blue":
#         p = zRot3x3(np.pi).dot(p)
#     # return newQ
    
#     return np.array([[Te[0,0], Te[0,1], Te[0,2], p[0]],
#                      [Te[1,0], Te[1,1], Te[1,2], p[1]],
#                      [Te[2,0], Te[2,1], Te[2,2], p[2]],
#                      [0.0,       0.0,     0.0,     1.0]])

def horizontalAngle(location, color):
    if color == "red":
        cons = 200
        origin = np.array([-200.0, -200.0, 0.0])
    else:
        cons = -200
        rotFix = zRot(np.pi)
        origin = np.array([200.0, 200.0, 0.0])

    z = normalize(np.subtract(np.array([location[0], location[1], 0.0]), origin))
    y = np.array([0.0, 0.0, 1.0])
    x = normalize(np.cross(y, z))

    TFinal = np.array([[x[0],  y[0], z[0],  location[0] + cons],
                       [x[1],  y[1], z[1],  location[1] + cons],
                       [x[2],  y[2], z[2],  location[2]],
                       [0.0,    0.0,  0.0,       1.0]])

    if color == "red":
        return TFinal
    else:
        return rotFix.dot(TFinal)

# Option 2
def calcNewQ4(q, pose, color, a):
    """
    Calculates the new joint(4) position to align robot end effector with a cube
        Parameters:
            q,       1x6 vector of joint inputs [q0,q1,q2,q3,q4,lg]
                        MAYBE take in FK's output if that's been calculated outside
            pose     4x4 matrix defining the current cube pos and orientation
            color    string, either blue or red
        Output:
            4x4 matrix of desired T0e to align robot 30 units above a cube
    """
    # REMINDER? - if IK shows this rotation can not be reached, should prob just make
    # q4 = -np.pi / 2.0 : least liklihood of collision as long as pos can be reached

     # Test Prints
    '''print("Cube:")
    print(pose)'''

    # FK only used for limits, maybe hardcode it instead
    FK = calculateFK()

    # Get projected end effector rotation, using current q4
    if color == "red":
        cons = 200
        origin = np.array([-200.0, -200.0, 0.0])
        startDir = np.array([1.0, 0.0, 0.0])
        effectorDown = np.array([[1.0,  0.0,  0.0],
                                 [0.0, -1.0,  0.0],
                                 [0.0,  0.0, -1.0]])
    else:
        cons = -200
        rotFix = zRot(np.pi)
        origin = np.array([200.0, 200.0, 0.0])
        startDir = np.array([-1.0, 0.0, 0.0])
        effectorDown = np.array([[-1.0, 0.0,  0.0],
                                 [0.0,  1.0,  0.0],
                                 [0.0,  0.0, -1.0]])
    # - getting Frame 5 w.r.t. downwards-facing Frame 4 and q0 = 0
    np.dot(effectorDown, zRot3x3(q[4]), out=effectorDown)
    # print(effectorDown)

    # - getting rotation that angles the robot to the block
    blockO = np.array([pose[0, 3], pose[1, 3], 0.0])
    blockDir = normalize(np.subtract(blockO, origin))

    angle, direction = angularDifference(startDir, blockDir, 2)
    roboRot = zRot3x3(direction * angle)

    # - and combine to get projectect T pre-consideration of the block
    T = roboRot.dot(effectorDown)
    print("origin " + str(origin))

    top, side, ref = inRange(FK, origin, np.copy(pose[:3, 3]), blockDir)
    # print("Top: " + str(top) + ",  Side: " + str(side))
    if not top:
        y = np.array([0.0, 0.0, 1.0])
        
        if not side:
            x = normalize(np.cross(y, ref))
            y = normalize(np.cross(ref, x))
            print("Diagonal")
            print("origin " + str(origin))
            print("block " + str(blockO))
            print("blockD " + str(blockDir))
            print("ref " + str(ref))

            pAngle = pose[:3, 3] - a * ref
            TFinal = np.array([[x[0], y[0], ref[0], pAngle[0] + cons],
                               [x[1], y[1], ref[1], pAngle[1] + cons],
                               [x[2], y[2], ref[2],    pAngle[2]],
                               [0.0,  0.0,     0.0,       1.0]])

            if color == "red":
                return TFinal
            else:
                return rotFix.dot(TFinal)
        
        print("Side")
        x = normalize(np.cross(y, blockDir))
        
        pAngle = pose[:3, 3] - a * blockDir
        TFinal = np.array([[x[0],  y[0], blockDir[0],  pAngle[0] + cons],
                           [x[1],  y[1], blockDir[1],  pAngle[1] + cons],
                           [x[2],  y[2], blockDir[2],     pose[2, 3]],
                           [0.0,    0.0,     0.0,            1.0]])

        if color == "red":
            return TFinal
        else:
            return rotFix.dot(TFinal)

    print("Top")


    # Use projected-T as a comparison to calc the smallest necessary motion
    #   if we dont care about smallest motion, just use original effectorDown instead
    dq4 = checkCubeAngle(pose, jointR=T[:3, :3])
    # print(dq4)

    q4 = clampToLims(q[4] + dq4, np.pi / 2.0, FK.lowerLim[0, 4], FK.upperLim[0, 4])

    # SIDEBONUS: Check new transformation of end effector
    Te = T.dot(zRot3x3(q4 - q[4]))
    #print(T2)

    # Want to rotate so that the grips do not overlap the cube's white face...
    dotX = np.dot(Te[:, 0], pose[:3, 2])

    # So if the cube's z axis is parallel to the end effector's x axis, shift by 90 degrees
    if abs(dotX) > 0.8:
        q4 = clampToLims(q4 - np.pi / 2.0, np.pi, FK.lowerLim[0, 4], FK.upperLim[0, 4])
        Te = T.dot(zRot3x3(q4 - q[4]))
        # print("yeah")

    # Test Prints
    '''print("Robot")
    print(Te)'''
    # return newQ
    
    TFinal = np.array([[Te[0,0], Te[0,1], Te[0,2], pose[0,3] + cons],
                       [Te[1,0], Te[1,1], Te[1,2], pose[1,3] + cons],
                       [Te[2,0], Te[2,1], Te[2,2],  pose[2,3] + a],
                       [0.0,       0.0,     0.0,         1.0]])
    if color == "red":
        return TFinal
    else:
        return rotFix.dot(TFinal)

# TODO - SIDEBONUS method might only work in one direction
# TODO - another rotation adjustment needed if the newQ result clashes with another block
