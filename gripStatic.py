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
    if (angle > np.pi / 4.0): angle = (angle - np.pi / 2.0)

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

def calcNewQ4(q, pose, color,a):
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
    print("Cube:")
    print(pose)

    # FK only used for limits, maybe hardcode it instead
    FK = calculateFK()

    # Get projected end effector rotation, using current q4
    if color is "red":
        origin = np.array([-200.0, -200.0, 0.0])
        startDir = np.array([1.0, 0.0, 0.0])
        effectorDown = np.array([[1.0,  0.0,  0.0],
                                 [0.0, -1.0,  0.0],
                                 [0.0,  0.0, -1.0]])
    else:
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

    # Use projected-T as a comparison to calc the smallest necessary motion
    #   if we dont care about smallest motion, just use original effectorDown instead
    dq4 = checkCubeAngle(pose, jointR=T[:3, :3])
    print(dq4)

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
        print("yeah")

    # Test Prints
    print("Robot")
    print(Te)
    if color == "red":
        cons=200
    else:
        cons=-200
    # return newQ
    return np.array([[Te[0,0], Te[0,1], Te[0,2],    pose[0,3]+cons],
                     [Te[1,0], Te[1,1], Te[1,2],    pose[1,3]+cons],
                     [Te[2,0], Te[2,1], Te[2,2], pose[2,3] + a],
                     [0.0,       0.0,     0.0,         1.0]])


# TODO - SIDEBONUS method might only work in one direction
# TODO - another rotation adjustment needed if the newQ result clashes with another block
#    -> place this check in final.py probably

# ***TESTING CODE***
'''np.set_printoptions(suppress=True)

name = ['cube_static1', 'cube_dynamic1', 'cube_dynamic5', 'cube_static8', 'cube_static6', 'cube_static3']
pose = [np.array([[   0.95533656,   -0.29551994,    0.000166  ,  -79.99892686],
       [   0.29551999,    0.9553364 ,   -0.00055237,  219.99643718],
       [   0.00000465,    0.00057676,    0.99999983,    9.58659075],
       [   0.        ,    0.        ,    0.        ,    1.        ]]),
np.array([[  0.00000023,  -0.30821291,   0.9513174 , -42.57727738],
       [  0.00000124,   0.9513174 ,   0.30821291, -39.96743901],
       [ -1.        ,   0.00000111,   0.0000006 ,  59.99982957],
       [  0.        ,   0.        ,   0.        ,   1.        ]]),
np.array([[  0.69477728,   0.71922495,  -0.00000115,  -0.00001179],
       [ -0.71922495,   0.69477728,   0.00000037,   0.00000341],
       [  0.00000107,   0.00000057,   1.        ,  59.99982944],
       [  0.        ,   0.        ,   0.        ,   1.        ]]),
np.array([[  -0.00084314,    0.09983283,    0.99500387,   20.01637106],
       [   0.00008457,    0.99500422,   -0.0998328 , -374.00164301],
       [  -0.99999964,   -0.00000003,   -0.00084737,   29.00828337],
       [   0.        ,    0.        ,    0.        ,    1.        ]]),
np.array([[   0.99499637,    0.0998297 ,    0.00403217,  -39.96167382],
       [  -0.0998324 ,    0.99500415,    0.00047469,  373.00300266],
       [  -0.00396464,   -0.00087486,    0.99999176,   29.04805336],
       [   0.        ,    0.        ,    0.        ,    1.        ]]),
np.array([[   0.95533655,   -0.29551993,    0.00019515,   90.00136458],
       [   0.29552   ,    0.95533633,   -0.00064996, -230.00453873],
       [   0.00000564,    0.0006786 ,    0.99999977,    9.59809759],
       [   0.        ,    0.        ,    0.        ,    1.        ]])]

# Simple ideal frame rotation for testing purposes: np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
print(calcNewQ4([-np.pi / 4.0, 0.0, 0.0, np.pi / 2.0, -np.pi / 2.0, 0.0], pose[3], "red"))'''
