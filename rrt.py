import numpy as np
import random
from copy import deepcopy
from time import sleep
from matplotlib import pyplot as plt

from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
from calculateFK import calculateFK

def boundaryCollision(point, boundary):
    outOfBounds = False
    # print("================ Boundary Collision ================")
    # print("Point " + str(point))

    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(point[0])

    for i in range(len(startPos)):

        # Check if joint i is within the boundary
        outOfBounds |= (startPos[i][0] < boundary[0])
        outOfBounds |= (startPos[i][1] < boundary[1])
        outOfBounds |= (startPos[i][2] < boundary[2])
        outOfBounds |= (startPos[i][0] > boundary[3])
        outOfBounds |= (startPos[i][1] > boundary[4])
        outOfBounds |= (startPos[i][2] > boundary[5])

    return outOfBounds

def obstacleCollision(start, goal, obstacles):
    """
    start is a pose containing q1 -> qe
    goals is a pose containing q1 -> qe
    returns if there is a feasible straight line path
    """
    lineCollision = False
    # print("================ Obstacle Collision ================")
    # print("lineCollision is " + str(lineCollision))
    # print("Start " + str(start))
    # print("Goal " + str(goal))

    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(start[0])
    goalPos, _ = f.forward(goal[0])

    # print("StartPos " + str(startPos))
    # print("GoalPos " + str(goalPos))

    for obstacle in obstacles:
        # Iterate through every joint
        for i in range(len(startPos)):
            results = detectCollision([startPos[i]], [goalPos[i]], obstacle)
            for result in results:
                lineCollision |= result

    # print("lineCollision is " + str(lineCollision))
    return lineCollision

def findNN(points, newPoint):
    """
    Return i such that points[i] is the closest distance
    from newPoint for all the points in the array
    """

    minI = -1
    minDist = 99999
    for i in range(len(points)):
        # find distance
        a = points[i]
        b = newPoint
        dist = np.abs(sum([(a[i] - b[i])**2 for i in range(3)])**0.5)

        # update minI
        if dist < minDist:
            minI = i
            minDist = dist

    return minI

def graphTrajectory(points):

    # Data for a three-dimensional line
    zline = []
    xline = []
    yline = []
    f = calculateFK()
    for q_ix in range(0, len(points), 10):
        endXYZ = f.forward(points[q_ix])[0][-1]
        zline.append(endXYZ[2])
        yline.append(endXYZ[1])
        xline.append(endXYZ[0])


    # ax = plt.axes(projection='3d')
    #
    # ax.plot3D(xline, yline, zline, 'gray')
    # plt.show()

def deleteElement(array, left, right) :
    j = 0
    for i in range(len(array)) :
        if i <= left or i >= right :
            array[j] = array[i]
            j += 1

def postProcessing(points, obstacles):
    if len(points)<=5:
        return points
    processed = deepcopy(points)

    efficiency = 0
    i = 0
    maxIter = len(points)
    while(i < maxIter):

        a = random.randrange(1, len(processed) - 1)
        b = random.randrange(1, len(processed) - 1)

        # b is always larger than a
        if b == a: continue
        if(b < a): a, b = b, a

        if(a == b): continue
        coll = obstacleCollision([processed[a]], [processed[b]], obstacles)

        if not coll:
            efficiency += b - a -1
            # print("Eliminated %i elements"%(b-a - 1))

            y = 0
            for x in range(len(processed)) :
                if x <= a or x >= b :
                    processed[y] = processed[x]
                    y += 1
            processed = processed[:len(processed) - (b - a - 1)]
        i += 1

    # verify collision free path exists
    verificationFlag = True
    verificationFlag &= (points[0] == processed[0])
    verificationFlag &= (points[-1] == processed[-1])
    for i in range(len(processed) - 1):
        if obstacleCollision([processed[i]], [processed[i + 1]], obstacles):
            verificationFlag &= False

    if not verificationFlag:
        print ("Something is wrong in post-processing")
        print(processed)

    return processed

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (1x6).
    :param goal:        goal pose of the robot (1x6).
    :return:            returns an mx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix..
    """

    #map_struct = loadmap("maps/map5.txt")
    # start = np.array([1.140773925689457, 0.11726018970498742, 1.0621186359361474, 1.56795931069834, -1.9240993391887418, 0.0])
    # goal = np.array([0, 0, 0, 0, 0, 0])
    # path = Astar(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    # print(path)

    #start = np.array([0, 0, 0, 0, 0, 0])

    # goalXYZ = [-275, 0, 375]
    # goalDeg = np.array([0, -15, -111, -11, 0, 0])
    # goalDeg = np.array([0, -3.5, -111, -11, 0, 0])
    # goal = np.radians(goalDeg)

    obstacles = map.obstacles
    base1=np.array([[-30,-30,0.5,-0.1,-0.1,80]])
    base2=np.array([[0.1,0.1,0.1,30,30,80]])
    boundary = map.boundary
    bufferRadius = 40

    #31.75
    # subtract bufferRadius from start XYZ
    # and add to end XYZ
    for obstacle in obstacles:
        obstacle[0] = max(obstacle[0] - bufferRadius, boundary[0])
        obstacle[1] = max(obstacle[1] - bufferRadius, boundary[1])
        obstacle[2] = max(obstacle[2] - bufferRadius, boundary[2])
        obstacle[3] = min(obstacle[3] + bufferRadius, boundary[3])
        obstacle[4] = min(obstacle[4] + bufferRadius, boundary[4])
        obstacle[5] = min(obstacle[5] + bufferRadius, boundary[5])


    # T0e = [ [0, 0, 1, -350],
    #         [0, -1, 0, 0],
    #         [-1, 0, 0, 400],
    #         [0, 0, 0, 1]]
    # print(inverse(np.array(T0e)))
    print(obstacles)

    print(obstacles)
    # print(boundary)

    if (np.array_equal(goal, start)):
        print("start equals goal")
        return [start]

    f = calculateFK()
    startPos, _ = f.forward(start)
    goalPos, _ = f.forward(goal)

    # Desired XYZ of end-effector at goal
    startE = startPos[-1]
    goalE = goalPos[-1]

    # Desired opening width of end-effector at goal
    goalEWidth = goal[-1]

    print("Start XYZ: ", startE)
    print("Goal XYZ: ", goalE)

    print("Start Thetas: ", start)
    print("Goal Thetas: ", goal)

    print("Obstacles: ", obstacles)

    # Check if straight line path exists between start and goal
    lineCollision = obstacleCollision([start], [goal], obstacles)

    # check if start or goal pose is inside a C-space obstacle
    startObstacle = obstacleCollision([start], [start], obstacles)
    goalObstacle = obstacleCollision([goal], [goal], obstacles)
    obstacles=np.append(obstacles,base1,axis=0)
    obstacles=np.append(obstacles,base2,axis=0)
    if not lineCollision:
        print ("no collision on straight line")
        return [start, goal]
    elif(startObstacle or goalObstacle):
        print("Target or Start inside obstacle")
        return ([])

    # currentPose
    currentPose = start

    # list of feasible points on the line such that
    # feasible line exists between adjacent points
    # in the list
    points = [list(start)]
    goalFound = False

    # total number of iterations
    maxIter = 1000
    i = 0

    # Lower joint limits in radians (grip in mm
    # (negative closes more firmly))
    lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]

    # Upper joint limits in radians (grip in mm)
    upperLim = [1.4, 1.4, 1.7, 1.7, 1.5, 30]

    while (not goalFound and i < maxIter):
        # sample a pose
        randQ1 = random.uniform(lowerLim[0], upperLim[0])
        randQ2 = random.uniform(lowerLim[1], upperLim[1])
        randQ3 = random.uniform(lowerLim[2], upperLim[2])
        randQ4 = random.uniform(lowerLim[3], upperLim[3])
        randQE = random.uniform(lowerLim[4], upperLim[4])

        newPose = [randQ1, randQ2, randQ3, randQ4, randQE, goalEWidth]
        #print(i, newPose)

        coll = obstacleCollision([currentPose],[newPose], obstacles)
        coll |= boundaryCollision([currentPose], boundary)

        if not coll:
            points.append(list(newPose))
            currentPose = newPose
            i=0
            if (not obstacleCollision([newPose],[goal], obstacles)):
                points.append(list(goal))
                print("Straight Line to goal feasible")
                goalFound = True
        i += 1
        if i==maxIter:
            print("max iterations reached")

    print("Before post-processing: " + str(len(points)))
    processed = postProcessing(points, obstacles)
    graphTrajectory(processed)
    print("After post-processing: " + str(len(processed)))

    # print(processed[-1])

    # for q_ix in range(len(processed)):
    #     for joint in range(6):
    #         endXYZ = f.forward(points[q_ix])[0][joint]
    #         print("[%i][%i]"%(q_ix, joint) + str(endXYZ) )

    return processed
