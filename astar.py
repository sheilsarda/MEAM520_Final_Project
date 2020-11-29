from heapq import heappush, heappop  # Recommended.
import numpy as np
import math
import matplotlib.pyplot as plt
from calculateFK import calculateFK
from detectCollision import detectCollision

def Astar(map, start, goal):
    """
    Parameters:
        maps,       struct containing map information
        start,      inital configuration array, shape=(6,)
        goal,       final configuration array, shape=(6,)
    Output:
        path,       configuration coordinates along the path  with
                    shape=(N,6). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """
    start_backup = start
    goal_backup = goal
    path = [goal]
    start = start[[0,1,2,3]]
    goal = goal[[0,1,2,3]]

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(map, [0.1, 0.1, 0.1, 0.3], 30)
    print("Finished Loading in Occupancy Map")
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    i, j, k, l = occ_map.occ.shape

    if not occ_map.is_valid_index(goal_index) :
        print("invalid Goal Index")
        return np.array([])
    if not occ_map.is_valid_index(start_index):
        print("invalid start Index")
        return np.array([])
    if occ_map.is_occupied_index(start_index):
        print("Occupied start Index")
        return np.array([])
    if occ_map.is_occupied_index(goal_index):
        print("Occupied Goal Index")
        return np.array([])
    # initialize cost, cost-to-come from start to current node + heuristic from current to goal
    g = np.full((i, j, k, l), np.inf)

    # initialize parent node
    # two dimension, there are 2 number in each voxels
    p = np.zeros((i, j, k, l, 4))

    # create heap name G
    G = []

    # cost to go from current node to goal node is heuristic
    g[start_index] = 0

    # push the start and goal:  cost and its position index to the heap
    heappush(G, (g[start_index], start_index))
    heappush(G, (g[goal_index], goal_index))

    # this is the same as have Q: priority queue of open/alive nodes, if not in Q, the node is closed.
    # should add all the node th Q at first to mark all of them open.
    # Here use state to mark the state as open, alive, close
    # create state to record the status of the node: open/alive/close
    state = np.zeros((i, j, k, l))  # 0 is not open, 1 is open, 2 is close
    state[start_index] = 1  # add start_index to open

    # if all the cost in g are smaller than inf, there might have a path found
    # or if the goal is closed, there might have a path found
    while np.min(g) < np.inf and not state[goal_index] == 2:  # while there is node not open and the goal is not closed
        # pick the node with smallest cost as the current node, pop it out then mark it as closed
        u = heappop(G)[1]
        state[u] = 2

        # extra check to make sure we dont explore more nodes around the goal than needed
        if state[goal_index == 2]:
            break

        # for the neighbors of the current node, update their cost and update their parent
        # count3 = count3 + 1
        for i in range(u[0] - 1, u[0] + 2):
            for j in range(u[1] - 1, u[1] + 2):
                for k in range(u[2] - 1, u[2] + 2):
                    for l in range(u[3] - 1, u[3] + 2):
                        v = (i, j, k, l)
                        # determine the distance between voxel
                        if occ_map.is_valid_index(v):
                            if not occ_map.is_occupied_index(v):

                                # chech if the neightbor is open, is it in Q. Here we check its status with 2
                                if not state[v] == 2:  # if neighbor is not closed
                                    # calculate costs
                                    h = np.linalg.norm(occ_map.index_to_metric_negative_corner(goal_index) - occ_map.index_to_metric_negative_corner(v))
                                    c = np.linalg.norm(occ_map.index_to_metric_negative_corner(u) - occ_map.index_to_metric_negative_corner(v))
                                    d = g[u] + c

                                    # check old cost with new cost, then update
                                    if state[v] == 1:  # this neighbor is already opened
                                        if g[v] > d:
                                            G.remove((g[v]+h, v))
                                            g[v] = d
                                            p[v] = u
                                            heappush(G, (g[v]+h, v))
                                    elif state[v] == 0:  # this neighbor haven't been touched
                                        g[v] = d
                                        p[v] = u
                                        # heuristic is only used with heap-q
                                        heappush(G, (g[v]+h, v))
                                        state[v] = 1

    # if the parent of the goal node is empty, that measn no path is found
    if (p[goal_index] == (0, 0, 0, 0)).all():  # goal not found
        path = None
        raise Exception("No Path Found. ")

    else:
        # using the parent index, trackback to form the path
        pointer = goal_index
        while pointer != start_index:
            par = p[pointer]
            path.append(np.append(occ_map.index_to_metric_negative_corner(par), goal_backup[[4, 5]]))
            ind1 = int(par[0])
            ind2 = int(par[1])
            ind3 = int(par[2])
            ind4 = int(par[3])
            pointer = (ind1, ind2, ind3, ind4)
        path.append(start_backup)
        path.reverse()
        path = np.array(path)

    return path

class OccupancyMap:

    def __init__(self, map, resolution, radius):
        """
        Initialize Occupancy map. This would create the grid map based on the dimension and resolutions
        defined in the Json file. It would then load the blocks and mark their corresponding grid as True
         as occupied.
        :param map: the map struct.
        """

        self.FK = calculateFK()
        # Pad length of gripper
        self.FK.L5 = 34 + 34

        # Get joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9])    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7])       # Upper joint limits in radians (grip in mm)
        self.resolution = resolution
        self.origin = self.lowerLim

        # Loading blocks into occupancy map.
        self.blocks = map.obstacles
        self.InflatedBlocks = self.BlockInflation(self.blocks, radius)

        # For each joint find all combinations
        th1_val = np.arange(self.lowerLim[0], self.upperLim[0], resolution[0])
        th2_val = np.arange(self.lowerLim[1], self.upperLim[1], resolution[1])
        th3_val = np.arange(self.lowerLim[2], self.upperLim[2], resolution[2])
        th4_val = np.arange(self.lowerLim[3], self.upperLim[3], resolution[3])

        # Initialize occupancy map
        self.occ = np.zeros([len(th1_val)+1, len(th2_val)+1, len(th3_val)+1, len(th4_val)+1])
        self.occ_check = np.zeros([len(th1_val)+1, len(th2_val)+1, len(th3_val)+1, len(th4_val)+1])


    def metric_to_index(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is the lesser index.
        """
        return np.floor((np.array(metric) - np.array(self.origin))/self.resolution).astype('int')

    def index_to_metric_center(self, ind):
        """
        :param ind:
        :return:
        """
        return self.index_to_metric_negative_corner(ind) + np.array(self.resolution)/2.0

    def index_to_metric_negative_corner(self,ind):
        """
        Given the index, return the
        :param ind:
        :return:
        """
        return ind*np.array(self.resolution) + np.array(self.origin)

    def is_occupied_index(self, ind):
        """
        Check if occupied is occupied.
        :param ind:
        :return:
        """
        if not self.occ_check[ind[0]][ind[1]][ind[2]][ind[3]]:
            self.process_index(ind)
        return self.occ[ind[0]][ind[1]][ind[2]][ind[3]]

    def process_index(self, ind):
        """
        Calculates if the metric described by this index is occupied
        :param ind:
        :return:
        """
        beg_index = [0,1,2,3,4]
        end_index = [1,2,3,4,5]
        joint_vector = self.index_to_metric_negative_corner(ind)
        joint_pos, foo = self.FK.forward(np.append(joint_vector, [0, 0]))
        beg_points = joint_pos[beg_index, ::]
        end_points = joint_pos[end_index, ::]

        for block in self.InflatedBlocks:
            is_collided = detectCollision(beg_points, end_points, block)
            if any(is_collided):
                self.occ[ind[0]][ind[1]][ind[2]][ind[3]] = True
                break
        self.occ_check[ind[0]][ind[1]][ind[2]][ind[3]] = True

    def is_valid_index(self, ind):
        """
        Check if the index is valide or not.
        :param ind:
        :return:
        """
        if ind[0] >= 0 and ind[0] < self.occ.shape[0]:
            if ind[1] >= 0 and ind[1] < self.occ.shape[1]:
                if ind[2] >= 0 and ind[2] < self.occ.shape[2]:
                    if ind[3] >= 0 and ind[3] < self.occ.shape[3]:
                        return True
        return False

    def is_valid_metric(self, metric):
        """
        Check it the metric is inside the boundary or not.
        :param metric:
        :return:
        """
        for i in range(4):
            if metric[i] <= self.lowerLim[i] or metric[i] >= self.upperLim[i]:
                return False
        return True

    def BlockInflation(self, blocks, radius):
        """
        Inflate the Block dimension with the radius of the robot.
        :param block:
        :param radius:
        :return:  ndarray
        """
        modifier = np.array([-radius, -radius, -radius, radius, radius, radius])
        return np.array([modifier + block for block in blocks])
