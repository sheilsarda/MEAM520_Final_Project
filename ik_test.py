from calculateIK import Main
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R

r = R.from_rotvec(np.pi * np.array([0, 1, 0]))

main = Main()

# T0e = np.array([[   0.99500416,    0.09983344,    0.00000973,   -9.99981939],
#                 [  -0.09983344,    0.99500416,    0.00008253, -375.99841183],
#                 [  -0.00000144,   -0.00008309,    1.        ,   29.00170962],
#                 [   0.        ,    0.        ,    0.        ,    1.        ]])

T0e = np.array([[  0,    1,    0.0,   -9.99981939],
                [  1,    0,    0.0, -375.99841183],
                [  0,    0,   -1.0,   39.00170962],
                [  0,    0.        ,    0.0,    1]])

# print(r.as_matrix())

# new_rot = r.as_matrix() * T0e[:3,:3]
# print(new_rot)

# T0e[:3, :3] = new_rot
[q, isPos] = main.inverse(deepcopy(T0e))
print(q)