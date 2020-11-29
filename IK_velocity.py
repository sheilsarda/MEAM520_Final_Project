import numpy as np


def IK_velocity (q, v, omega, joint):
    #dq = np.array([0, 0, 0, 0, 0, 0])
    Jac=calcJacobian(q,joint)
    vel=np.append(v,omega)
    vind=np.argwhere(np.isnan(vel))
    filteredJac=np.delete(Jac,vind.T,0)
    nv=np.delete(vel,vind.T,0).T
    dq=np.matmul(np.linalg.pinv(filteredJac),nv)

    return dq
