def calcJacobian(q, joint):
    Jacv=np.zeros((3,6)) #zeroes after col: joint+1
    Jacw=np.zeros((3,6)) #zeroes after col: joint+1
    #For loop with the size of 3 x joint
    
    R=calculateFK()
    z=[] # np.array(3,joint)
    z.append(np.array([0,0,1]))
    z.append(np.array([-np.sin(q[0]),np.cos(q[0]),0]))
    z.append(np.array([-np.sin(q[0]),np.cos(q[0]),0]))
    z.append(np.array([-np.sin(q[0]),np.cos(q[0]),0]))

    Jp,T0e=R.forward(q)
    z.append([T0e[0,2],T0e[1,2],T0e[2,2]])
    z.append([T0e[0,2],T0e[1,2],T0e[2,2]])

    for i in range (1,joint+1):
        Jo=Jp[5]-Jp[i-1]
        Jr=np.cross(z[i-1],Jo)
        Jacv[0,i-1]=Jr[0]
        Jacv[1,i-1]=Jr[1]
        Jacv[2,i-1]=Jr[2]

    z = np.array(z)
    z[joint+1:len(z)] *= 0
    Jacw=np.transpose(np.array(z))
    return (np.append(Jacv, Jacw, axis=0))
