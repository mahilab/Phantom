def calc_torques(q,qd,qref):
    Tau = [0,0,0]
    Kp = (10,10,10)
    Kd = (0.5,0.5,0.5)
    for i in range(0,3):
        Tau[i] = Kp[i]*(qref[i]-q[i])-Kd[i]*qd[i]
    return tuple(Tau)