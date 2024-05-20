import numpy as np
from scipy.sparse import bsr_array
from scipy.optimize import lsq_linear
from hsa_hopper.hsa_model import HSAPotential

def estimate_vel_acc(x: np.ndarray, t: np.ndarray):
    """
    Given two arrays for a signal x(t), compute estimates of the first and second derivatives
    of x(t) on the slice t[1:-1].

    Input:
        x (np.ndarray): (N+1,) array of signal values
        t (np.ndarray): (N+1,) array of time values

    Output:
        xdot (np.ndarray): (N,) array of velocities on t[1:-1]
        xddot (np.ndarray): (N,) array of accelerations on t[1:-1]
    """
    M = len(t)
    A_rows = np.hstack([np.array([2*i, 2*i, 2*i+1, 2*i+1]) for i in range(0,M-2)])
    A_cols = np.hstack([np.array([2*i, 2*i+1, 2*i, 2*i+1]) for i in range(0,M-2)])
    A_data = []
    b = np.zeros(2*(M-2))
    for i in range(M-2):
        dt = t[i]-t[i-1]
        A_data.append([-dt, .5*dt**2, dt, .5*dt**2])
        b[2*i] = x[i]-x[i+1]
        b[2*i+1] = x[i+2]-x[i+1]
    data = np.hstack(A_data)
    A_sparse = bsr_array((data,(A_rows,A_cols)), shape=(2*(M-2),2*(M-2)))
    result = lsq_linear(A_sparse, b)
    xdot = result.x[0:-1:2]
    xddot = result.x[1::2]
    return xdot, xddot
