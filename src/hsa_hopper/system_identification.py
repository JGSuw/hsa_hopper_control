import numpy as np
import scipy as sp
from scipy.sparse import bsr_array
from scipy.optimize import lsq_linear
from scipy.optimize import minimize, LinearConstraint

def est_velo_accel(x: np.ndarray, t: np.ndarray):
    """
    Given a signal x sampled at times t, estimate the first and second
    derivative of x (velocity and acceleration) on the interior points
    by constructing a system of linear equations from second order Taylor
    expansions of x(t).

    Input:
        x (np.ndarray): (N,) array of sample values
        t (np.ndarray): (N,) array of samples times
    
    Output:
        xdot (np.ndarray): (N,) array of estimated velocity on interior nodes
        xddot (np.ndarray): (N,) array of estimated acceleration on interior nodes
    """
    M = len(t)
    b = np.zeros(2*M)

    # first construct boundary conditions
    
    # x[1] = x[0] + v[0]*dt[0] + .5a[0]*dt[0]^2
    dt = t[1]-t[0]
    A_rows = [0,0]
    A_cols = [0,1]
    A_data = [dt, .5*dt**2]
    b[0] = x[1]-x[0]

    # v[1] = v[0] + a[0]*dt[0]
    A_rows += [1,1,1]
    A_cols += [0,1,2]
    A_data += [1,dt,-1]
    b[1] = 0

    # x[M-2] = x[M-1] - v[M-1]*dt[M-1] + .5a[M-1]*dt[M-1]^2
    dt = t[-1]-t[-2]
    A_rows += [2,2]
    A_cols += [2*M-2, 2*M-1]
    A_data += [-dt, .5*dt**2]
    b[2] = x[-2]-x[-1]

    # v[M-2] = v[M-1] - a[M-1]*dt[M-1]
    A_rows += [3,3,3]
    A_cols += [2*M-4, 2*M-2, 2*M-1]
    A_data += [-1,1,-dt]
    b[3] = 0

    # now construct rows for interior equations
    for i in range(1,M-1):
        # x[k-1] = x[k] - v[k]*dt[k] + .5a[k]*dt[k]^2
        dt = t[i]-t[i-1]
        A_rows += [2+2*i,2+2*i]
        A_cols += [2*i, 2*i+1]
        A_data += [-dt, .5*dt**2]
        b[2+2*i] = x[i-1]-x[i]

        # x[k+1] = x[k] + v[k]*dt[k] + .5a[k]*dt[k]^2
        A_rows += [2+2*i+1, 2+2*i+1]
        A_cols += [2*i, 2*i+1]
        A_data += [dt, .5*dt**2]
        b[2+2*i+1] = x[i+1]-x[i]

    data = np.hstack(A_data)
    A_sparse = bsr_array((data,(A_rows,A_cols)), shape=(2*M,2*M))
    result = lsq_linear(A_sparse, b)
    xdot = result.x[0:-1:2]
    xddot = result.x[1::2]
    return xdot, xddot

def fit_motor_dynamics(x: np.ndarray,
                       xdot: np.ndarray,
                       xddot: np.ndarray,
                       u: np.ndarray):
    """
    Takes estimates of motor position, velocity, and acceleration,
    and command torques, and returns estimates of motor inertia and damping.

    Input:
        x (np.ndarray): (N,) array of motor angles in radians,
        xdot (np.ndarray): (N,) array of motor velocities in radians/s
        xddot (np.ndarray): (N,) array of motor accelerations in radians/s^2
        u (np.ndarray): (N,) array of feedforward torques in Nm

    Output:
        result (scipy.optimize.minimze result): result of optimization,
            the attribute result.x contains the identified model parameters,
            result.x[0] is motor inertia, result.x[1] is motor damping.
    """
    N = 2
    M = x.shape[0]
    A = np.zeros((M,N))
    b = u
    for i in range(M):
        A[i,0] = xddot[i]
        A[i,1] = xdot[i]
    ATA = A.T@A
    ATb = A.T@b
    bTb = b.T@b

    # objective function and derivatives
    f = lambda y: y.T@ATA@y+bTb-2*y.T@ATb
    jac = lambda y: 2*ATA@y-2*ATb
    hess = lambda y: 2*ATA

    # constraints - enforce y strictly positive
    C = np.array([[1,0],[0,1]])
    lb = np.zeros(2)
    constraints = [LinearConstraint(C,lb=lb)]

    y0 = np.array([0,0]) # some initial guess
    
    # call trust-constr to solve the optimization problem
    result = minimize(f, y0,
                      jac=jac,
                      hess=hess,
                      constraints=constraints,
                      method='trust-constr',
                      options={'maxiter': 1000,
                               'gtol': 1e-12,
                               }
                      )
    
    return result
