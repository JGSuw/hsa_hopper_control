from .collocation import interp_covector, interp_tensor, diff_tensor
import numpy as np
from scipy.optimize import minimize
from scipy.sparse import csr_array

class PiecewiseSmoother:

    def __init__(self, 
                 t: np.ndarray,     # (N,) array of time samples
                 x: np.ndarray,     # (N,dim) array to smooth
                 knots: np.ndarray, # (M,) array of knot indices in the time vector
                 degree: int,       # polynomial degree
                 cont=0):           # continuity conditions
    
        self.t = t
        self.x = x
        self.knots = knots
        self.degree = degree
        self.cont = cont

        X_shape = (len(knots)+1, degree+1, x.shape[1])
        self.jac = np.zeros(X_shape)


def cont_constraints(smoother: PiecewiseSmoother, coeffs: np.ndarray):
    x_dims = smoother.x.shape[1:]
    N_knots = len(smoother.knots)
    N_poly = smoother.degree+1
    cont = smoother.cont
    X_shape = (N_knots+1, N_poly, *x_dims)
    if coeffs.shape != X_shape:
        # get a multidimensional view of the coefficient array
        X = np.reshape(coeffs, X_shape, order='F')
    else:
        # use the multidimensional view of the coeffs array
        X = coeffs

    # allocate memory for constraint defects
    defects = np.zeros((cont+1, N_knots, *x_dims))

    # calculate defects
    for i in range(N_knots):
        # knot point under consideration
        kb = smoother.knots[i]
        b = smoother.t[kb]

        # get start of left interval
        if i == 0:
            a = smoother.t[0]
        else:    
            ka = smoother.knots[i-1]
            a = smoother.t[ka]
        # get start of right interval
        if i == N_knots-1:
            c = smoother.t[-1]
        else:
            kc = smoother.knots[i+1]
            c = smoother.t[kc]
        # time covectors for left and right side
        Ta = interp_covector(b,a,b,N_poly) 
        Tb = interp_covector(b,b,c,N_poly) 
        defects[0, i, :] = np.einsum('ij,jk->k',Ta,X[i,:]) - np.einsum('ij,jk->k',Tb,X[i+1,:])

        # continuity of other derivatives
        dta = diff_tensor(a,b,N_poly)
        dtb = diff_tensor(b,c,N_poly)
        for j in range(1,cont+1):
            # left side evaluation
            defects[j, i, :] = np.einsum('ij,jk->k',Ta,dta@X[i,:]) - np.einsum('ij,jk->k',Tb,dtb@X[i+1,:])
            # raise diff tensors to the next order
            dta = dta @ dta
            dtb = dtb @ dtb 
    
    # return flattened view of defects for the optimizer
    return defects.flatten(order='F')

def residuals(smoother: PiecewiseSmoother, coeffs: np.ndarray):
    x_dim = smoother.x.shape[1]
    N_knots = len(smoother.knots)
    N_poly = smoother.degree+1
    X_shape = (N_knots+1, N_poly, x_dim)
    if coeffs.shape != X_shape:
        # get a multidimensional view of the coefficient array
        X = np.reshape(coeffs, X_shape, order='F')
    else:
        # use the multidimensional view of the coeffs array
        X = coeffs
    # container for storing interpolation residuals for each knot
    residuals = []
    smoother.jac = np.zeros(X_shape)
    for i in range(N_knots+1):
        if i == 0:
            ka = 0
            kb = smoother.knots[0]
            a = smoother.t[0]
            b = smoother.t[kb]
        elif i == N_knots:
            ka = smoother.knots[-1]
            kb = smoother.t.size
            a = smoother.t[ka]
            b = smoother.t[-1]
        else:
            ka = smoother.knots[i-1]
            kb = smoother.knots[i]
            a = smoother.t[ka]
            b = smoother.t[kb]
        t = smoother.t[ka:kb]
        x = smoother.x[ka:kb]
        # get an interpolation tensor
        T = interp_tensor(t, a, b, N_poly)
        # compute residuals on this knot
        res = np.einsum('ij,jk->ik',T,X[i,:,:]) - x
        smoother.jac[i,:,:] = np.einsum('ij,ik->jk',T,2*res)
        residuals.append(res)
    
    # returned concatenated residuals
    return np.concatenate(residuals, axis=0)

def sos_error(smoother: PiecewiseSmoother, coeffs: np.ndarray):
    res = residuals(smoother, coeffs)
    return np.sum(res**2)


def evaluate(smoother: PiecewiseSmoother, coeffs: np.ndarray, ord=0):
    x_dims = smoother.x.shape[1:]
    N_knots = len(smoother.knots)
    N_poly = smoother.degree+1
    X_shape = (N_knots+1, N_poly, *x_dims)
    if coeffs.shape != X_shape:
        # get a multidimensional view of the coefficient array
        X = np.reshape(coeffs, X_shape, order='F')
    else:
        # use the multidimensional view of the coeffs array
        X = coeffs
    evals = []
    for i in range(N_knots+1):
        if i == 0:
            ka = 0
            kb = smoother.knots[0]
            a = smoother.t[ka]
            b = smoother.t[kb]
        elif i == N_knots:
            ka = smoother.knots[-1]
            kb = smoother.t.size
            a = smoother.t[ka]
            b = smoother.t[-1]
        else:
            ka = smoother.knots[i-1]
            kb = smoother.knots[i]
            a = smoother.t[ka]
            b = smoother.t[kb]
        t = smoother.t[ka:kb]
        # get an interpolation tensor
        T = interp_tensor(t, a, b, N_poly)
        dt = diff_tensor(a, b, N_poly)

        # contract T with dt by ord times
        for j in range(ord):
            T = np.einsum('ij,jk->ik', T, dt)

        # compute residuals on this knot
        x = np.einsum('ij,jk->ik',T,X[i,:,:])
        evals.append(x)

    return np.concatenate(evals, axis=0)

def fit(smoother: PiecewiseSmoother):
    x_dims = smoother.x.shape[1:]
    N_knots = len(smoother.knots)
    N_poly = smoother.degree+1
    X_shape = (N_knots+1, N_poly, *x_dims)
    coeffs = np.zeros(np.prod(X_shape))
    f = lambda coeffs: (sos_error(smoother, coeffs), smoother.jac.flatten(order='F'))
    if smoother.cont >= 0:
        g = lambda coeffs: cont_constraints(smoother, coeffs)
        result = minimize(f, coeffs, constraints=[{'type': 'eq', 'fun' : g}], method='SLSQP', jac=True)
    else:
        result = minimize(f, coeffs, method='SLSQP', jac=True)
    result.coeffs = np.reshape(result.x, X_shape, order='F')
    return result