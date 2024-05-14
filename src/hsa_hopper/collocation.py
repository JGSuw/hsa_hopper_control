import math
import numpy as np
from scipy.optimize import minimize, OptimizeResult
import yaml
import hsa_hopper.dynamics as dynamics
from .dynamics import DynamicsParameters

def interp_covector(t, a, b, N, ord=0):
    """
    Computes a covector for evaluating a N-1 order interpolation on the interval (a,b).
    This scheme uses a symmetric polynomial basis centered at the midpoint (b+a)/2.
    Monomial degree increases with the index, so that T[i] = s(t)**i, and s(t) is the time
    parameterization t -> (2/(b-a))*(t-(b+a)/2)

        Inputs: 
        t (float): time to evaluate interpolation at
        a (float): left-side of interpolation interval
        b (float): right-side of interpolation interval
        N (int): dimension of polynomial basis - highest monomial degree is N-1

        Returns:
        T (np.ndarray): (1,N) covector.
    """
    T = np.zeros((1,N))
    scale = 2/(b-a)
    mid = (b+a)/2
    for n in range(N):
        if ord >= 0: # differentiation or interpolation, same code
            if n-ord >= 0:
                T[0,n] = (t-mid)**(n-ord) * math.factorial(n)/(math.factorial(n-ord)) * (scale ** n)
        if ord == -1: # integration case
            if n%2 == 0:
                T[n] = (b-a)/(n+1)
    return T

def quad_int_tensor(a,b,N):
    """
    Helper function to compute the integral from t=a to t=b of T.T@T
    where T = interp_covector(t,a,b,N).
    This allows us to easy compute the integral of the square of an interpolation
    function.

        Parameters:
            a (float): start of interval
            b (float): end of interval
            N (int): number of interpolation coefficients / monomial terms

        Returns:
            I (np.ndarray): an (N,N) square matrix so that given N-vector c, c@I@c computes the integral
                of the squared interpolation implemented by c.
    """
    I = np.zeros((N,N))
    for (i,j) in np.ndindex(I.shape):
        if ((i+j)%2) == 0:
            I[i,j] = (b-a)/(i+j+1)
    return I

class CollocationParameters:
    def __init__(self, Ns, Nx, Nu, a, b):
        """
        Creates a dictionary of collocation parameters for constructing approximate
        polynomial solutions to boundary-vaue problems subject to dynamics 
        x_ddot = f(x,x_dot,u)

            Parameters:
                Ns: number of splines
                Nx: dimension of position (x) interpolation
                Nu: dimension of control (u) interpolation
                T: time interval, BVP is solved for t in range (0,T)

            Returns: dictionary of parameters that define the solution space
        """ 
        self.Ns, self.Nx, self.Nu, self.a, self.b = Ns, Nx, Nu, a, b
        tk = np.linspace(a,b,Ns+1)
        tc = np.array([[(tk[i]+tk[i+1])/2+(tk[i+1]-tk[i])*(np.cos(s)/2) for s in np.linspace(-np.pi,0,Nx)] for i in range(Ns)])
        self.tk = tk
        self.tc = tc

    def attribute_dict(self):
        attributes = {}
        for key in self.__dict__:
            value = self.__dict__[key]
            if type(value) == np.ndarray:
                attributes[key] = value.tolist()
            else:
                attributes[key] = value
        return attributes

    def make_from_dict(attributes):
        return CollocationParameters(
                attributes['Ns'],
                attributes['Nx'],
                attributes['Nu'],
                attributes['a'],
                attributes['b']
                )

class PiecewiseInterpolation:
    def __init__(self,
                 mat: np.array, # ndarray with shape (M,N)
                 tk: np.array,  # ndarray with shape (M+1,)
    ):
        """
        Create a piecewise polynomial interpolation of a scalar function.

        Inputs:
            mat (np.ndarray): 2D array with shape (M,N) of interpolation coefficients
            tk (np.ndarray): 1D array with shape (M+1,) of knot points
        """
        if len(mat.shape) != 2:
            raise ValueError('mat must be 2D ndarray')
        self.mat = mat
        self.M, self.N = mat.shape
        if tk.shape != (self.M+1,):
            raise ValueError('tk must be ndarray with length mat.shape[0]+1')
        self.tk = tk

    def evaluate(self, t):
        """
        Evaluates the interpolation at time t.

            Inputs:
            t (float): time to evaluate

            Returns:
            x (float): value of interpolation x(t)
        """
        try: 
            idx = next(i-1 for i in range(1,self.M+1) if t <= self.tk[i])
        except StopIteration:
            idx = self.M-1
        T = interp_covector(t,self.tk[idx],self.tk[idx+1],self.N)
        return (T[0,:]@self.mat[idx,:])

    def attribute_dict(self):
        return {'mat': self.mat.tolist(), 'tk': self.tk.tolist()}

    def make_from_dict(attributes):
        return PiecewiseInterpolation(
                np.array(attributes['mat']),
                np.array(attributes['tk'])
                )
    
            
def fit_interpolation(x: np.ndarray, u: float, t: float, collo_params: CollocationParameters):
    """
    Generate an initial guess for the optimal control problem subject to boundary constraints.
    x: array of position coordinates to interpolate
    u: array of controls to interpolate
    t: array of times to interpolate
    params: dictionary of collocation parameters

    Returns:
        c_mat: coefficient matrix for x interpolation
        d_mat: coefficient matrix for u interpolation
    """
    Ns = collo_params.Ns
    Nx = collo_params.Nx
    Nu = collo_params.Nu
    tk = collo_params.tk
    tc = collo_params.tc

    # solve for x interpolation
    A = np.zeros((Ns*Nx,Ns*Nx))   # A matrix for linear system A@c = b
    B = np.interp(tc, t, x)
    b = np.reshape(B,Ns*Nx)
    for i in range(Ns):
        for j in range(Nx):
            A[Nx*i+j,Nx*i:Nx*(i+1)] = interp_covector(tc[i,j],tk[i],tk[i+1],Nx)
    c,err,rank,vectors = np.linalg.lstsq(A,b,rcond=None)
    c_mat = np.reshape(c,(Ns,Nx))

    # solve for u interpolation
    A = np.zeros((Ns*Nu,Ns*Nu))   # A matrix for linear system A@d = B
    B = np.interp(tc[:,:Nu], t, u)
    b = np.reshape(B,Ns*Nu)
    for i in range(Ns):
        for j in range(Nu):
            A[Nu*i+j,Nu*i:Nu*(i+1)] = interp_covector(tc[i,j],tk[i],tk[i+1],Nu)
    d,err,rank,vectors = np.linalg.lstsq(A,b,rcond=None)
    d_mat = np.reshape(d,(Ns,Nu))
    x_interp = PiecewiseInterpolation(c_mat,tk)
    u_interp = PiecewiseInterpolation(d_mat,tk)
    return x_interp, u_interp

class HopBVP:
    def __init__(self, 
                 initial_cond: np.ndarray, 
                 final_cond: np.ndarray,
                 lb: np.ndarray, 
                 ub: np.ndarray, 
                 dynamic_params: DynamicsParameters,
                 collo_params: CollocationParameters):
        self.initial_cond = initial_cond
        self.final_cond = final_cond
        self.lb = lb
        self.ub = ub
        self.dynamic_params = dynamic_params
        self.collo_params = collo_params

    def attribute_dict(self):
        attributes = {}
        for key in self.__dict__:
            value = self.__dict__[key]
            if type(value) == np.ndarray:
                attributes[key] = value.tolist()
            elif type(value) == DynamicsParameters:
                attributes[key] = value.attribute_dict()
            elif type(value) == CollocationParameters:
                attributes[key] = value.attribute_dict()
            elif type(value) == OptimizeResult:
                sub_attr = {}
                for subkey in dir(value):
                    subvalue = getattr(value, subkey)
                    if type(subvalue) == np.ndarray:
                        sub_attr[subkey] = subvalue.tolist()
                    else:
                        try:
                            if str(subvalue).isnumeric():
                                sub_attr[subkey] = subvalue
                        except:
                            pass
                attributes[key] = sub_attr
        return attributes


    def boundary_conditions(self, z):
        '''
        Computes boundary value constraints on initial and final state given
        large array of position/control interpolation coefficients.

            Parameters:
                z (array like): array of interpolation coefficients
                collo_params (dict): dictionary that defines the collocation parameters

            Returns:
                data (np.array): array of residuals to the boundary conditions
        '''
        Ns = self.collo_params.Ns
        Nx = self.collo_params.Nx
        tc = self.collo_params.tc
        c1 = z[0:Nx]
        data = np.zeros(4)
        I0 = interp_covector(tc[0,0],tc[0,0],tc[0,-1],Nx)
        data[0] = (I0@c1)[0]-self.initial_cond[0]
        I1 = interp_covector(tc[0,0],tc[0,0],tc[0,-1],Nx,ord=1)
        data[1] = (I1@c1)[0]-self.initial_cond[1]
        c2 = z[(Ns-1)*Nx:Ns*Nx]
        I0 = interp_covector(tc[-1,-1],tc[-1,0],tc[-1,-1],Nx)
        data[2] = (I0@c2)[0]-self.final_cond[0]
        I1 = interp_covector(tc[-1,-1],tc[-1,0],tc[-1,-1],Nx,ord=1)
        data[3] = (I1@c2)[0]-self.final_cond[1]
        return data

    def continuity_constraints(self, z):
        '''
        Computes the residual of equality constraints on the piecewise interpolation
        that enforce continuity of position, velocity, acceleration, and control.

            Parameters:
                z (array like): array of interpolation coefficients
            
            Returns:
                data (np.array): array of constraint residuals
        '''
        Ns = self.collo_params.Ns
        Nx = self.collo_params.Nx
        Nu = self.collo_params.Nu
        tk = self.collo_params.tk
        data = np.zeros(3*(Ns-1))
        for i in range(Ns-1):
            c0, d0 = z[Nx*i:Nx*(i+1)], z[Ns*Nx+Nu*i:Ns*Nx+Nu*(i+1)]
            a0,b0 = tk[i], tk[i+1]
            c1, d1 = z[Nx*(i+1):Nx*(i+2)], z[Ns*Nx+Nu*(i+1):Ns*Nx+Nu*(i+2)]
            a1,b1 = tk[i+1], tk[i+2]
            t = tk[i+1]
            # position
            data[3*i] = (interp_covector(t,a0,b0,Nx)@c0)[0] - (interp_covector(t,a1,b1,Nx)@c1)[0]
            # velocity
            data[3*i+1] = (interp_covector(t,a0,b0,Nx,ord=1)@c0)[0] - (interp_covector(t,a1,b1,Nx,ord=1)@c1)[0]
            # acceleration
            #data[4*i+2] = (interp_covector(t,a0,b0,Nx,ord=2)@c0)[0] - (interp_covector(t,a1,b1,Nx,ord=2)@c1)[0]
            # control
            data[3*i+2] = (interp_covector(t,a0,b0,Nu)@d0)[0] - (interp_covector(t,a1,b1,Nu)@d1)[0]
        return data

    def dynamic_constraints(self, z):
        '''
        Computes residual of equality constraints that enforce the second-order equation
        xddot - fun(x,xdot,u) = 0 at the knot points of the interpolation scheme.

            Parameters:
                z (array like): array of interpolation coefficients
                fun (function(x,xdot,u)->xddot): function that computes second-order dynamics

            
            Returns:
                data (np.array): array of constraint residuals
        '''
        Ns = self.collo_params.Ns
        Nx = self.collo_params.Nx
        Nu = self.collo_params.Nu
        tc = self.collo_params.tc
        data = np.zeros(Ns*(Nx))
        for i in range(Ns):
            c = z[Nx*i:Nx*(i+1)]
            d = z[Ns*Nx+Nu*i:Ns*Nx+Nu*(i+1)]
            a,b = tc[i,0],tc[i,-1]
            I0 = np.vstack([interp_covector(_t,a,b,Nx,ord=0) for _t in tc[i,:]])
            I1 = np.vstack([interp_covector(_t,a,b,Nx,ord=1) for _t in tc[i,:]])
            I2 = np.vstack([interp_covector(_t,a,b,Nx,ord=2) for _t in tc[i,:]])
            x = (I0@c)
            xdot = (I1@c)
            xddot = (I2@c)
            u = (I0[:,:Nu]@d)
            f = np.array([
                dynamics.evaluate(x[i], xdot[i], u[i], self.dynamic_params)
                for i, _t in enumerate(tc[i,:])
                ])
            data[(Nx)*i:(Nx)*(i+1)] = xddot - f
        return data

    def inequality_constraints(self, z):
        '''
        Enforces bounds on position and control at all collocation points
        by computing linear inequalities.

            Parameters:
                z (array like): array of interpolation coefficients
            
            Returns:
                data (np.array): array of constraint residuals
        '''
        Ns = self.collo_params.Ns
        Nx =  self.collo_params.Nx
        Nu = self.collo_params.Nu
        tc = self.collo_params.tc
        Kx =  self.dynamic_params.Kx
        x0 = self.dynamic_params.x0
        lb = self.lb
        ub = self.ub
        data = np.zeros(Ns*(4*Nx))
        for i in range(Ns):
            for j in range(Nx):
                c = z[Nx*i:Nx*(i+1)]
                d = z[Ns*Nx+Nu*i:Ns*Nx+Nu*(i+1)]
                t,a,b = tc[i,j], tc[i,0], tc[i,-1]
                I = interp_covector(t,a,b,Nx)
                theta = (I@c)[0]
                data[4*(Nx*i+j)] = theta - lb[0]
                data[4*(Nx*i+j)+1] = ub[0] - theta
                u = (I[:,:Nu]@d)[0]
                data[4*(Nx*i+j)+2] = u + Kx*(x0-theta) - lb[1]
                data[4*(Nx*i+j)+3] = ub[1] - u - Kx*(x0-theta)
            return data
    
    def cost(self, z):
        Ns = self.collo_params.Ns
        Nx =  self.collo_params.Nx
        Nu = self.collo_params.Nu
        tk = self.collo_params.tk
        Kx =  self.dynamic_params.Kx
        x0 = self.dynamic_params.x0
        _grad = np.zeros(z.shape)
        _cost = 0
        scale = 1
        for i in range(Ns):
            # unpacking z into u and x parts
            c = z[Nx*i:Nx*(i+1)]
            d = z[Ns*Nx+Nu*i:Ns*Nx+Nu*(i+1)]

            # quadratic integration tensor
            I = quad_int_tensor(tk[i],tk[i+1],Nx)

            # tau**2 = (u+Kx*(x0-x))**2
            # expanding...
            # u**2 + Kx**2 * (x0**2 -2*x0*x + x**2) + 2*Kx*u*(x0-x)

            # u**2 term
            _cost += d@(I[:Nu,:Nu]@d)
            # grad wrt to u
            _grad[Ns*Nx+Nu*i:Ns*Nx+Nu*(i+1)] += 2*I[:Nu,:Nu]@d

            # (Kx*x0)**2 term, no grad
            _cost += (Kx*x0)*I[0,0]*(Kx*x0)

            # -2*Kx**2 * (x0*x) term
            _cost -= 2*(Kx*x0)*(I[0,:]@(Kx*c))
            # grad wrt to x
            _grad[Nx*i:Nx*(i+1)] -= 2*(Kx*x0)*(I[0,:]*Kx)

            # (Kx*x)**2 term
            _cost += (Kx*c)@(I@(Kx*c))
            # grad wrt to x
            _grad[Nx*i:Nx*(i+1)] += 2*Kx*I@(Kx*c)

            # 2*Kx*u*(x0-x) term
            _cost += 2*Kx*(d@(I[:Nu,0]*x0-I[:Nu,:]@c))
            # grad wrt to u
            _grad[Ns*Nx+Nu*i:Ns*Nx+Nu*(i+1)] += 2*Kx*(I[:Nu,0]*x0-I[:Nu,:]@c)
            # grad wrt to x
            _grad[Nx*i:Nx*(i+1)] -= 2*Kx*(d@I[:Nu,:])

        return _cost*scale, _grad*scale

    def optimize(self, initial_guess, options={}):
        self.result = minimize(self.cost, initial_guess, method='SLSQP', jac=True, 
                constraints = [
                {'type' : 'eq', 'fun': self.boundary_conditions},
                {'type' : 'eq', 'fun': self.continuity_constraints},
                {'type' : 'ineq', 'fun': self.inequality_constraints},
                {'type' : 'eq', 'fun': self.dynamic_constraints},
                ],
                options=options
                )
        Ns = self.collo_params.Ns
        Nx = self.collo_params.Nx
        Nu = self.collo_params.Nu
        self.c_mat = np.reshape(self.result.x[:Ns*Nx],(Ns,Nx))
        self.d_mat = np.reshape(self.result.x[Ns*Nx:],(Ns,Nu))

        return self.result

