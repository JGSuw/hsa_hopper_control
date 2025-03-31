import math
import numpy as np
from scipy.optimize import minimize, OptimizeResult
import yaml
import hsa_hopper.dynamics as dynamics
from .dynamics import DynamicsParameters, stance_dynamics

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
                T[0,n] = (b-a)/(n+1)
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

def diff_tensor(a,b,N):
    """
    Returns a (N,N) matrix that can differentiate an N-1 degree polynomial.
    """
    L = np.zeros((N,N))
    for i in range(N-1):
        L[i,i+1] = i+1
    return 2/(b-a)*L

class CollocationParameters:
    def __init__(self, Ns, Nx, Nu, Nc, a, b):
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
        self.Ns, self.Nx, self.Nu, self.Nc, self.a, self.b = Ns, Nx, Nu, Nc, a, b
        assert(Nc > 1)
        tk = np.linspace(a,b,Ns+1)
        tc = np.zeros((Ns,Nc))
        for i in range(Ns):
            a = tk[i]
            b = tk[i+1]
            tc[i,:] = (a+b)/2+(b-a)/2*np.cos(np.linspace(-np.pi,0,Nc))
        self.tk = tk
        self.tc = tc

        # tensors for evaluating interpolations at collocation points
        self.T = np.zeros((Ns,Nc,Nx))
        self.dt = np.zeros((Ns,Nx,Nx))
        for i in range(Ns):
            self.dt[i,:,:] = diff_tensor(tk[i],tk[i+1],Nx)
            for j in range(Nc):
                self.T[i,j,:] = interp_covector(tc[i,j],tk[i],tk[i+1],Nx)

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
                attributes['Nc'],
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

    def evaluate(self, t, ord=0):
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
        T = interp_covector(t,self.tk[idx],self.tk[idx+1],self.N,ord=ord)
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
    Nc = collo_params.Nc
    tk = collo_params.tk
    tc = collo_params.tc

    # solve for x interpolation (x must be 1d)
    A = np.zeros((Ns,Nx,Ns,Nx))
    b = np.zeros((Ns,Nx))
    print(A.shape)
    print(b.shape)
    theta = np.linspace(-np.pi,0,Nx)
    for i in range(Ns):
        t0 = tk[i]
        t1 = tk[i+1]
        tvec = (t0+t1)/2+(t1-t0)/2*np.cos(theta)
        b[i,:] = np.interp(tvec,t,x)
        for j in range(Nx):
            A[i,j,i,:] = interp_covector(tvec[j],tk[i],tk[i+1],Nx)
    c= np.linalg.tensorsolve(A,b)
    print(c)
    c_mat = c

    A = np.zeros((Ns,Nu,Ns,Nu))
    b = np.zeros((Ns,Nu))
    theta = np.linspace(-np.pi,0,Nu)
    for i in range(Ns):
        t0 = tk[i]
        t1 = tk[i+1]
        tvec = (t0+t1)/2+(t1-t0)/2*np.cos(theta)
        b[i,:] = np.interp(tvec,t,u)
        for j in range(Nu):
            A[i,j,i,:] = interp_covector(tvec[j],tk[i],tk[i+1],Nu)
    d= np.linalg.tensorsolve(A,b)
    print(d)
    d_mat = d
    

    # solve for x interpolation
    # A = np.zeros((Ns*Nc,Ns*Nx))   # A matrix for linear system A@c = b
    # B = np.interp(tc, t, x)
    # b = np.reshape(B,Ns*Nc)
    # for i in range(Ns):
    #     for j in range(Nc):
    #         A[Nc*i+j,Nx*i:Nx*(i+1)] = interp_covector(tc[i,j],tk[i],tk[i+1],Nx)

    # c,err,rank,vectors = np.linalg.lstsq(A,b,rcond=None)
    # c_mat = np.reshape(c,(Ns,Nx))

    # solve for u interpolation
    # A = np.zeros((Ns*Nc,Ns*Nu))   # A matrix for linear system A@d = B
    # B = np.interp(tc[:,:Nc], t, u)
    # b = np.reshape(B,Ns*Nc)
    # for i in range(Ns):
    #     for j in range(Nu):
    #         A[Nu*i+j,Nu*i:Nu*(i+1)] = interp_covector(tc[i,j],tk[i],tk[i+1],Nu)
    # d,err,rank,vectors = np.linalg.lstsq(A,b,rcond=None)
    # d_mat = np.reshape(d,(Ns,Nu))
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

        # get interpolation tensors
        T = self.collo_params.T
        dt = self.collo_params.dt

        # get state polynomial coefficients
        x_coeffs = np.reshape(z[:Ns*Nx], (Ns,Nx), order='F')
        xi_coeffs = x_coeffs[0,:]
        xdoti_coeffs = dt[0,:,:]@xi_coeffs
        xf_coeffs = x_coeffs[-1,:]
        xdotf_coeffs = dt[-1,:,:]@xf_coeffs

        return np.array([
            np.dot(T[0,0,:],xi_coeffs)-self.initial_cond[0],
            np.dot(T[0,0,:],xdoti_coeffs)-self.initial_cond[1],
            np.dot(T[-1,-1,:],xf_coeffs)-self.final_cond[0],
            np.dot(T[-1,-1,:],xdotf_coeffs)-self.final_cond[1],
        ])        

        # tc = self.collo_params.tc
        # c1 = z[0:Nx]
        # data = np.zeros(4)
        # I0 = interp_covector(tc[0,0],tc[0,0],tc[0,-1],Nx)
        # data[0] = (I0@c1)[0]-self.initial_cond[0]
        # I1 = interp_covector(tc[0,0],tc[0,0],tc[0,-1],Nx,ord=1)
        # data[1] = (I1@c1)[0]-self.initial_cond[1]
        # c2 = z[(Ns-1)*Nx:Ns*Nx]
        # I0 = interp_covector(tc[-1,-1],tc[-1,0],tc[-1,-1],Nx)
        # data[2] = (I0@c2)[0]-self.final_cond[0]
        # I1 = interp_covector(tc[-1,-1],tc[-1,0],tc[-1,-1],Nx,ord=1)
        # data[3] = (I1@c2)[0]-self.final_cond[1]

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
        Nc = self.collo_params.Nc
        Nx = self.collo_params.Nx
        tk = self.collo_params.tk

        # get differentiation tensors for knot points
        T0 = self.collo_params.T[:-1,-1,:]
        T1 = self.collo_params.T[1:,0,:]
        dt = self.collo_params.dt

        # reshape optimization variables to get coefficients of x interpolation
        x_coeffs = np.reshape(z[:Ns*Nx],(Ns,Nx),order='F')
        xdot_coeffs = np.einsum('ijk,ik->ij', dt, x_coeffs)

        # compute interpolations
        x0 = np.einsum('ij,ij->i',T0,x_coeffs[:-1,:])
        xdot0 = np.einsum('ij,ij->i',T0,xdot_coeffs[:-1,:])
        x1 = np.einsum('ij,ij->i',T1,x_coeffs[1:,:])
        xdot1 = np.einsum('ij,ij->i',T1,xdot_coeffs[1:,:])

        return np.hstack((x0-x1,xdot0-xdot1))

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
        Nc = self.collo_params.Nc

        # get interpolation tensors
        T = self.collo_params.T
        dt = self.collo_params.dt

        # get state polynomial coefficients
        x_coeffs = np.reshape(z[:Ns*Nx],(Ns,Nx),order='F')
        xdot_coeffs = np.einsum('ijk,ik->ij',dt,x_coeffs)
        xddot_coeffs = np.einsum('ijk,ik->ij',dt,xdot_coeffs)

        # get control polynomial coefficients
        u_coeffs = np.reshape(z[Ns*Nx:],(Ns,Nu),order='F')

        # Store defects in this array
        defects = np.zeros((Ns,Nc))

        # compute x, xdot, xddot, and u
        x = np.einsum('ijk,ik->ij',T,x_coeffs)
        xdot = np.einsum('ijk,ik->ij',T,xdot_coeffs)
        xddot = np.einsum('ijk,ik->ij',T,xddot_coeffs)
        u = np.einsum('ijk,ik->ij',T[:,:,:Nu],u_coeffs) 

        # compute residuals
        for (i,j) in np.ndindex((Ns,Nc)):
            defects[i,j] = stance_dynamics(x[i,j],xdot[i,j],u[i,j],self.dynamic_params)
        defects = defects - xddot
        return defects.flatten(order='F')

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
        Nc = self.collo_params.Nc
        T = self.collo_params.T
        Kx =  self.dynamic_params.Kx
        x0 = self.dynamic_params.x0
        lb = self.lb
        ub = self.ub
        x_coeffs = np.reshape(z[:Ns*Nx],(Ns,Nx),order='F')
        u_coeffs = np.reshape(z[Ns*Nx:],(Ns,Nu),order='F')

        # compute state and control interpolation
        x = np.einsum('ijk,ik->ij',T,x_coeffs)
        u = np.einsum('ijk,ik->ij',T[:,:,:Nu],u_coeffs)

        # compute motor torque
        tau = u + Kx*(x0-x)

        # compute bounds on state and motor torque
        data = np.zeros((Ns,Nc,4))
        data[:,:,0] = x - lb[0]
        data[:,:,1] = ub[0] - x
        data[:,:,2] = tau - lb[0]
        data[:,:,3] = ub[0] - tau

        return data.flatten(order='F') 

    def cost(self, z, Kv = .546, Kfudge=0.78, R = .29):
        Ns = self.collo_params.Ns
        Nx =  self.collo_params.Nx
        Nu = self.collo_params.Nu
        tk = self.collo_params.tk
        Kx =  self.dynamic_params.Kx
        x0 = self.dynamic_params.x0

        grad = np.zeros((Ns,Nx+Nu))
        x = np.reshape(z[:Ns*Nx], (Ns,Nx), order='F') 
        x_grad = np.zeros((Ns,Nx))
        u = np.reshape(z[Ns*Nx:], (Ns,Nu), order='F')
        u_grad = np.zeros((Ns,Nu))

        # product tensor, used to multiply two (Nx-1) degree polynomials
        # into a 2*(Nx-1) degree polynomial
        prod = np.zeros((2*Nx-1,Nx,Nx))
        for (i,j) in np.ndindex((Nx,Nx)):
            prod[i+j,i,j] = 1
        # tensor to integrate a 2*(Nx-1) degree polynomial over the spline domains
        I = np.vstack([interp_covector(0,tk[i],tk[i+1],2*Nx-1,ord=-1) for i in range(Ns)])
        # bilinear form composes integration with product
        prod_int = np.einsum('ij,jkl->ikl', I, prod)

        # differentiation tensor
        dt = self.collo_params.dt

        # torque
        tau = -Kx*x
        tau[:, :Nu] = u
        tau[:,0] += Kx*x0
        # tau_du = np.zeros((Ns,Nx,Nu))
        tau_dx = -Kx

        # velocity
        xdot = np.einsum('ijk,ik->ij', dt, x)
            
        # mechanical work
        # derivative with respect to torque
        mech_work_dtau = np.einsum('ijk,ik->ij', prod_int, xdot)
        mech_work = np.einsum('ij,ij->i', mech_work_dtau, tau)

        # derivative with respect to xdot
        mech_work_dxdot = np.einsum('ijk,ij->ik', prod_int, tau)

        # accumulate derivative with respect to x
        mech_work_dx = -Kx*mech_work_dtau
        x_grad += mech_work_dx
        x_grad += np.einsum('ij,ijk->ik', mech_work_dxdot, dt)

        # accumulate derivative with resupect to u
        u_grad += mech_work_dtau[:,:Nu]

        # thermal work
        current = tau / (Kv*Kfudge)
        voltage = current * R
        therm_work_dI = np.einsum('ijk,ik->ij', prod_int, voltage)
        therm_work = np.einsum('ij,ij->i', therm_work_dI, current)
        therm_work_dV = np.einsum('ijk,ij->ik', prod_int, voltage)

        # accumulate gradients
        current_dtau = 1/(Kv*Kfudge)
        voltage_dtau = R*current_dtau
        x_grad += therm_work_dI*current_dtau*tau_dx
        x_grad += therm_work_dV*voltage_dtau*tau_dx
        u_grad += therm_work_dI[:,:Nu]*current_dtau
        u_grad += therm_work_dV[:,:Nu]*voltage_dtau

        # pack the gradient
        grad[:,:Nx] = x_grad
        grad[:,Nx:] = u_grad

        return np.sum(mech_work+therm_work), np.reshape(grad, Ns*(Nx+Nu), order='F')

    def cost_noregen(self, 
                     z: np.ndarray, 
                     Kfudge = 0.78,
                     Kv = .546, R = .094, alpha=1.):


        """
        Note to self: this objective function currently only works where self.collo_params.Ns == 1
        """
        Nx =  self.collo_params.Nx
        Nu = self.collo_params.Nu
        tk = self.collo_params.tk
        a = tk[0]
        b = tk[-1]
        dt = (tk[-1]-tk[0])/100
        Kx =  self.dynamic_params.Kx
        x0 = self.dynamic_params.x0

        # unpack optimization variables into interp coefficients for x and u
        c = z[:Nx]
        d = z[Nx:]

        # compute and stack interpolation covectors, with row index corresponding to 
        # time, and column index corresponding to monomial terms
        tvec = np.arange(a,b,dt)
        A = np.vstack([interp_covector(t,a,b,Nx) for t in tvec])
        D = diff_tensor(a,b,Nx)
        A_diff = A@D

        # compute state and control
        x = A@c
        xdot = A_diff@c
        u = A[:,:Nu]@d
        torque = u + Kx*(x0-x)
        I = torque/(Kv*Kfudge)

        # compute losses for integration,
        # includes thermal power and positive mechanical power
        thermal_power = R*I**2
        mech_power = torque*xdot

        # positive mechanical power approximated via smooth max
        # mech_power_exp = np.exp(alpha*mech_power)
        # pos_mech_power = (mech_power*mech_power_exp)/(1+mech_power_exp)

        # positive electrical power approximated via smooth max
        electrical_power = thermal_power + mech_power
        electrical_power_exp = np.exp(alpha*electrical_power)
        pos_electrical_power = (electrical_power*electrical_power_exp)/(1+electrical_power_exp)
    
        # integrate losses and initialize memory for gradient calculation
        # _cost = np.trapz(thermal_power+pos_mech_power,x=tvec)
        # gradient calculation will be tricky
        # first need a linear operator to represent the trapezoidal integration
        trapz = dt*np.ones(tvec.shape[0])
        trapz[0] = trapz[-1] = dt/2 # end points have half weight in trapezoid integral

        _cost = np.dot(trapz, pos_electrical_power)
        _grad = np.zeros(z.shape)

        # gradient of torque/current wrt to interpolation coefficients
        dtau_dx = -Kx*A     # (M,Nx) array
        dI_dx = dtau_dx / (Kv*Kfudge)
        dtau_du = A[:,:Nu]  # (M,Nu) array
        dI_du = dtau_du / (Kv*Kfudge)

        # most of the following multiplications are broadcast along the time
        # axis - comments have been included to clarify some of these steps.

        # calculating gradient of thermal power
        dI2_dx = 2*dI_dx.T*I # (Nx,M)*(M,) -> (Nx,M)
        dI2_du = 2*dI_du.T*I # (Nu,M)*(M,) -> (Nu,M)

        # the following two operations reduce over the time axis
        _grad[:Nx] += R*(dI2_dx@trapz) # (Nx,M)@(M,) -> (Nx,)
        _grad[Nx:] += R*(dI2_du@trapz) # (Nu,M)@(M,) -> (Nu,)
        
        # calculating gradient of mechanical power
        # dM_dx = dtau_dx.T * xdot + (A_diff.T) * torque
        # dM_du = dtau_du.T * xdot # (Nu,M)*(M,) -> (Nu,M)
        dp_dx = R*dI2_dx + dtau_dx.T*xdot + (A_diff.T)*torque
        dp_du = R*dI2_du + dtau_du.T*xdot # (Nu,M) + ((Nu,M) * (M,)) -> (Nu,M)

        # now the tricky part - differentiating through the smoothmax
        # dsm_dM = mech_power_exp * (1 + alpha*mech_power + mech_power_exp) / (1+mech_power_exp)**2
        # dsm_dx = dM_dx * dsm_dM # (Nx,M)*(M,) -> (Nx,M)
        # dsm_du = dM_du * dsm_dM # (Nu,M)*(M,) -> (Nu,M)
        dsm_dp = electrical_power_exp * (1+alpha*electrical_power + electrical_power_exp)
        dsm_dp /= (1+electrical_power_exp)**2
        dsm_dx = dp_dx * dsm_dp # (Nx,M)*(M,) -> (Nx,M)
        dsm_du = dp_du * dsm_dp # (Nu,M)*(M,) -> (Nu,M)

        # the following two operations reduce over the time axis
        _grad[:Nx] += dsm_dx @ trapz   # (Nx,M)@(M,) -> (Nx,)
        _grad[Nx:] += dsm_du @ trapz   # (Nu,M)@(M,) -> (Nu,)

        return _cost, _grad

    def optimize(self, initial_guess, options={}, regeneration=True):
        if regeneration:
            cost = self.cost
        else:
            cost = self.cost_noregen
        self.result = minimize(cost, initial_guess, method='SLSQP', jac=True, 
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
        self.c_mat = np.reshape(self.result.x[:Ns*Nx],(Ns,Nx),order='F')
        self.d_mat = np.reshape(self.result.x[Ns*Nx:],(Ns,Nu),order='F')

        return self.result

