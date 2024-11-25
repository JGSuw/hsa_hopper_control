from .kinematics import KinematicParameters, forward_kinematics
from .hsa_model import HSAModel
import numpy as np
g = 9.81
_FLIGHT_MODE = 1
_STANCE_MODE = 2
class DynamicsParameters:
    def __init__(self,
            m: float,
            J: float,
            bx: float,
            by: float,
            Kx: float,
            x0: float,
            kinematics: KinematicParameters,
            hsa_model = None,
            psi = None
    ):
        self.m, self.J, self.bx, self.by, self.Kx, self.x0, self.psi = m,J,bx,by,Kx,x0,psi
        self.kinematics = kinematics
        self.hsa_model = hsa_model
    
    def attribute_dict(self):
        attributes = {}
        for key in self.__dict__:
            value = self.__dict__[key]
            if type(value) == HSAModel or type(value) == KinematicParameters:
                attributes[key] = value.attribute_dict()
            else:
                attributes[key] = value
        return attributes

def evaluate(position_rad: float, 
             velocity_rad: float, 
             u: float, 
             params: DynamicsParameters):
    
    '''
    Compute the stance dynamics of the hopper as second-order ODE, i.e.
    x_ddot = f(x,x_dot,u).

        Parameters:
            position_rad (float): motor angle in radians relative to the calibration
            velocity_rad (float): motor velocity in radians
            u (float): motor torque in Nm
            params (DynamicsParameters): contains all the model information

        Returns:
            acceleration_rad (float): angular acceleration in rad/s**2
    '''
    
    f,df,d2f = forward_kinematics(params.kinematics, position_rad, jacobian=True, hessian=True)
    y, l = f[0], f[1]
    dy, dl = df[0], df[1]
    ldot = dl*velocity_rad
    ydot = dy*velocity_rad
    d2y = d2f[0]
    # inertia = (params.J+params.m*dy**2)
    inertia = params.J
    # T = 1/2 m * (dy * xdot)**2
    # dT/dxdot = m * dy * dy * xdot
    # d/dt(dT/dxdot) = m*dy*dy*xddot + 2*m*dy*xdot*d2y*xdot
    # d/dx T = m*dy*xdot*d2y*xdot
    # coriolis = params.m*(ydot)*(d2y*velocity_rad)
    coriolis = 0
    rayleigh = (params.bx+params.by*dy**2)*velocity_rad
    potential = params.Kx*(position_rad-params.x0) + params.m*g*dy
    if params.hsa_model is not None:
        z = np.array([l,params.psi])
        zdot = np.array([ldot, 0])
        potential += dl*(params.hsa_model.dV(z)+params.hsa_model.dR(z,zdot))
    acceleration_rad = (u-potential-coriolis-rayleigh)/inertia
    return acceleration_rad

