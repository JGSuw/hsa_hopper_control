from .kinematics import KinematicParameters, forward_kinematics
from .hsa_model import HSAModel 
import numpy as np
g = 9.81
_FLIGHT_MODE = 1
_STANCE_MODE = 2
class DynamicsParameters:
    def __init__(self,
            J: float,
            m_cart: float,
            m_foot: float,
            bx: float,
            by: float,
            bl: float,
            Kx: float,
            x0: float,
            kinematics: KinematicParameters,
            hsa_model = None,
            psi = None
    ):
        self.J, self.m_cart, self.m_foot = J, m_cart, m_foot 
        self.bx, self.by, self.bl = bx, by, bl
        self.Kx, self.x0, self.psi = Kx, x0, psi
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

def stance_dynamics(
            x: float, 
            xdot: float, 
            u: float, 
            params: DynamicsParameters):
    
    '''
    Compute the stance dynamics of the hopper as second-order ODE, i.e.
    x_ddot = f(x,x_dot,u).

        Parameters:
            x (float): motor angle in radians relative to the calibration, in radians
            xdot (float): motor velocity in radians
            u (float): motor torque in Nm
            params (DynamicsParameters): contains all the model information

        Returns:
            acceleration_rad (float): angular acceleration in rad/s**2
    '''
    
    f,df,d2f = forward_kinematics(params.kinematics, x, jacobian=True, hessian=True)
    y, l = f[0], f[1]
    dy, dl = df[0], df[1]
    ldot = dl*xdot
    d2y = d2f[0]
    inertia = (params.J+params.m_cart*dy**2)
    coriolis = params.m_cart*(dy*xdot)*(d2y*xdot)
    rayleigh = (params.bx+params.by*dy**2+params.bl*dl**2)*xdot
    potential = params.Kx*(x-params.x0) + params.m_cart*g*dy
    if params.hsa_model is not None:
        spring_torque = params.hsa_model.spring_force(l,params.psi)*dl
        dissipation_torque = params.hsa_model.dissipation_force(l,ldot,params.psi)*dl
    else:
        spring_torque = 0
        dissipation_torque = 0
    acceleration_rad = (u-spring_torque-dissipation_torque-potential-coriolis-rayleigh)/inertia
    return acceleration_rad

def flight_dnyamics(
            x: float, 
            xdot: float, 
            u: float, 
            params: DynamicsParameters):
    
    '''
    Compute the stance dynamics of the hopper as second-order ODE, i.e.
    x_ddot = f(x,x_dot,u).

        Parameters:
            x (float): motor angle in radians relative to the calibration, in radians
            xdot (float): motor velocity in radians
            u (float): motor torque in Nm
            params (DynamicsParameters): contains all the model information

        Returns:
            acceleration_rad (float): angular acceleration in rad/s**2
    '''
    
    f,df,d2f = forward_kinematics(params.kinematics, x, jacobian=True, hessian=True)
    y, l = f[0], f[1]
    dy, dl = df[0], df[1]
    ldot = dl*xdot
    d2y = d2f[0]
    inertia = (params.J+params.m_foot*dl**2)
    coriolis = params.m_cart*(dy*xdot)*(d2y*xdot)
    rayleigh = (params.bx+params.by*dy**2+params.bl*dl**2)*xdot
    potential = params.Kx*(x-params.x0) + params.m_foot*g*dl
    if params.hsa_model is not None:
        spring_torque = params.hsa_model.spring_force(l,params.psi)*dl
        dissipation_torque = params.hsa_model.dissipation_force(l,ldot,params.psi)*dl
    else:
        spring_torque = 0
        dissipation_torque = 0
    acceleration_rad = (u-spring_torque-dissipation_torque-potential-coriolis-rayleigh)/inertia
    return acceleration_rad