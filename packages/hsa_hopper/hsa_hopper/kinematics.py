import math
import numpy as np

class KinematicParameters:
    def __init__(self, 
                 a: float,  # femur length, pin joint to pin joint
                 b: float,  # tibia lnegth, pin joint to pin joint
                 c: float,  # rod length, pin joint to end cap
                 d:float ): # vertical offset, motor axis to HSA base
        self.a = a
        self.b = b
        self.c = c
        self.d = d

# Computes forward kinematics, jacobian, and hessian.
def forward_kinematics(p: KinematicParameters, theta: float, jacobian = False, hessian = False):
    c_theta = math.cos(theta)
    s_theta = math.sin(theta)
    x = p.a/p.b*c_theta

    phi = math.acos(x)
    c_phi = math.cos(phi)
    s_phi = math.sin(phi)
    y = -p.a*s_theta+p.b*s_phi
    L = p.c - y - p.d
    if not jacobian and not hessian:
        return np.array([y, L])
    

    dx_dtheta = -p.a/p.b*s_theta
    dphi_dx = -1/math.sqrt(1-x**2)
    dphi_dtheta = dphi_dx*dx_dtheta
    dy_dtheta = -p.a*c_theta + p.b*c_phi*dphi_dtheta
    if not hessian:
        return np.array([y, L]), np.array([dy_dtheta, -dy_dtheta])

    d2x_d2theta = -p.a/p.b*c_theta
    d2phi_d2x = -x/(1-x**2)**(3/2)
    d2phi_d2theta = (d2phi_d2x*dx_dtheta)*dx_dtheta + dphi_dx*d2x_d2theta

    dy_dtheta = -p.a*c_theta + p.b*c_phi*dphi_dtheta
    d2y_d2theta = p.a*s_theta - p.b*s_phi*dphi_dtheta**2 + p.b*c_phi*d2phi_d2theta
    return np.array([y, p.c - y - p.d]), np.array([dy_dtheta, -dy_dtheta]), np.array([d2y_d2theta, -d2y_d2theta])