import asyncio
import numpy as np
from hsa_hopper.constants import _RAD_TO_DEG, _REV_TO_DEG, _REV_TO_RAD

class PDController():
    def __init__(self, kpy, kdy, uy, kpx, kdx, t_init, x_init):
        self.kpy = kpy
        self.kdy = kdy
        self.uy = uy
        self.kpx_config = kpx
        self.kdx_config = kdx
        self.t_window = np.array(t_init)
        self.x_window = np.array(x_init)

    # quadratic interpolation to value of theta inbetween now and next transaction
    def interpolate(self):
        t0 = self.t_window[0]
        t1 = self.t_window[1]-t0
        t2 = self.t_window[2]-t0
        x0 = self.x_window[0]
        x1 = self.x_window[1]
        x2 = self.x_window[2]

        tm1 = (t1)/2
        tm13, tm12 = tm1**3, tm1**2
        tm2 = t1+(t2-t1)/2
        tm23, tm22 = tm2**3, tm2**2

        A = np.array(
            [[0, 0, 0, 0, 1, 0],
             [t1**2, 0, t1, 0, 1, 0],
             [0, t1**2, 0, t1, 0, 1],
             [0, t2**2, 0, t2, 0, 1],
             [2*t1, -2*t2, 1, -1, 0, 0],
             [(-tm13)/3, -(tm23-tm13)/3, (tm22-tm12)/2, -(tm22-tm12)/2, tm2-tm1, tm1-tm2]
        ])
        b = np.array([x0, x1, x1, x2, 0, 0])
        coeffs = np.linalg.solve(A,b)
        t = 3/2*t2
        x = coeffs[1]*t**2 + coeffs[3]*t + coeffs[5]
        return x
    
    def gain_conversion(self, x_rad, f, df, d2f):
        ux = df*self.uy - df*self.kpy*f
        kpx = d2f*(self.kpy*f-self.uy) + df*self.kpy*df
        kdx = df*self.kdy*df
        kp_scale = kpx/self.kpx_config
        kd_scale = kdx/self.kdx_config
        return (x_rad/_REV_TO_RAD, kp_scale, kd_scale, ux)
    
    def update_window(self, x_rad, t):
        self.x_window[0] = self.x_window[1]
        self.x_window[1] = self.x_window[2]
        self.x_window[2] = x_rad
        self.t_window[0] = self.t_window[1]
        self.t_window[1] = self.t_window[2]
        self.t_window[2] = t