import numpy as np
from hsa_hopper.collocation import PiecewiseInterpolation

class HopController():
    _STARTUP = 0
    _FLIGHT = 1
    _STANCE = 2
    def __init__(self,
                 kp: float,         # tracking error propertional gain
                 kd: float,         # tracking error derivative gain
                 kx: list,          # virtual (motor angle) spring constant
                 x0: list,          # virtual (motor angle) spring offset
                 b: list,           # virtual (motor angle) damping coefficient
                 x_interp: list,    # reference trajectory for each mode
                 u_interp: list,    # reference command for each mode
                 sigma: float,      # flight/stance switching position
                 window: int,       # window size for velocity estimation via savgol of quadratic interpolation
                 ):
        self.mode = None
        self.t0_s = None
        self.kp = kp
        self.kd = kd
        self.kx = kx
        self.x0 = x0
        self.b = b
        self.x_interp = x_interp
        self.u_interp = u_interp
        self.sigma = sigma
        self.N = window//2
        self.tdata = np.zeros(window)
        self.xdata = np.zeros(window)
        self.A_mat = np.zeros((window,3))
        self.A_mat[:,0] = np.ones(window)
        self.x_error = 0
        self.xdot_error = 0
        self.u_ff = 0

    def initialize(self, mode, t0_s):
        self.mode = mode
        self.t0_s = t0_s

    def quad_fit(self):
        dt = self.tdata-self.tdata[self.N]
        self.A_mat[:,1] = dt
        self.A_mat[:,2] = dt**2
        coeffs, residuals, rank, s = np.linalg.lstsq(self.A_mat, self.xdata, rcond=-1)
        return coeffs, dt

    def pushoff_switch_condition(self):
        coeffs, dt = self.quad_fit()
        xdotm1 = coeffs[1] + 2*dt[self.N-1]*coeffs[2]
        xdotp1 = coeffs[1] + 2*dt[self.N+1]*coeffs[2]
        return xdotm1 > 0 and xdotp1 < 0
    
    def flight_switch_condition(self, coeffs):
        return coeffs[0] <= self.sigma and coeffs[1] < 0
    
    def stance_switch_condition(self, coeffs):
        return coeffs[0] >= self.sigma and coeffs[1] > 0

    def update(self, x_rad: float, t_s: float):
        """
        Updates the controller state (mode, t0_s) according to x_rad and previous
        controller state. x_rad is compared to x_td_rad, 
        and causes change to self.mode and update of self.t0_s if x_rad crosses
        x_td_rad in the direction corresponding to self.mode.

        Inputs: 
            x_rad (float): motor angle relative to calibration position in radians.
            t_s (float): precise system time when x_rad was received.
        """
        self.tdata[0:-1] = self.tdata[1:]
        self.tdata[-1] = t_s
        self.xdata[0:-1] = self.xdata[1:]
        self.xdata[-1] = x_rad
        coeffs, dt = self.quad_fit()
        # check guard conditions to switch modes
        if self.mode == HopController._STARTUP:
            if self.flight_switch_condition():
                self.mode = HopController._FLIGHT
                self.t0_s = t_s
        elif self.mode == HopController._FLIGHT:
            if self.stance_switch_condition():
                self.mode = HopController._STANCE
                self.t0_s = t_s
        elif self.mode == HopController._STANCE:
            if self.flight_switch_condition():
                self.mode = HopController._FLIGHT
                self.t0_s = t_s
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')

        # compute controls
        if self.mode == HopController._STARTUP:
            x_interp = self.x_interp[HopController._STARTUP]
            u_interp = self.x_unterp[HopController._STARTUP]
            x_ref = x_interp.evaluate(t_s-self.t0_s)
            self.x_error = x_error = x_ref - coeffs[0]
            xdot_ref = x_interp.evaluate(t_s-self.t0_s,ord=1)
            self.xdot_error = xdot_error = xdot_ref - coeffs[1]
            self.u_ff = self.kp*x_error + self.kd*xdot_error
            self.u_ff += u_interp.evaluate(t_s-self.t0_s)
        elif self.mode == HopController._FLIGHT:
            x_interp = self.x_interp[HopController._FLIGHT]
            self.u_ff = 0
        elif self.mode == HopController._STANCE:
            x_interp = self.x_interp[HopController._STANCE]
            u_interp = self.x_unterp[HopController._STANCE]
            x_ref = x_interp.evaluate(t_s-self.t0_s)
            self.x_error = x_error = x_ref - coeffs[0]
            xdot_ref = x_interp.evaluate(t_s-self.t0_s,ord=1)
            self.xdot_error = xdot_error = xdot_ref - coeffs[1]
            self.u_ff = self.kp*x_error + self.kd*xdot_error
            self.u_ff += u_interp.evaluate(t_s-self.t0_s)
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')

    def output(self, t_s):
        return self.kx[self.mode], self.b[self.mode], self.x0[self.mode], self.u_ff


