import numpy as np

class HopController():
    _STARTUP = 0
    _FLIGHT = 1
    _STANCE0 = 2
    _STANCE1 = 3
    def __init__(self,
                 kp: list,          # Proportional gain in stance
                 kd: list,          # Derivative gain in stance
                 x0_rad: list,      # Angle spring equilibrium in stance
                 u_ff: list,        # push-off torque during _STANCE1
                 sigma: float,      # flight/stance switching position
                 window: int,       # window size for velocity estimation via savgol of quadratic interpolation
                 ):
        self.mode = None
        self.t0_s = None
        self.kp = kp
        self.kd = kd
        self.x0_rad = x0_rad
        self.u_ff = u_ff
        self.sigma = sigma
        self.N = window//2
        self.tdata = np.zeros(window)
        self.xdata = np.zeros(window)
        self.A_mat = np.zeros((window,3))
        self.A_mat[:,0] = np.ones(window)

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
    
    def flight_switch_condition(self):
        coeffs, dt = self.quad_fit()
        return coeffs[0] <= self.sigma and coeffs[1] < 0
    
    def stance_switch_condition(self):
        coeffs, dt = self.quad_fit()
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
        if self.mode == HopController._STARTUP:
            if self.flight_switch_condition():
                self.mode = HopController._FLIGHT
                self.t0_s = t_s
        elif self.mode == HopController._FLIGHT:
            if self.stance_switch_condition():
                self.mode = HopController._STANCE0
        elif self.mode == HopController._STANCE0:
            if self.pushoff_switch_condition():
                self.mode = HopController._STANCE1
        elif self.mode == HopController._STANCE1:
            if self.flight_switch_condition():
                self.mode = HopController._FLIGHT
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')

    def output(self):
        if self.mode == HopController._STARTUP:
            return self.kp[0], self.kd[0], self.x0_rad[0], self.u_ff[0]
        elif self.mode == HopController._FLIGHT:
            return self.kp[1], self.kd[1], self.x0_rad[0], self.u_ff[1]
        elif self.mode == HopController._STANCE0:
            return self.kp[2], self.kd[2], self.x0_rad[0], self.u_ff[2]
        elif self.mode == HopController._STANCE1:
            return self.kp[3], self.kd[3], self.x0_rad[0], self.u_ff[3]
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')


