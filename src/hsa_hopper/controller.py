import numpy as np

class HopController():
    _STARTUP = 0
    _FLIGHT = 1
    _STANCE0 = 2
    _STANCE1 = 3
    def __init__(self,
                 kp: float,         # Proportional gain in stance
                 kd: float,         # Derivative gain in stance
                 x0_rad: float,     # Angle spring equilibrium in stance
                 u_ff: float,       # push-off torque during _STANCE1
                 x_td_rad: float,   # motor angle switching from _FLIGHT to _STANCE0
                 x_lo_rad: float,   # motor angle switching from _STANCE1 to _FLIGHT and _STARTUP to _FLIGHT
                 window: int,       # window size for velocity estimation via savgol of quadratic interpolation
                 ):
        self.mode = None
        self.t0_s = None
        self.kp = kp
        self.kd = kd
        self.x0_rad = x0_rad
        self.u_ff = u_ff
        self.x_td_rad = x_td_rad
        self.x_lo_rad = x_lo_rad
        self.tdata = np.zeros(window)
        self.xdata = np.zeros(window)
        self.A_mat = np.zeros((window,3))
        self.A_mat[:,0] = np.ones(window)

    def pushoff_switch_condition(self):
        # get a quadratic fit to (tdata, xdata)
        N = self.tdata.shape[0]//2
        dt = self.tdata-self.tdata[N]
        self.A_mat[:,1] = dt
        self.A_mat[:,2] = dt**2
        coeffs, residuals, rank, s = np.linalg.lstsq(self.A_mat, self.xdata, rcond=-1)
        # compute velocity before and after the midpoint
        xdotm1 = coeffs[1] + 2*dt[N-1]*coeffs[2]
        xdotp1 = coeffs[1] + 2*dt[N+1]*coeffs[2]
        return xdotm1 > 0 and xdotp1 < 0

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
        self.xdata[0:-2] = self.xdata[1:]
        self.xdata[-1] = x_rad
        if self.mode == HopController._STARTUP:
            if x_rad <= self.x_lo_rad:
                self.mode = HopController._FLIGHT
                self.t0_s = t_s
        elif self.mode == HopController._FLIGHT:
            if x_rad >= self.x_td_rad:
                self.mode = HopController._STANCE0
        elif self.mode == HopController._STANCE0:
            if self.pushoff_switch_condition():
                self.mode = HopController._STANCE1
        elif self.mode == HopController._STANCE1:
            if x_rad <= self.x_lo_rad:
                self.mode = HopController._FLIGHT
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')

    def output(self):
        if self.mode == HopController._STARTUP:
            return self.kp, self.kd, self.u_ff, self.x0_rad
        elif self.mode == HopController._FLIGHT:
            return 0, 0, 0, self.xdata[-1]
        elif self.mode == HopController._STANCE0:
            return self.kp, self.kd, 0, self.x0_rad
        elif self.mode == HopController._STANCE1:
            return self.kp, self.kd, self.u_ff, self.x0_rad
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')


