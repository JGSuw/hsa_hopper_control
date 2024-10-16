import numpy as np

class HopController():
    _STARTUP = 0
    _FLIGHT = 1
    _STANCE = 2
    def __init__(self,
                 kp: np.ndarray,
                 kd: np.ndarray,
                 x0_rad: np.ndarray,
                 u_interp: np.ndarray,
                 x_td_rad: float,
                 x_lo_rad: float,
                 ):
        """
        Implements the control law for hopping, a combination of 
        motor angle PD control with optimized feed-forward torque.

            Inputs:
                kp (np.ndarray): (3,) array of proportional gains on motor angle,
                    one per control mode (startup, stance, flight).
                kd (np.ndarray): (3,) array of derivative gains on motor angle,
                    one per control mode.
                x0 (np.ndarray): (3,) array of angle setpoints for proportional control,
                    one per control mode.
                u_interp (list of PiecewiseInterpolation): list of feed-forward torque
                    interpolation functions, one per control mode.
                x_td (float): guard angle for transitioning between modes (0->1->2->1->2 etc)
        """
        self.initialized = False
        self.mode = None
        self.t0_s = None
        self.kp = kp
        self.kd = kd
        self.x0_rad = x0_rad
        self.u_interp = u_interp
        self.x_td_rad = x_td_rad
        self.x_lo_rad = x_lo_rad

    def initialize(self, t0_s: float):
        """
        Sets the mode of this controller to HopController._STARTUP, and sets the 
        time datum to t0_s (time in seconds).
        """
        self.mode = HopController._STARTUP
        self.t0_s = t0_s
        self.initialized = True

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
        if (not self.initialized):
            raise RuntimeError('Cannot update HopController before initialization')
        elif self.mode == HopController._STARTUP:
            if x_rad <= self.x_lo_rad:
                self.mode = HopController._FLIGHT
                self.t0_s = t_s
        elif self.mode == HopController._FLIGHT:
            if x_rad >= self.x_td_rad:
                self.mode = HopController._STANCE
                self.t0_s = t_s
        elif self.mode == HopController._STANCE:
            if x_rad <= self.x_lo_rad:
                self.mode = HopController._FLIGHT
                self.t0_s = t_s
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')

    def output(self, t_s: float):
        """
        Returns the controller gains, setpoint, and feed-forward torque
        corresponding to t_s - self.t0_s and the current mode.

            Inputs:
                t_s (float): precise system time for when message is expected
                    to be received by Moteus controller.

            Returns:
                kp (float): proportional gain term
                kd (float): derivative gain term
                x0_rad (float): proportional control setpoint in radians,
                    relative to the motor calibration (must be shifted and converted to revolutions!)
                u_ff (float): feed-forward torque in N/m.
        """
        if not (self.initialized):
            raise RuntimeError('Cannot execute HopController.output before initialization')
        elif self.mode == HopController._STARTUP:
            pass
        elif self.mode == HopController._FLIGHT:
            pass
        elif self.mode == HopController._STANCE:
            pass
        else:
            raise RuntimeError(f'Invalid value self.mode={self.mode} encountered in update.')
        kp = self.kp[self.mode]
        kd = self.kd[self.mode]
        x0_rad = self.x0_rad[self.mode]
        if self.u_interp[self.mode] != None:
            u_ff = self.u_interp[self.mode].evaluate(t_s - self.t0_s)
        else:
            u_ff = 0.
        return kp, kd, x0_rad, u_ff


