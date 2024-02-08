from hsa_hopper import *
import math
import yaml
import numpy as np
import asyncio
import os
import time

class HopController():
    _PULL = 0
    _DEADZONE1 = 1
    _PUSH = 2
    _DEADZONE2 = 3
    def __init__(self,
                 kinematics: KinematicParameters,
                 kp_pull: float,
                 kd_pull: float,
                 u_pull: float,
                 kp_push: float,
                 kd_push: float,
                 u_push: float,
                 L0: float,
                 L1: float,
                 y01: float,
                 y12: float,
                 y23: float,
                 y30: float,
                #  d1: float,
                #  d2: float, 
                 kpx: float,
                 kdx: float,
                 t_window: list,
                 x_window: list
                 ):
        self.kinematics = kinematics
        self.kp_pull = kp_pull
        self.kd_pull = kd_pull
        self.u_pull  = u_pull 
        self.kp_push = kp_push
        self.kd_push = kd_push
        self.u_push  = u_push 
        self.L0 = L0
        self.L1 = L1
        # self.d1      = d1  
        # self.d2      = d2
        self.y01 = y01
        self.y12 = y12
        self.y23 = y23
        self.y30 = y30

        self.pull_controller = PDController(
                kp_pull, 
                kd_pull, 
                kp_pull*L0+u_pull, 
                kpx, kdx, 
                t_window, 
                x_window)

        self.push_controller = PDController(
                kp_push, 
                kd_push, 
                kp_push*L1+u_push, 
                kpx, kdx, 
                t_window, 
                x_window)

        self.mode = HopController._PULL

    def update(self, motor_pos_rad, t):

        self.pull_controller.update_window(motor_pos_rad, t)
        self.push_controller.update_window(motor_pos_rad, t)

        if self.mode == HopController._PULL:
            print('MODE PULL')
            interp = self.pull_controller.interpolate()
            f,J,H = forward_kinematics(self.kinematics, interp)
            if f[0] < self.y01:
                print('SWITCH TO DEADZONE 2')
                self.mode = HopController._DEADZONE2
                return interp, 0., 0., 0.
            else:
                return self.pull_controller.gain_conversion(interp, f[0], J[0], H[0])
            
        elif self.mode == HopController._DEADZONE2:
            print('MODE DEADZONE 2')
            interp = self.push_controller.interpolate()
            f,J,H = forward_kinematics(self.kinematics, interp)
            if f[0] > self.y12:
                print('SWITCH TO PUSH')
                self.mode = HopController._PUSH
                return self.push_controller.gain_conversion(interp, f[0], J[0], H[0])
            else:
                return interp, 0., 0., 0.
            
        elif self.mode == HopController._PUSH:
            print('MODE PUSH')
            interp = self.push_controller.interpolate()
            f,J,H = forward_kinematics(self.kinematics, interp)
            if f[0] > self.y23:
                print('SWITCH TO DEADZONE 1')
                self.mode = HopController._DEADZONE1
                return interp, 0., 0., 0.
            else:
                return self.push_controller.gain_conversion(interp, f[0], J[0], H[0])
            
        elif self.mode == HopController._DEADZONE1:
            print('MODE DEADZONE 1')
            interp = self.pull_controller.interpolate()
            f,J,H = forward_kinematics(self.kinematics, interp)
            if f[0] < self.y30:
                print('SWITCH TO PULL')
                self.mode = HopController._PULL
                return self.pull_controller.gain_conversion(interp, f[0], J[0], H[0])
            else:
                return interp, 0., 0., 0.

async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)

    # hsa setpoint
    hsa_setpoint = 1500
    robot.servo.write_setpoint(hsa_setpoint)

    # get motor gains
    kpx, kdx = await robot.motor.get_pd_gains()

    # compute task space setpoint
    f,J,H = robot.forward_kinematics(-math.pi/4)
    y0 = f[0]

    # task space gain parameters
    kpy = 200.
    kdy = 1.
    uy = y0*kpy

    # read 3 states from the motor to initialize pd controller
    t0 = time.perf_counter()*1e3
    t_window = []
    x_window = []
    for i in range(3):
        motor_state = await robot.motor.get_state()
        t = time.perf_counter()*1e3-t0
        pos, vel = robot.convert_motor_posvel(motor_state)
        t_window.append(t)
        x_window.append(pos)

    # construct HopController
    
    # using zero gains and controls to test state machine
    # hop_controller = HopController(
    #     robot.kinematics,
    #     0.,     
    #     0.,     
    #     0.,     
    #     0.,     
    #     0.,     
    #     0.,      
    #     .19-.06,     # ymin
    #     .19,        # ymax
    #     .01,         # deadzone 1 length
    #     .01,         # deadzone 2 length
    #     kpx,        # motor kp (Nm/rad)
    #     kpy,        # motor kd (Nm/rad) 
    #     t_window,   # time measurements (for interpolation)
    #     x_window    # angle measurements (rad, for interpolation)
    # )

    hop_controller = HopController(
        robot.kinematics,
        25.,       # kp_pull
        .25,        # kd_pull
        -10.,      # u_pull
        50.,       # kp_push
        .5,        # kd_push
        35.,       # u_push
        .125,      # ymin
        .19,       # ymax
        .135,
        .135,
        .185,
        .16,
        kpx,        # motor kp (Nm/rad)
        kpy,        # motor kd (Nm/rad) 
        t_window,   # time measurements (for interpolation)
        x_window    # angle measurements (rad, for interpolation)
    )

    hop_controller.mode = HopController._PUSH
    motor_state = None
    while motor_state is None:
        motor_state = await robot.motor.get_state()
        t = time.perf_counter()*1e3-t0
    x, xdot = robot.convert_motor_posvel(motor_state)
    position, kp_scale, kd_scale, torque = hop_controller.update(x, t)

    # try running this for a short time
    while (time.perf_counter()-t0/1e3) < 5.:
        motor_state = await robot.motor.set_position(
            position = position,
            kp_scale = kp_scale,
            kd_scale = kd_scale,
            feedforward_torque = torque,
            query = True
        )
        print(torque)
        if motor_state is not None:
            t = time.perf_counter()*1e3-t0
            x, xdot = robot.convert_motor_posvel(motor_state)
            f,J,H = forward_kinematics(robot.kinematics, x)
            print(f[0])
            try:
                position, kp_scale, kd_scale, torque = hop_controller.update(x, t)
            except BaseException as e:
                print("failed to udpate controller, singular interpolation matrix likely")
                print(str(e))
    await robot.motor.controller.set_stop()

if __name__ == "__main__":
    asyncio.run(main())
    