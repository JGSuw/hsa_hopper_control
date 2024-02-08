from hsa_hopper import *
from hsa_hopper.constants import _REV_TO_DEG, _RAD_TO_DEG
import math
import yaml
import numpy as np
import asyncio
import os
import time
import datetime
import pandas as pd
from collections import deque

class TrajectoryData():
    def __init__(self):
        self.theta_deg = deque()          # hsa_hopper.Robot QDD coordinates
        self.phi_deg = deque()            # moteus.controller QDD coordinates
        self.y = deque()                  # distance from body to foot
        self.L = deque()                  # HSA length
        self.dy = deque()                 # derivative of y wrt to theta
        self.d2y = deque()                # second derivative of y wrt to theta
        self.mode = deque()               # mode of the HopController
        self.theta_setpoint_deg = deque() # commanded setpoint for QDD's internal PID
        self.kp_theta = deque()           # commanded proportional gain for QDD's internal PID, Nm/rad
        self.kd_theta = deque()           # commanded derivative gain for QDD's internal PID, (Nm*seconds)/(rad)
        self.u_theta = deque()            # commanded feedforward torque for QDD's internal PID (Nm)
        self.u_y = deque()
        self.energy = deque()
        self.tvec = deque()               # time query and PID update (local CPU, seconds),

    def append(self, 
        theta_deg,
        phi_deg,
        y,
        L,
        dy,
        d2y,
        mode,
        theta_setpoint_deg,
        kp_theta,
        kd_theta,
        u_theta,
        u_y,
        energy,
        t
    ):
       self.theta_deg.append(theta_deg)
       self.phi_deg.append(phi_deg)
       self.y.append(y)
       self.L.append(L)
       self.dy.append(dy)
       self.d2y.append(d2y)
       self.mode.append(mode)
       self.theta_setpoint_deg.append(theta_setpoint_deg)
       self.kp_theta.append(kp_theta)
       self.kd_theta.append(kd_theta)
       self.u_theta.append(u_theta)
       self.u_y.append(u_y)
       self.energy.append(energy)
       self.tvec.append(t)

    def to_dataframe(self):
        return pd.DataFrame( data = {
            'theta_deg' : self.theta_deg,
            'phi_deg' : self.phi_deg,
            'y' : self.y,
            'L' : self.L,
            'dy' : self.dy,
            'd2y' : self.d2y,
            'mode' : self.mode,
            'kp_theta' : self.kp_theta,
            'kd_theta' : self.kd_theta,
            'u_theta' : self.u_theta,
            'u_y' : self.u_y,
            'energy' : self.energy,
            't' : self.tvec
        })

class HopController():
    _PULL = 0
    _DEADZONE1 = 1
    _PUSH = 2
    _DEADZONE2 = 3
    def __init__(self,
                 kinematics: KinematicParameters,
                y_gains: np.matrix,
                y_torques: np.array,
                y_offsets: np.array,
                y_guards: np.matrix,
                kpx: float,
                kdx: float,
                t_window: list,
                x_window: list
                 ):
        self.kinematics = kinematics
        self.y_guards = y_guards
        self.controllers = [
            PDController(
                y_gains[i,0], y_gains[i,1],
                y_torques[i] + y_gains[i,0] * y_offsets[i],
                kpx, kdx,
                t_window, x_window
            ) for i in range(4)
        ]
        self.mode = HopController._PULL

    def update(self, motor_pos_rad, t):
        for controller in self.controllers:
            controller.update_window(motor_pos_rad, t)

        interp = self.controllers[self.mode].interpolate()
        f,J,H = forward_kinematics(self.kinematics, interp)
        a,b = self.y_guards[self.mode, 0], self.y_guards[self.mode,1]
        reset = a*f[0] + b < 0
        if reset:
            if self.mode == HopController._PULL:
                print('Switch to DEADZONE1')
                self.mode = HopController._DEADZONE1

            elif self.mode == HopController._DEADZONE1:
                print('Switch to PUSH')
                self.mode = HopController._PUSH

            elif self.mode == HopController._PUSH:
                print('Switch to DEADZONE2')
                self.mode = HopController._DEADZONE2

            elif self.mode == HopController._DEADZONE2:
                print('Switch to PULL')
                self.mode = HopController._PULL
        
        controller = self.controllers[self.mode]
        return controller.gain_conversion(interp, f[0], J[0], H[0])

async def main(config_path):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 

    with open(os.path.join(root_folder, config_path), 'r') as f:
        experiment_config = yaml.load(f,yaml.Loader)
    
    hardware_config_path = os.path.join(root_folder, experiment_config['hardware'])
    robot = Robot(hardware_config_path)

    controller_config = experiment_config['controller']

    # hsa setpoint
    robot.servo.write_setpoint(int(controller_config['servo_pos']))

    # get motor gains
    kpx, kdx = await robot.motor.get_pd_gains()

    # compute task space setpoint
    f,J,H = robot.forward_kinematics(-math.pi/4)
    y0 = f[0]

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
    y_gains = np.matrix(controller_config['y_gains'], dtype = float)
    y_torques = np.array(controller_config['y_torques'], dtype = float)
    y_offsets = np.array(controller_config['y_offsets'], dtype = float)
    y_guards = np.matrix(controller_config['y_guards'], dtype = float)

    hop_controller = HopController(
        robot.kinematics,
        y_gains, 
        y_torques,
        y_offsets,
        y_guards,
        kpx,        # motor kp (Nm/rad)
        kdx,        # motor kd (Nm/rad) 
        t_window,   # time measurements (for interpolation)
        x_window    # angle measurements (rad, for interpolation)
    )

    # stand up with femur at zero-degrees for 10 seconds at start
    hop_controller.mode = HopController._PULL
    while (time.perf_counter()-t0/1e3) < 5.:
        motor_state = await robot.set_position_rev(0., query=True)
        t = time.perf_counter()*1e3-t0
        if motor_state is not None:
            x, xdot= robot.convert_motor_posvel(motor_state)
            position_rev, kp_scale, kd_scale, torque = hop_controller.update(x, t)

    # record energy
    power_state = None
    while power_state is None:
        power_state = await robot.pdb.get_power_state()
    energy = power_state.energy * 3600

    # arrays for holding data
    hops = []
    this_hop_data = TrajectoryData()

    # start the hopping
    last_mode = hop_controller.mode
    min_pos_rev = robot.min_pos_rev - robot.calib_pos_rev + .001
    max_pos_rev = robot.max_pos_rev - robot.calib_pos_rev - .001
    while (time.perf_counter()-t0/1e3) < experiment_config['duration']:
        try:
            position_rev = min(max_pos_rev, max(min_pos_rev, position_rev))
            motor_state = await robot.set_position_rev(
                position_rev,
                kp_scale = kp_scale,
                kd_scale = kd_scale,
                feedforward_torque = torque,
                query=True
            )
            power_state = await robot.pdb.get_power_state()
        except BaseException as e:
            print('failed to send set_position command, check the motor limits')
            print(str(e))
            break

        if power_state is not None:
            energy = power_state.energy * 3600

        if motor_state is not None:
            t = time.perf_counter()*1e3-t0
            x, xdot = robot.convert_motor_posvel(motor_state)
            f,J,H = forward_kinematics(robot.kinematics, x)
            mode = hop_controller.mode
            u_y = y_gains[mode,0]*y_offsets[mode] + y_torques[mode]
            try:
                position_rev, kp_scale, kd_scale, torque = hop_controller.update(x, t)
                this_mode = hop_controller.mode

                # partition data for recording a new epoch
                if (this_mode == HopController._PULL) and (last_mode != HopController._PULL):
                    hops.append(this_hop_data)
                    this_hop_data = TrajectoryData()
                last_mode = this_mode

                # add data to the current hop
                this_hop_data.append(
                    x * _RAD_TO_DEG,
                    motor_state.position * _REV_TO_DEG,           
                    f[0],
                    f[1],
                    J[0],
                    H[0],
                    hop_controller.mode,
                    position_rev * _REV_TO_DEG,
                    kp_scale * kpx,          
                    kd_scale * kdx,          
                    torque,
                    u_y,           
                    energy,
                    t
                )
            except BaseException as e:
                print("failed to udpate controller, singular interpolation matrix likely")
                print(str(e))
    await robot.motor.controller.set_stop()
    
    # save data to file
    prefix = os.path.join(root_folder, experiment_config['data_folder'])
    now = time.time()
    datestring = str(datetime.date.fromtimestamp(now))
    experiment_folder = os.path.join(prefix, datestring+f'_{int(now)}')
    os.makedirs(experiment_folder)
    for i, hop in enumerate(hops):
        path = os.path.join(experiment_folder, f'hop_{i}.csv')
        hop.to_dataframe().to_csv(path)
    
    # save experiment config for reproduction
    with open(os.path.join(experiment_folder, 'experiment_config.yaml'), 'w') as f:
        yaml.dump(experiment_config, f)
    
    with open(os.path.join(experiment_folder, 'hardware_config.yaml'), 'w') as f:
        yaml.dump(robot.hardware_config, f)

import sys
if __name__ == "__main__":
    experiment_config = sys.argv[1]
    asyncio.run(main(experiment_config))
    