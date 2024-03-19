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
import pickle
from collections import deque


async def main(experiment_config):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    
    hardware_config_path = os.path.join(root_folder, experiment_config['hardware'])
    robot = Robot(hardware_config_path)

    kpx, kdx = await robot.motor.get_pd_gains()

    motor_state = None
    while motor_state is None:
        motor_state = await robot.set_position_rev(0., kp_scale = 0., kd_scale = 0., query=True)
        x_rad, xdot_rad = robot.convert_motor_posvel(motor_state)

    freqs = np.array(experiment_config['jiggle_frequencies'])
    T = experiment_config['jiggle_time']
    amplitude = experiment_config['jiggle_amplitude']
    servo_min = experiment_config['servo_min_pulse']
    servo_max = experiment_config['servo_max_pulse']
    motor_min_deg = experiment_config['motor_min_deg']
    motor_max_deg = experiment_config['motor_max_deg']
    N_setpoints = experiment_config['N_setpoints']
    servo_pos = np.linspace(servo_min, servo_max, N_setpoints)
    motor_pos = np.linspace(motor_min_deg, motor_max_deg, N_setpoints)

    data = {'motor_angle' : [],
            'motor_torque' : [],
            'hsa_angle' : [], 
            'hsa_rest_len' : [],
            'hsa_len' : [],
            'dldtheta': [],
            'times' : []}
    for i,p in enumerate(servo_pos):
        await robot.motor.controller.set_stop()
        robot.servo.write_setpoint(int(p))
        input('')
        motor_angle = []
        motor_torque = []
        hsa_len = []
        dl_dtheta = []
        times = []
        motor_state = None
        for j, x0 in enumerate(motor_pos):
            t = t0 = time.perf_counter()
            y = lambda t: x0 + amplitude*sum(np.sin(2*np.pi*f*(t-t0)) for f in freqs)
            while (t-t0) < T:
                motor_setpoint = y(t)
                motor_state = await robot.set_position_deg(motor_setpoint, query=True)
                t = time.perf_counter()
                if motor_state is not None:
                    x_rad, xdot_rad = robot.convert_motor_posvel(motor_state)
                    f, df = forward_kinematics(robot.kinematics, x_rad, jacobian=True)
                    motor_angle.append(x_rad)
                    motor_torque.append(motor_state.torque)
                    hsa_len.append(f[1])
                    dl_dtheta.append(df[1])
                    times.append(t)
        await robot.motor.controller.set_stop()
        data['motor_angle'].append(motor_angle)
        data['motor_torque'].append(motor_torque)
        data['hsa_len'].append(hsa_len)
        data['dldtheta'].append(dl_dtheta)
        data['times'].append(times)
        data['hsa_angle'].append(robot.servo.pulse_to_angle(p))
        print(np.average(hsa_len))

    # save data to file
    prefix = os.path.join(root_folder, experiment_config['data_folder'])
    now = time.time()
    datestring = str(datetime.date.fromtimestamp(now))
    experiment_folder = os.path.join(prefix, datestring+f'_{int(now)}')
    os.makedirs(experiment_folder)

    path = os.path.join(experiment_folder, 'data.pickle')
    with open(path, 'wb') as file:
        pickle.dump(data,file)

    # save experiment config for reproduction
    with open(os.path.join(experiment_folder, 'experiment_config.yaml'), 'w') as f:
        yaml.dump(experiment_config, f)
    
    with open(os.path.join(experiment_folder, 'hardware_config.yaml'), 'w') as f:
        yaml.dump(robot.hardware_config, f)

import sys
if __name__ == "__main__":
    config_rel_path = sys.argv[1]
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 

    with open(os.path.join(root_folder, config_rel_path), 'r') as f:
        experiment_config = yaml.load(f,yaml.Loader)

    asyncio.run(main(experiment_config))