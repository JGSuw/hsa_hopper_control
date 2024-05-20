from hsa_hopper.hardware import Robot
from hsa_hopper.kinematics import forward_kinematics
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
        x_rad = robot.convert_motor_pos(motor_state)

    freqs = np.array(experiment_config['jiggle_frequencies'])
    T = experiment_config['jiggle_time']
    servo_pos = np.array(experiment_config['servo_pulse'])
    motor_min_deg = np.array(experiment_config['motor_min_deg'])
    motor_max_deg = np.array(experiment_config['motor_max_deg'])

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
        t = t0 = time.perf_counter()
        a,b = motor_min_deg[i], motor_max_deg[i]
        x0 = (b+a)/2
        amplitude = (b-a)/(2*len(freqs))
        y = lambda t: x0 - amplitude*sum(np.cos(2*np.pi*f*(t-t0)) for f in freqs)
        try:
            while (t-t0) < T:
                motor_setpoint = y(t)
                motor_state = await robot.set_position_deg(motor_setpoint, query=True, kd_scale = 1.)
                t = time.perf_counter()
                if motor_state is not None:
                    x_rad = robot.convert_motor_pos(motor_state)
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
        except BaseException:
            print('exception caught!')
            robot.motor.controller.set_stop()

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
