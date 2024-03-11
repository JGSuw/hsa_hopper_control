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
    jiggle_amplitude = experiment_config['jiggle_amplitude']
    kp_scale = experiment_config['kp_scale']
    kd_scale = experiment_config['kd_scale']
    jiggle_torque = experiment_config['jiggle_torque']

    servo_pos = np.array(experiment_config['servo_pos'], dtype=int)
    angle = np.zeros(len(servo_pos), dtype=float)
    data = {'kpx' : kp_scale*kpx, 'kdx': kd_scale*kdx, 
            'motor_setpoint' : [],
            'motor_angle' : [],
            'hsa_angle' : [], 
            'hsa_len' : [],
            'ff_torque' : [],
            'times' : []}
    for i,p in enumerate(servo_pos):
        ff_torque = []
        motor_angle= []
        hsa_len = []
        times = []
        robot.servo.write_setpoint(int(p))
        input('manually jiggle the foot to estimate rest length, then press enter to continue')
        motor_state = None
        while motor_state is None:
            motor_state = await robot.set_position_rev(0., kp_scale = 0., kd_scale = 0., query=True)
            x0_rad, xdot_rad = robot.convert_motor_posvel(motor_state)
        await robot.motor.controller.set_stop()
        t = t0 = time.perf_counter()
        jiggle_lengths = []
        while (t-t0) < T:
            motor_setpoint = x0_rad
            motor_torque = -jiggle_torque*np.sum([np.sin((2*np.pi*f)*(t-t0)) for f in freqs])
            motor_state = await robot.set_position_rad(
                motor_setpoint, 
                kp_scale = kp_scale, 
                kd_scale = kd_scale, 
                feedforward_torque = motor_torque,
                query=True)
            t = time.perf_counter()

            if motor_state is not None:
                x_rad, xdot_rad = robot.convert_motor_posvel(motor_state)
                f = forward_kinematics(robot.kinematics, x_rad)
                motor_angle.append(x_rad)
                hsa_len.append(f[1])
                ff_torque.append(motor_torque)
                times.append(t)

        # await robot.set_position_rad(x_rad, feedforward_torque=0., kp_scale = 0., kd_scale = 0.)
        await robot.motor.controller.set_stop()
        data['motor_setpoint'].append(motor_setpoint)
        data['motor_angle'].append(motor_angle)
        data['hsa_len'].append(hsa_len)
        data['ff_torque'].append(ff_torque)
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