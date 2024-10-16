
from hsa_hopper.hardware import Robot
from hsa_hopper.kinematics import *
from hsa_hopper.constants import _REV_TO_DEG, _RAD_TO_DEG
import yaml
import numpy as np
import asyncio
import os
import time
import datetime
from collections import deque

async def main(experiment_config):

    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    hardware_config_path = os.path.join(root_folder, experiment_config['hardware'])
    robot = Robot(hardware_config_path)

    servo_pos = experiment_config['servo_pos']
    duration = experiment_config['duration']
    torque = experiment_config['torque']
    fet_temp = experiment_config['fet_temp']

    input('Mount the robot on the rail stand and enable power to the HSA servo.')
    data = {}
    data['energy'] = []
    data['motor_angle'] = []
    data['q_current'] = []
    data['d_current'] = []
    data['motor_torque'] = []
    data['motor_temperature'] = []
    data['time'] = []

    try:
        robot.servo.write_setpoint(servo_pos)
        t = t0 = time.perf_counter()
        while t - t0 < 2.:
            time.sleep(.01)
            t = time.perf_counter()

        for i, ff_torque in enumerate(torque):
            print(f'begin test with torque = {ff_torque}')
            motor_angle = deque()
            q_current = deque()
            d_current = deque()
            motor_torque = deque()
            motor_temperature = deque()
            time_s = deque()
            # monitor motor temperature, wait for it to go below fet_temp
            motor_temp = np.inf
            while motor_temp > fet_temp:
                motor_state = await robot.set_position_deg(
                    0.,
                    kp_scale=0.,
                    kd_scale = 0.,
                    feedforward_torque = 0.,
                    query=True
                )
                if motor_state is not None:
                    motor_temp = motor_state.temperature
                    print(f'motor temperature = {motor_temp}')

            E00 = (await robot.pdb0.get_power_state()).energy * 3600
            E01 = (await robot.pdb1.get_power_state()).energy * 3600

            t = t0 = time.perf_counter()
            # send feedforward torque for {duration} seconds
            while t - t0 < duration:
                motor_state = await robot.set_position_deg(
                    0.,
                    kp_scale = 0.,
                    kd_scale = 0.,
                    feedforward_torque = ff_torque,
                    query=True
                )
                if motor_state is not None:
                    if robot.convert_motor_pos(motor_state) > 0.:
                        raise BaseException("Runtime error: motor angle crossed zero, HSA is probably destroyed")
                t = time.perf_counter()
                if motor_state is not None:
                    motor_angle.append(robot.convert_motor_pos(motor_state))
                    q_current.append(motor_state.q_current)
                    d_current.append(motor_state.d_current)
                    motor_torque.append(motor_state.torque)
                    motor_temperature.append(motor_state.temperature)
                    time_s.append(t)

            E10 = (await robot.pdb0.get_power_state()).energy * 3600
            E11 = (await robot.pdb1.get_power_state()).energy * 3600

            data['energy'].append([E10-E00, E11-E01])
            data['motor_angle'].append(list(motor_angle))
            data['q_current'].append(list(q_current))
            data['d_current'].append(list(d_current))
            data['motor_torque'].append(list(motor_torque))
            data['motor_temperature'].append(list(motor_temperature))
            data['time'].append(list(time_s))

    except BaseException as e:
        print("Exceptionc caught")
        print(e)

    await robot.motor.controller.set_stop()

    # save data to file
    prefix = os.path.join(root_folder, experiment_config['data_folder'])
    now = time.time()
    datestring = str(datetime.date.fromtimestamp(now))
    experiment_folder = os.path.join(prefix, datestring+f'_{int(now)}')
    os.makedirs(experiment_folder)

    path = os.path.join(experiment_folder, 'data.yaml')
    with open(path, 'w') as f:
        yaml.dump(data, f, yaml.Dumper)

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