
from hsa_hopper import *
from hsa_hopper.constants import _REV_TO_DEG, _RAD_TO_DEG
import yaml
import numpy as np
import asyncio
import os
import time
import datetime
import pickle

async def main(experiment_config):

    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    hardware_config_path = os.path.join(root_folder, experiment_config['hardware'])
    robot = Robot(hardware_config_path)

    servo_pulse = experiment_config['servo_pulse']
    payload_mass = np.array(experiment_config['payload_mass'])
    duration = experiment_config['duration']
    data = {}
    input('Remove the hopper from the rail and place it facing up on a flat surface,\n and disconnect power from the servo.')
    print('Beginning record of queiscent power consumption...\n Stand by and do not touch the robot.')
    power_state = None
    while power_state is None:
        power_state = await robot.pdb.get_power_state()
        if power_state is not None:
            t0 = time.time()
    _WATT_HOURS_TO_JOULES = 3600
    E0 = _WATT_HOURS_TO_JOULES*power_state.energy
    t = t0
    while t - t0 < duration:
        power_state = await robot.pdb.get_power_state()
        if power_state is not None:
            t = time.time()
            E1 = _WATT_HOURS_TO_JOULES*power_state.energy
    quiescent_power = (E1-E0)/(t-t0)
    data['quiescent_power'] = quiescent_power
    data['hsa_angle'] = [robot.servo.pulse_to_angle(p) for p in servo_pulse]
    data['payload_mass'] = payload_mass
    data['power_consumption'] = []
    data['rest_length'] = []

    for i, p in enumerate(servo_pulse):
        input('Remove the hopper from the rail and place it facing up on a flat surface,\n and connect power to the servo.')
        robot.servo.write_setpoint(p)
        t0 = time.time()
        while time.time() - t0 < 3.:
            pass # wait for servo to reach its setpoint
        power_consumption = []
        rest_length = []
        for j, m in enumerate(payload_mass):
            input(f'Place the robot on the rail with {1000*m} grams of added mass.')
            power_state = None
            while power_state is None:
                power_state = await robot.pdb.get_power_state()
            t0 = time.time()
            E0 = _WATT_HOURS_TO_JOULES*power_state.energy
            l = []
            while time.time() - t0 < duration:
                motor_state = await robot.motor.get_state()
                if motor_state is not None:
                    x, xdot = robot.convert_motor_posvel(motor_state)
                    f = robot.forward_kinematics(x)
                    l.append(f[1])
            power_state = await robot.pdb.get_power_state()
            while power_state is None:
                power_state = await robot.pdb.get_power_state()
            t = time.time()
            E1 = _WATT_HOURS_TO_JOULES*power_state.energy
            power = (E1-E0)/(t-t0)
            power_consumption.append(power)
            rest_length.append(np.average(l))
        data['power_consumption'].append(power_consumption)
        data['rest_length'].append(rest_length)

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