from hsa_hopper.hardware import *
from hsa_hopper.kinematics import KinematicParameters, forward_kinematics
from hsa_hopper.collocation import PiecewiseInterpolation
from hsa_hopper.controller import HopController
from hsa_hopper.constants import _REV_TO_DEG, _RAD_TO_DEG
from hsa_hopper.force_sensor import ForceSensorProcess
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
        self.x_deg = deque()
        self.x_moteus_deg = deque()
        self.y = deque()
        self.l = deque()
        self.mode = deque()
        self.u_ff = deque()
        self.energy = deque()
        self.t_s = deque()
        
    def append(self,
        x_deg,
        x_moteus_deg,           
        y,
        l,
        mode,
        u_ff,
        energy,
        t_s
    ):
        self.x_deg.append(x_deg)
        self.x_moteus_deg.append(x_moteus_deg)
        self.y.append(y)
        self.l.append(l) 
        self.mode.append(mode)
        self.u_ff.append(u_ff)
        self.energy.append(energy)
        self.t_s.append(t_s)

    def to_dataframe(self):
        df = pd.DataFrame(self.__dict__)
        return df

async def main(experiment_config):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    
    hardware_config_path = os.path.join(root_folder, experiment_config['hardware'])
    robot = Robot(hardware_config_path)

    with open(hardware_config_path, 'r') as f:
        hardware_config = yaml.load(f, yaml.Loader)
        ati_sensor_config = hardware_config['ati_sensor']

#   ati_sensor = ForceSensorProcess(ati_sensor_config)
#   ati_sensor.start()
    
    #### build the HopController ####
    controller_config = experiment_config['controller']

    # controller gains (in units radians)
    _kp = np.array(controller_config['kp'])
    _kd = np.array(controller_config['kd'])
    _x0 = np.array(controller_config['x0'])

    # feed-forward torque interpolations
    _u = [  # modes 0,2 have interpolations, mode 1 (flight) has none
            PiecewiseInterpolation(np.array(d['mat']),np.array(d['tk']))
            if d is not None else None
            for d in controller_config['u_interp']
    ]
    # 'touchdown' motor angle in radians, relative to calibration
    _xtd = np.array(controller_config['x_td'])
    controller = HopController(_kp, _kd, _x0, _u, _xtd)

    # hsa setpoint
    robot.servo.write_setpoint(int(controller_config['servo_pos']))

    # get preconfigured motor gains in radians
    kp_moteus, kd_moteus = await robot.motor.get_pd_gains()
    t0_s = time.perf_counter()
    # apply gains from mode 0 (startup) and initial setpoint
    while (time.perf_counter()-t0_s) < 5.:
        kp_scale = controller.kp[HopController._STARTUP] / kp_moteus
        kd_scale = controller.kd[HopController._STARTUP] / kd_moteus
        x0 = controller.x0_rad[HopController._STARTUP]
        motor_state = await robot.set_position_rad(
                x0,
                kp_scale = kp_scale,
                kd_scale = kd_scale, 
                query=True)

    # record energy
    power_state = None
    while power_state is None:
        power_state = await robot.pdb.get_power_state()
    energy = power_state.energy * 3600 # convert watt-hours to joules

    # arrays for holding data from each hop
    this_hop_data = TrajectoryData()
    hops = [this_hop_data]

    # allocate queue for ATI measurements
#   N_ati_measurements = int(experiment_config['duration']*ati_sensor.config['rdt_output_rate'])
#   ati_z_force = deque(maxlen=N_ati_measurements)
#   ati_t = deque(maxlen=N_ati_measurements)

    # trigger ATI sensor
#   ati_sensor.trigger_sensor(N_ati_measurements)

    # initialize controller
    t_s = t0_s = time.perf_counter()
    controller.initialize(t0_s)

    while (t_s-t0_s) < experiment_config['duration']:
        try:
            t_s = time.perf_counter()
            kp, kd, x0_rad, u_ff = controller.output(t_s)
            kp_scale = kp / kp_moteus
            kd_scale = kd / kd_moteus
            motor_state = await robot.set_position_rad(
                x0_rad,
                kp_scale = kp_scale,
                kd_scale = kd_scale,
                feedforward_torque = u_ff,
                query=True
            )
#           power_state = await robot.pdb.get_power_state()
        except BaseException as e:
            print('failed to send set_position command, check the motor limits')
            print(str(e))
            break

#       if power_state is not None:
#           energy = power_state.energy * 3600

        if motor_state is not None:
            # update time, compute forward kinematics
            t_s = time.perf_counter()
            x_rad, xdot_rad = robot.convert_motor_posvel(motor_state)
            f, J = forward_kinematics(robot.kinematics, x_rad, jacobian=True)

            # update the controller
            output = controller.update(x_rad, t_s)
            this_mode = controller.mode

            # partition data for recording a new epoch
            if (this_mode == HopController._STANCE) and (last_mode != HopController._STANCE):
                this_hop_data = TrajectoryData()
                hops.append(this_hop_data)
            last_mode = this_mode
            this_hop_data.append(
                x_rad * _RAD_TO_DEG,
                motor_state.position * _REV_TO_DEG,           
                f[0],
                f[1],
                controller.mode,
                u_ff,
                0,
                t_s
            )

            # read ATI measurements
#           for m in ati_sensor.measurements():
#               ati_z_force.append(m.Fz)
#               ati_t.append(m.t)

    await robot.motor.controller.set_stop()

    
    # save hop data
    prefix = os.path.join(root_folder, experiment_config['data_folder'])
    now = time.time()
    datestring = str(datetime.date.fromtimestamp(now))
    experiment_folder = os.path.join(prefix, datestring+f'_{int(now)}')
    os.makedirs(experiment_folder)
    for i, hop in enumerate(hops):
        path = os.path.join(experiment_folder, f'hop_{i}.csv')
        hop.to_dataframe().to_csv(path)

    # save force sensor data
#   ati_sensor_path = os.path.join(experiment_folder, 'ati_sensor.csv')
#   df = pd.DataFrame(
#       data = {
#           'z_force': ati_z_force,
#           't': ati_t
#       })
#   df.to_csv(ati_sensor_path)
    
    # save experiment config for reproduction
    with open(os.path.join(experiment_folder, 'experiment_config.yaml'), 'w') as f:
        yaml.dump(experiment_config, f)
    
    with open(os.path.join(experiment_folder, 'hardware_config.yaml'), 'w') as f:
        yaml.dump(robot.hardware_config, f)

    # flush ati_sensor data buffer
#   ati_sensor.set_stop()
#   time.sleep(1.)
#   for m in ati_sensor.measurements():
#       pass

    # tell child process to stop and then join
#   ati_sensor.join(timeout=1.)
#   if ati_sensor.is_alive():
#       ati_sensor.terminate()

import sys
if __name__ == "__main__":
    config_rel_path = sys.argv[1]
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 

    with open(os.path.join(root_folder, config_rel_path), 'r') as f:
        experiment_config = yaml.load(f,yaml.Loader)

    asyncio.run(main(experiment_config))
