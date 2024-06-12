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
        self.q_current = deque()
        self.d_current = deque()
        self.torque = deque()
        self.t_s = deque()
        
    def append(self,
        x_deg,
        x_moteus_deg,           
        y,
        l,
        mode,
        u_ff,
        q_current,
        d_current,
        torque,
        t_s
    ):
        self.x_deg.append(x_deg)
        self.x_moteus_deg.append(x_moteus_deg)
        self.y.append(y)
        self.l.append(l) 
        self.mode.append(mode)
        self.u_ff.append(u_ff)
        self.q_current.append(q_current)
        self.d_current.append(d_current)
        self.torque.append(torque)
        self.t_s.append(t_s)

    def to_dataframe(self):
        df = pd.DataFrame(self.__dict__)
        return df

async def main(experiment_config):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    
    hardware_config_path = os.path.join(root_folder, experiment_config['hardware'])
    robot = Robot(hardware_config_path)

    # record initial energy from PDB
    ps0 = await robot.pdb0.get_power_state()
    ps1 = await robot.pdb1.get_power_state()
    initial_energy = (ps1.energy + ps0.energy) * 3600 # convert watt-hours to joules

    # measure energy consumed in 3-seconds to calculate quiescent power
    print('measuring quiescent power')
    t = t0_s = time.perf_counter()
    while (t - t0_s) < 5.:
        motor_state = await robot.set_position_rad(
            math.nan,
            kp_scale = 0,
            kd_scale = 0
        )
        ps0 = await robot.pdb0.get_power_state()
        ps1 = await robot.pdb1.get_power_state()
        t = time.perf_counter()
    quiescent_power = ((ps0.energy + ps1.energy)*3600 - initial_energy)/(t-t0_s)
    print(f'quiescent power: {quiescent_power}')
    
    # construct the force sensor process
    with open(hardware_config_path, 'r') as f:
        hardware_config = yaml.load(f, yaml.Loader)
        ati_sensor_config = hardware_config['ati_sensor']

    ati_sensor = ForceSensorProcess(ati_sensor_config)
    ati_sensor.start()
    
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
    _xtd = controller_config['x_td']
    _xlo = controller_config['x_lo']
    controller = HopController(_kp, _kd, _x0, _u, _xtd, _xlo)

    # hsa setpoint
    robot.servo.write_setpoint(int(controller_config['servo_pos']))

    # get preconfigured motor gains in radians
    kp_moteus, kd_moteus = await robot.motor.get_pd_gains()
    t0_s = time.perf_counter()

    # apply gains from mode 0 (startup) and initial setpoint
    print('Initializing...')
    while (time.perf_counter()-t0_s) < 2.:
        kp_scale = controller.kp[HopController._STARTUP] / kp_moteus
        kd_scale = controller.kd[HopController._STARTUP] / kd_moteus
        x0 = controller.x0_rad[HopController._STARTUP]
        motor_state = await robot.set_position_rad(
                x0,
                kp_scale = kp_scale,
                kd_scale = kd_scale, 
                query=True)

    # arrays for holding data from each hop
    hops = []
    this_hop_data = None

    # initialize controller
    print('Begin!')
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
        except BaseException as e:
            print('failed to send set_position command, check the motor limits')
            print(str(e))
            await robot.motor.controller.set_stop()
            break

        if motor_state is not None:
            # update time, compute forward kinematics
            t_s = time.perf_counter()
            x_rad = robot.convert_motor_pos(motor_state)
            f, J = forward_kinematics(robot.kinematics, x_rad, jacobian=True)

            # update the controller
            controller.update(x_rad, t_s)
            this_mode = controller.mode

            # partition data for recording a new epoch
            if (this_mode == HopController._STANCE) and (last_mode != HopController._STANCE):
                ps0 = await robot.pdb0.get_power_state()
                ps1 = await robot.pdb1.get_power_state()
                ati_sensor.start_stream()
                this_hop_data =  {
                        'traj' : TrajectoryData(),
                        'initial_energy' : (ps0.energy+ps1.energy)*3600
                        }
                hops.append(this_hop_data)
            last_mode = this_mode
            if this_hop_data is not None:
                this_hop_data['traj'].append(
                    x_rad * _RAD_TO_DEG,
                    motor_state.position * _REV_TO_DEG,           
                    f[0],
                    f[1],
                    controller.mode,
                    u_ff,
                    motor_state.q_current,
                    motor_state.d_current,
                    motor_state.torque,
                    t_s
                )

    await robot.motor.controller.set_stop()
    ati_sensor.stop_stream()

    # compute electrical energy consumed by the hops
    hop_energy = []
    for i in range(len(hops)-1):
        E0 = hops[i]['initial_energy']
        E1 = hops[i+1]['initial_energy']
        T = hops[i]['traj'].t_s[-1] - hops[i]['traj'].t_s[0]
        hop_energy.append((E1-E0)-quiescent_power*T)

    # save hop data
    prefix = os.path.join(root_folder, experiment_config['data_folder'])
    now = time.time()
    datestring = str(datetime.date.fromtimestamp(now))
    experiment_folder = os.path.join(prefix, datestring+f'_{int(now)}')
    os.makedirs(experiment_folder)
    for i in range(len(hops)-1):
        path = os.path.join(experiment_folder, f'hop_{i}.csv')
        hops[i]['traj'].to_dataframe().to_csv(path)

    path = os.path.join(experiment_folder, 'energy.csv')
    pd.DataFrame({'hop_energy' : hop_energy}).to_csv(path)

    # save force sensor data
    ati_sensor.write_data(os.path.join(experiment_folder, 'ati_measurements.csv'))
    
    # save experiment config for reproduction
    with open(os.path.join(experiment_folder, 'experiment_config.yaml'), 'w') as f:
        yaml.dump(experiment_config, f)
    
    with open(os.path.join(experiment_folder, 'hardware_config.yaml'), 'w') as f:
        yaml.dump(robot.hardware_config, f)

    # end ati_sensor child process
    ati_sensor.stop_process()
    ati_sensor.join(timeout=1)
    if ati_sensor.is_alive():
        ati_sensor.terminate()

import sys
if __name__ == "__main__":
    config_rel_path = sys.argv[1]
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 

    with open(os.path.join(root_folder, config_rel_path), 'r') as f:
        experiment_config = yaml.load(f,yaml.Loader)

    asyncio.run(main(experiment_config))
