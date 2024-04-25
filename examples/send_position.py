from hsa_hopper.hardware import Robot
from hsa_hopper.constants import _RAD_TO_DEG
import yaml
import os
import asyncio
import time
import moteus
import numpy as np
import math
import sys

async def main(position_deg, kp_deg, kd_deg):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)
    result = await robot.motor.controller.set_stop()
    kp_rad, kd_rad = await robot.motor.get_pd_gains() 
    kp_scale = (kp_deg * _RAD_TO_DEG)/(kp_rad)
    kd_scale = (kd_deg * _RAD_TO_DEG)/(kp_rad)
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 2.:
        result = await robot.set_position_deg(position_deg, 
                                              kp_scale=kp_scale, 
                                              kd_scale=kd_scale)
        if result is not None:
            print((result.position, result.torque))
        pass
    # result = await robot.motor.set_torque(0., query_state = True)
    result = await robot.motor.controller.set_stop()
    print(result)

# demonstrates how to load a hardware config file and construct a robot object
if __name__ == "__main__":
    position_deg = float(sys.argv[1])
    kp_deg = float(sys.argv[2])
    kd_deg = float(sys.argv[3])
    asyncio.run(main(position_deg, kp_deg, kd_deg))
