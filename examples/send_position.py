from hsa_hopper.hardware import Robot
import yaml
import os
import asyncio
import time
import moteus
import numpy as np
import math
import sys

async def main(position_deg):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)
    result = await robot.motor.controller.set_stop()
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 2.:
        result = await robot.set_position_deg(position_deg, kp_scale=2, kd_scale=1)
        # result = await robot.motor.controller.set_position(position=position, query=True)
        if result is not None:
            print((result.position, result.torque))
        pass
    # result = await robot.motor.set_torque(0., query_state = True)
    result = await robot.motor.controller.set_stop()
    print(result)

# demonstrates how to load a hardware config file and construct a robot object
if __name__ == "__main__":
    position_deg = float(sys.argv[1])
    asyncio.run(main(position_deg))
