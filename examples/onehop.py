from hsa_hopper import Robot
import yaml
import os
import asyncio
import time
import moteus
import numpy as np
import math
import sys

async def main(position):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)
    result = await robot.motor.controller.set_stop()
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 1.:
        result = await robot.motor.controller.set_position(position=position, query=True)
        # print(result.torque)
        print(result)
    # result = await robot.motor.set_torque(0., query_state = False)
    result = await robot.motor.controller.set_stop()
    print(result)

# demonstrates how to load a hardware config file and construct a robot object
if __name__ == "__main__":
    position = float(sys.argv[1])
    asyncio.run(main(position))