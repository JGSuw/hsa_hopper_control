from hsa_hopper import Robot
import yaml
import os
import asyncio
import time
import moteus
import numpy as np
import math

async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)
    torque = -3.
    result = await robot.motor.controller.set_stop()
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < .075:
        # result = await robot.motor.set_torque(torque, query_state = True)
        result = await robot.motor.controller.set_position(position=math.nan, kp_scale = 0., kd_scale = 0., feedforward_torque=torque, query=True)
        # print(result.torque)
        print(result)
    # result = await robot.motor.set_torque(0., query_state = False)
    result = await robot.motor.controller.set_stop()
    result = await robot.motor.query_moteus_state()
    print(result)

# demonstrates how to load a hardware config file and construct a robot object
if __name__ == "__main__":
    asyncio.run(main())