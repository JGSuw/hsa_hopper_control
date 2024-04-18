from hsa_hopper.hardware import Robot
from hsa_hopper.kinematics import forward_kinematics
import yaml
import os
import asyncio
import time
import math
import yaml

async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)
    result = await robot.motor.controller.set_stop()
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 1.:
        result = await robot.motor.get_state()
        if result is not None:
            print(f'position: {result.position}, velocity: {result.velocity}, torque: {result.torque}')
            x, xdot = robot.convert_motor_posvel(result)
            f = forward_kinematics(robot.kinematics, x)
            print(f'body position: {f[0]}, HSA length: {f[1]}')

    # result = await robot.motor.set_torque(0., query_state = True)
    # print(result.position)
    result = await robot.motor.controller.set_stop()
    print(result)

# demonstrates how to load a hardware config file and construct a robot object
if __name__ == "__main__":
    asyncio.run(main())
