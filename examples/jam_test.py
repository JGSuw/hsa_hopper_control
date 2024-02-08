import asyncio
import yaml
import hsa_hopper
import time
import sys
import os
import math

async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    hardware_config_path = os.path.join(root_folder, "resources/hardware.yaml")
    with open(hardware_config_path, 'r') as config_file:
        hardware_config = yaml.load(config_file, yaml.Loader)
    test_config_path = os.path.join(root_folder, "resources/jam_test.yaml")
    with open(test_config_path, 'r') as config_file:
        test_config = yaml.load(config_file, yaml.Loader)
    robot = hsa_hopper.Robot(hardware_config)
    await robot.motor.controller.set_stop()

    # move servo to neutral position and hold
    robot.servo.write_setpoint(1500)
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 1.:
        pass

    # move leg to standing position and hold
    t0 = time.perf_counter_ns()
    E0 = (await robot.pdb.get_power_state()).energy
    while (time.perf_counter_ns() - t0) / 1e9 < 3.:
        await robot.motor.set_position(position=test_config['motor_pos'], kp_scale = 2.5)
        E1 = (await robot.pdb.get_power_state()).energy
    t1 = time.perf_counter_ns()

    # report average power
    # formula units are (Watts * hours) * (3600 seconds / hour) / (seconds) 
    average_power = (E1 - E0) * 3600e9 / (t1 - t0)
    print(f'average power : {average_power}')

    # move servo to servo_pos and hold
    robot.servo.write_setpoint(1100)

    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 1.:
        await robot.motor.set_position(position=test_config['motor_pos'], kp_scale = 2.5)
        pass

    # measure initial power state
    await robot.motor.controller.set_stop()
    E0 = (await robot.pdb.get_power_state()).energy
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 10.:
        # result = await robot.motor.query_moteus_state() # QDD motor is passive
        await robot.motor.set_position(position=test_config['motor_pos'], kp_scale = 0., kd_scale = 0.)
        E1 = (await robot.pdb.get_power_state()).energy
        pass
    t1 = time.perf_counter_ns()

    # report average power
    # formula units are (Watts * hours) * (3600 seconds / hour) / (seconds) 
    average_power = (E1 - E0) * 3600e9 / (t1 - t0)
    print(f'average power : {average_power}')

    # record motor position
    motor_state = await robot.motor.get_state()
    motor_pos = motor_state.position
    print(f'resting motor position: {motor_pos}')

    # prompt user to disconnect servo
    robot.servo.write_setpoint(1500)
    input("Disable power to the HSA servo and enter anything to continue\n")

    # command QDD motor to motor_pos
    await robot.motor.controller.set_stop()
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns()-t0) / 1e9 < 1.:
        # await robot.motor.set_position(position=test_config['motor_pos'], kp_scale = 2.5)
        await robot.motor.set_position(position=motor_pos, kp_scale = 2.5)

    # measure power
    E0 = (await robot.pdb.get_power_state()).energy
    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns()-t0) / 1e9 < 10.:
        await robot.motor.set_position(position=motor_pos, kp_scale = 2.5)
        await robot.motor.set_position(position=motor_pos, kp_scale = 2.5)
        E1 = (await robot.pdb.get_power_state()).energy
    t1 = time.perf_counter_ns()

    # report average power
    average_power = (E1 - E0) * 3600e9 / (t1 - t0)
    print(f'average power : {average_power}')

    await robot.motor.controller.set_stop()

if __name__ == "__main__":
    asyncio.run(main())