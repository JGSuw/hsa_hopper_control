from hsa_hopper import *
import math
import yaml
import numpy as np
import asyncio
import os
import time

# a = 3*.0254
# b = .15
# c = .3
# d = .01
# params = KinematicParameters(a,b,c,d)
# t = [0., .01, .02]
# x = [.1,.1,.1]
# f,J,H = forward_kinematics(params, -math.pi/4)
# kyp = 200.
# kyd = 1.
# kxp = 400./(2*math.pi)
# kxd = 2./(2*math.pi)
# y0 = f[0]
# print(y0)
# pd_controller = PDController(params, kyp, kyd, kyp*y0, kxp, kxd, t, x)
# print(pd_controller.gain_conversion(pd_controller.interpolate()))

async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = Robot(config_path)

    # get motor gains
    kpx, kdx = await robot.motor.get_pd_gains()

    # compute task space setpoint
    f,J,H = robot.forward_kinematics(-math.pi/4)
    y0 = f[0]

    # task space gain parameters
    kpy = 200.
    kdy = 1.
    uy = y0*kpy

    # read 3 states from the motor to initialize pd controller
    t0 = time.perf_counter()*1e3
    t_window = []
    x_window = []
    for i in range(3):
        motor_state = await robot.motor.get_state()
        t = time.perf_counter()*1e3-t0
        pos, vel = robot.convert_motor_posvel(motor_state)
        t_window.append(t)
        x_window.append(pos)

    # construct pd controller
    pd_controller = PDController(kpy, kdy, uy, kpx, kdx, t_window, x_window)

    # test interpolation
    interp = pd_controller.interpolate()

    # test gain conversion
    f, df, d2f = forward_kinematics(robot.kinematics, x_window[-1])
    x_rev, kp_scale, kd_scale, torque = pd_controller.gain_conversion(interp, f[0], df[0], d2f[0])

    # try running this for a short time
    while (time.perf_counter()-t0/1e3) < 10.:
        motor_state = await robot.motor.set_position(
            position = x_rev,
            kp_scale = kp_scale,
            kd_scale = kd_scale,
            feedforward_torque = torque,
            query = True
        )
        if motor_state is not None:
            t = time.perf_counter()*1e3-t0
            pos, vel = robot.convert_motor_posvel(motor_state)
            pd_controller.update_window(pos, vel)
            try:
                interp = pd_controller.interpolate()
                f, df, d2f = forward_kinematics(robot.kinematics, interp)
                x_rev, kp_scale, kd_scale, torque = pd_controller.gain_conversion(interp, f[0], df[0], d2f[0])
            except:
                print("failed to convert gains, singular interpolation matrix likely")
    await robot.motor.controller.set_stop()

if __name__ == "__main__":
    asyncio.run(main())
    