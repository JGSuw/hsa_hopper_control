import os
import sys
import yaml
import asyncio
import hsa_hopper
import time

class HopController:
    def __init__(self,
                xmin = -30.,    # minimum angle degrees
                xmax = 45.,     # maximum angle degrees
                x12 = 15.,      # mode transition angle (1,2), degrees
                x23 = 35.,      # mode transition angle (2,3), degrees
                x31 = -20.,     # mode transition angle (3,1), degrees
                k1p = 0.01,     # proportional gain scale in mode 1
                k1d = 0.04,     # derivative gain scale in mode 1
                ff1 = 0.,
                k2p = .5,      # proportional gain scale in mode 2
                k2d = 0.25,       # derivative gain scale in mode 2
                ff2 = 0.,
                k3p = 0.75,       # proportional gain scale in mode 3
                k3d = 0.,       # derivative gain scale in mode 3
                ff3 = -3.
                ):
        self.xmin=xmin
        self.xmax=xmax
        self.x12=x12
        self.x23=x23
        self.x31=x31
        self.k1p=k1p
        self.k1d=k1d
        self.ff1=ff1
        self.k2p=k2p
        self.k2d=k2d
        self.ff2=ff2
        self.k3p=k3p
        self.k3d=k3d
        self.ff3=ff3
        self.mode = None

    def update_mode(self, x):
        if self.mode == None:
            self.mode = 2
        if self.mode == 1 and x*360 > self.x12:
            self.mode = 2
        elif self.mode == 2 and x*360 > self.x23:
            self.mode = 3
        elif self.mode == 3 and x*360 < self.x31:
            self.mode = 1

    def output(self):
        if self.mode == None:
            return None
        elif self.mode == 1:
            return (self.k1p, self.k1d, self.x31 / 360, self.ff1)
        elif self.mode == 2:
            return (self.k2p, self.k2d, self.xmax / 360, self.ff2)
        elif self.mode == 3:
            return (self.k3p, self.k3d, self.xmin / 360, self.ff3)


async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = hsa_hopper.Robot(config_path)
    await robot.motor.controller.set_stop()

    # initialize controller
    hop_controller = HopController()
    motor_state = await robot.motor.get_state()
    # print(motor_state.position)
    hop_controller.update_mode(motor_state.position)
    # print(hop_controller.mode)
    # print(hop_controller.output())

    t0 = time.perf_counter_ns()
    while (time.perf_counter_ns() - t0) / 1e9 < 10.:
        kp_scale, kd_scale, setpoint, torque = hop_controller.output()
        position = setpoint + calib_position
        state = await robot.motor.set_position(position = position, kp_scale = kp_scale, kd_scale = kd_scale, query = True, feedforward_torque = torque)
        if state is not None:
            print(state.position)
            print((state.position - calib_position)*360)
            hop_controller.update_mode(state.position - calib_position)
            print(hop_controller.mode)
    await robot.motor.controller.set_stop()

if __name__ == "__main__":
    asyncio.run(main())