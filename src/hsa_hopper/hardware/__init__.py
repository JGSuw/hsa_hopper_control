from .motor import Motor
from .pdb import PowerDistributionBoard
from .servo import Servo
from ..kinematics import KinematicParameters, forward_kinematics
from hsa_hopper.constants import _REV_TO_RAD, _RAD_TO_DEG, _REV_TO_DEG

import moteus_pi3hat
import yaml
import math

def pi3hat_tranport_factory(hardware_config):
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            hardware_config["motor"]["bus"] : [hardware_config["motor"]["id"]],
            hardware_config["pdb"]["bus"] : [hardware_config["pdb"]["id"]]
        }
    )
    return transport

class Robot:
    def __init__(self, hardware_config_path):
        with open(hardware_config_path, 'r') as config_file:
            self.hardware_config = yaml.load(config_file, yaml.Loader)

        self.calib_pos_deg = self.hardware_config["motor"]['calib_pos_deg']
        self.calib_pos_rad = self.calib_pos_deg/_RAD_TO_DEG
        self.calib_pos_rev = self.calib_pos_deg/_REV_TO_DEG

        self.min_pos_deg = self.hardware_config["limits"]['motor_min_angle_deg'] + self.calib_pos_deg
        self.min_pos_rad = self.min_pos_deg/_RAD_TO_DEG 
        self.min_pos_rev = self.min_pos_deg/_REV_TO_DEG

        self.max_pos_deg = self.hardware_config["limits"]['motor_max_angle_deg'] + self.calib_pos_deg
        self.max_pos_rad = self.max_pos_deg/_RAD_TO_DEG 
        self.max_pos_rev = self.max_pos_deg/_REV_TO_DEG

        self.transport = pi3hat_tranport_factory(self.hardware_config)
        self.motor = Motor(self.min_pos_rev, self.max_pos_rev, self.hardware_config["motor"]["id"], self.transport)
        self.pdb = PowerDistributionBoard(self.hardware_config["pdb"]["id"], self.transport)
        self.servo = Servo(self.hardware_config["servo"]["port"],
                            self.hardware_config["servo"]["baudrate"],
                            self.hardware_config["servo"]["gear_ratio"])
        self.kinematics = KinematicParameters(
            self.hardware_config["kinematics"]["a"],
            self.hardware_config["kinematics"]["b"],
            self.hardware_config["kinematics"]["c"],
            self.hardware_config["kinematics"]["d"]
        )

    def convert_motor_pos(self, motor_state):
        pos = motor_state.position*_REV_TO_RAD - self.calib_pos_rad
        return pos
    
    async def set_position_deg(self, motor_pos_deg, **kwargs):
        return await self.motor.set_position(position = (motor_pos_deg+self.calib_pos_deg)/_REV_TO_DEG, **kwargs)

    async def set_position_rad(self, motor_pos_rad, **kwargs):
        return await self.motor.set_position(position = (motor_pos_rad+self.calib_pos_rad)/_REV_TO_RAD, **kwargs)

    async def set_position_rev(self, motor_pos_rev, **kwargs):
        return await self.motor.set_position(position = (motor_pos_rev+self.calib_pos_rev), **kwargs)

    def forward_kinematics(self, motor_pos_rad):
        return forward_kinematics(self.kinematics, motor_pos_rad)
