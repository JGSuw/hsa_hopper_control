import moteus
import asyncio
import math

from ..constants import _RAD_TO_DEG, _REV_TO_DEG, _REV_TO_RAD

class MotorState:
    def __init__(self, state):
        self.position = state.values[moteus.Register.POSITION]
        self.velocity= state.values[moteus.Register.VELOCITY]
        self.torque= state.values[moteus.Register.TORQUE]
        self.mode = state.values[moteus.Register.MODE]
        self.fault = state.values[moteus.Register.FAULT]

class Motor():
    def __init__(self, min_pos_rev, max_pos_rev, id = 1, transport = None):
        self.controller = moteus.Controller(id=id, transport = transport)
        self.stream = moteus.Stream(self.controller)
        self.min_pos_rev = min_pos_rev
        self.max_pos_rev = max_pos_rev

    def make_query_state(self):
        to_query_fields = {
            moteus.Register.POSITION : moteus.F32,
            moteus.Register.VELOCITY : moteus.F32,
            moteus.Register.TORQUE : moteus.F32,
            moteus.Register.MODE : moteus.INT8,
            moteus.Register.FAULT : moteus.INT8,
        }
        return self.controller.make_custom_query(to_query_fields)

    async def get_state(self):
        state = await self.controller.execute(self.make_query_state())
        return MotorState(state)
    
    async def set_position(self, *args , **kwargs):
        if 'position' in kwargs.keys():
            position = kwargs.get('position')
            if position < self.min_pos_rev:
                raise ValueError(f'Requested position {position} is less than minimum position {self.min_pos_rev}')
            if kwargs['position'] > self.max_pos_rev:
                raise ValueError(f'Requested position {position} is greater than maximum position {self.min_pos_rev}')
        state = await self.controller.set_position(*args, **kwargs)
        if state is not None:
            return MotorState(state)

    def make_set_torque(self, torque, query=False):
        if query:
            qr = moteus.QueryResolution()
        else:
            qr = None
        return self.controller.make_position(position = math.nan, kp_scale = 0., kd_scale = 0., feedforward_torque=torque, query_override=qr)

    async def set_torque(self, torque, query=False):
        command = self.make_set_torque(torque, query = query)
        state = await self.controller.execute(command)
        if query:
            return MotorState(state)
        
    async def query_moteus_state(self):
        return await self.controller.set_position(position=math.nan, query=True)
    
    async def get_pd_gains(self):
        kp = float( (await self.stream.command(
            b'conf get servo.pid_position.kp',
            allow_any_response=True
        )).decode('utf8'))
        kd = float( (await self.stream.command(
            b'conf get servo.pid_position.kd',
            allow_any_response=True
        )).decode('utf8'))
        return (kp/_REV_TO_RAD, kd/_REV_TO_RAD)
