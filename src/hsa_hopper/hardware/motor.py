import moteus
import asyncio
import math

from ..constants import _RAD_TO_DEG, _REV_TO_DEG, _REV_TO_RAD

_QUERY_RESOLUTION = moteus.QueryResolution()
_QUERY_RESOLUTION.position = moteus.F32
_QUERY_RESOLUTION.velocity = moteus.IGNORE
_QUERY_RESOLUTION.torque = moteus.F32
_QUERY_RESOLUTION.q_current = moteus.IGNORE
_QUERY_RESOLUTION.d_current = moteus.IGNORE
_QUERY_RESOLUTION.temperature = moteus.IGNORE
_QUERY_RESOLUTION.voltage = moteus.IGNORE
_QUERY_RESOLUTION._extra = {
    moteus.Register.MODE : moteus.INT8,
    moteus.Register.FAULT : moteus.INT8
}

class MotorState:
    def __init__(self, state):
        self.position = state.values[moteus.Register.POSITION]
        self.torque = state.values[moteus.Register.TORQUE]
        # self.q_current = state.values[moteus.Register.Q_CURRENT]
        # self.d_current = state.values[moteus.Register.D_CURRENT]
        # self.voltage = state.values[moteus.Register.VOLTAGE]
        self.mode = state.values[moteus.Register.MODE]
        self.fault = state.values[moteus.Register.FAULT]

class Motor():
    def __init__(self, min_pos_rev, max_pos_rev, id = 1, transport = None):
        self.controller = moteus.Controller(
            id=id, 
            transport = transport,
            query_resolution = _QUERY_RESOLUTION)
        self.stream = moteus.Stream(self.controller)
        self.min_pos_rev = min_pos_rev
        self.max_pos_rev = max_pos_rev

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
        
    async def get_state(self):
        state = await self.controller.set_position(position=math.nan, query=True)
        if state is not None:
            return MotorState(state)
        else:
            return None
    
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
