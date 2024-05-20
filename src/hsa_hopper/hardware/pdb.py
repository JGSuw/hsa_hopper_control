import enum
import moteus
import asyncio

class PDBRegister(enum.IntEnum):
    STATE = 0x000
    FAULT_CODE = 0x001
    SWITCH_STATUS = 0x002
    LOCK_TIME = 0x003
    BOOT_TIME = 0x004
    OUTPUT_VOLTAGE = 0x010
    OUTPUT_CURRENT = 0x011
    TEMPERATURE = 0x012
    ENERGY = 0x013

class PowerState:
    def __init__(self, state):
        # self.voltage = state.values[PDBRegister.OUTPUT_VOLTAGE]
        # self.current = state.values[PDBRegister.OUTPUT_CURRENT]
        self.energy = state.values[PDBRegister.ENERGY]

class PowerDistributionBoard(moteus.Controller):
    def __init__(self, id = 32, transport = None):
        moteus.Controller.__init__(self, id = id, transport = transport)
    
    async def query(self):
        to_query_fields = {
            # PDBRegister.OUTPUT_VOLTAGE: moteus.F32,
            # PDBRegister.OUTPUT_CURRENT: moteus.F32,
            PDBRegister.ENERGY : moteus.F32
        }
        return await moteus.Controller.custom_query(self, to_query_fields)

    async def get_power_state(self):
        state = await self.query()
        if state is not None:
            return PowerState(state)
        else:
            return None

    async def print_info(self):
        power_state = await self.get_power_state()
        print(f'Voltage: {power_state.voltage}')
        print(f'Current: {power_state.current}')
        print(f'Energy: {power_state.energy}')