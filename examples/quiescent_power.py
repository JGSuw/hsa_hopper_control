
import asyncio
import hsa_hopper.hardware
import os
import yaml
import sys
import time
import datetime
import math

async def main(note):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    data_folder = os.path.join(root_folder, "data")
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = hsa_hopper.hardware.Robot(config_path) 
    duration = 60. # measure power for 10 seconds
    E00 = (await robot.pdb0.get_power_state()).energy
    E01 = (await robot.pdb1.get_power_state()).energy
    t0 = time.perf_counter()
    await robot.motor.controller.set_stop()
    while (time.perf_counter()-t0) < duration:
        # moteus_state = await robot.motor.query_moteus_state() # querying this state to ensure the board is on
        await robot.set_position_deg(-45,kp_scale=0.,kd_scale=0.)
        E10 = (await robot.pdb0.get_power_state()).energy
        E11 = (await robot.pdb1.get_power_state()).energy
    t1 = time.perf_counter()
    data = {
        'power0' : (E10-E00)/(t1-t0)*3600,
        'power1' : (E11-E01)/(t1-t0)*3600,
        'note' : note
    }
    now = str(datetime.datetime.fromtimestamp(time.time()))
    now = '_'.join(now.split(' '))
    outfilename = now + '_quiescent_power_test.yaml'
    with open(os.path.join(data_folder, outfilename), 'w') as outfile:
        yaml.dump(data, outfile, yaml.Dumper)

if __name__ == "__main__":
    note = None
    if len(sys.argv) > 1:
        note = sys.argv[1]
    asyncio.run(main(note))
