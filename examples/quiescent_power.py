
import asyncio
import hsa_hopper
import os
import yaml
import sys
import time
import datetime

async def main(note):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    data_folder = os.path.join(root_folder, "data")
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = hsa_hopper.Robot(config_path) 
    duration = 10. # measure power for 10 seconds
    E0 = (await robot.pdb.get_power_state()).energy
    t0 = time.perf_counter_ns()
    await robot.motor.controller.set_stop()
    while (time.perf_counter_ns()-t0)/1e9 < duration:
        # moteus_state = await robot.motor.query_moteus_state() # querying this state to ensure the board is on
        E1 = (await robot.pdb.get_power_state()).energy
    t1 = time.perf_counter_ns()
    data = {
        'power' : (E1-E0)/(t1-t0)*3600e9,
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