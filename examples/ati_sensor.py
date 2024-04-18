from hsa_hopper.force_sensor import ForceSensorProcess
import os
import yaml
import time

def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    with open(config_path, 'r') as f:
        hardware_config = yaml.load(f, yaml.Loader)
    child_process = ForceSensorProcess(hardware_config['ati_sensor'])
    child_process.start()
    child_process.tare_sensor()
    N_measurements = int(4*hardware_config['ati_sensor']['rdt_output_rate'])
    child_process.trigger_sensor(N_measurements)
    count = 0
    time.sleep(.01)
    while not child_process.cmd_complete.is_set():
        time.sleep(.01)
        # get the measurements
        for m in child_process.measurements():
            print(
                {
                    'Fx' : m.Fx,
                    'Fy' : m.Fy,
                    'Fz' : m.Fz,
                    't' : m.t
                }
            )
            count += 1

    # check for measurements one more time to flush the rtd_queue
    for m in child_process.measurements():
        print(
            {
                'Fx' : m.Fx,
                'Fy' : m.Fy,
                'Fz' : m.Fy,
                't' : m.t
            }
        )
        count += 1

    print(f'Total number of measurements received: {count}, {N_measurements-count} were dropped.')
    child_process.set_stop()
    time.sleep(1.)
    child_process.join()

if __name__ == "__main__":
    main()
