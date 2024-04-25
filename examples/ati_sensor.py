from hsa_hopper.force_sensor import ForceSensorProcess
import os
import yaml
import time
import sys

def main(output_path):
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    with open(config_path, 'r') as f:
        hardware_config = yaml.load(f, yaml.Loader)
    child_process = ForceSensorProcess(hardware_config['ati_sensor'])
    child_process.start()
    count = 0
    time.sleep(.01)
    child_process.start_stream()
    time.sleep(5.)
    child_process.stop_stream()
    time.sleep(.01)
    child_process.write_data(output_path)
    child_process.stop_process()
    child_process.join()

if __name__ == "__main__":
    output_path = sys.argv[1]
    main(output_path)
