import yaml
import asyncio
import hop_experiment
import numpy as np

import sys
import os
if __name__ == "__main__":
    config_rel_path = sys.argv[1]
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 

    with open(os.path.join(root_folder, config_rel_path), 'r') as f:
        experiment_config = yaml.load(f,yaml.Loader)

    for servo_pos in experiment_config['servo_pos']:

        controller_config = experiment_config['controller']

        controller_config['servo_pos'] = servo_pos
        
        print(f'Starting trial at servo_pos = {int(servo_pos)}\n')
        input('Enter anything to start trial\n')
        asyncio.run(hop_experiment.main(experiment_config))