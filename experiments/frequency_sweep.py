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

    for kpy in experiment_config['kpy']:

        controller_config = experiment_config['controller']
        # controller_config["oscillator_K"] = kpy

        for gains in controller_config['y_gains']:
            gains[0] = kpy
        
        print(f'Starting trial at kpy = {int(kpy)}\n')
        input('Enter anything to start trial\n')
        asyncio.run(hop_experiment.main(experiment_config))
        print(f'Completed trial with kpy={int(kpy)}\n')