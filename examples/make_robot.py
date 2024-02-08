from hsa_hopper import Robot
import yaml
import os
import asyncio

async def main():
    this_folder = os.path.dirname(os.path.abspath(__file__))
    root_folder = os.path.dirname(this_folder) 
    config_path = os.path.join(root_folder, "resources/hardware.yaml")
    robot = await Robot(config_path)
    print(robot)

# demonstrates how to load a hardware config file and construct a robot object
if __name__ == "__main__":
    asyncio.run(main())