from time import perf_counter
from multiprocessing import Process, JoinableQueue, Event
import queue
from collections import deque
import NetFT
import socket
import os.path
import pandas as pd

# indices of measurements in data returned by ATI sensor
_Fx = 0
_Fy = 1
_Fz = 2
_Mx = 3
_My = 4
_Mz = 5

class Measurement():
    def __init__(self, Fx,Fy,Fz,Mx,My,Mz,t):
        self.Fx = Fx
        self.Fy = Fy
        self.Fz = Fz
        self.Mx = Mx
        self.My = My
        self.Mz = Mz
        self.t = t

class StartStream():
    def __init__(self):
        pass

class StopStream():
    def __init__(self):
        pass

class WriteData():
    def __init__(self, path):
        self.path = path

class StopProcess():
    def __init__(self):
        pass

class ForceSensorProcess(Process):
    def __init__(self, config):
        self.config = config
        self.cmd_queue = JoinableQueue()
        self.rdt_queue = deque()
        self.streaming = False
        super().__init__(target=self.target)

    def start_stream(self):
        self.cmd_queue.put(StartStream())

    def stop_stream(self):
        self.cmd_queue.put(StopStream())

    def write_data(self, path):
        self.cmd_queue.put(WriteData(path))

    def stop_process(self):
        self.cmd_queue.put(StopProcess())

    def target(self):
        done = False
        sensor = NetFT.Sensor(self.config['ip'], self.config['socket_timeout'])
        while True:
            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get()
                if type(cmd) == StartStream:
                    self.streaming = True
                    self.cmd_queue.task_done()
                elif type(cmd) == StopStream:
                    self.streaming = False
                    self.cmd_queue.task_done()
                elif type(cmd) == WriteData:
                    child_write_data(cmd.path, self.rdt_queue)
                    self.cmd_queue.task_done()
                elif type(cmd) == StopProcess:
                    self.cmd_queue.task_done()
                    break
            elif self.streaming:
                data = sensor.getMeasurements(1)
                data = sensor.receive()
                t = perf_counter()
                measurement = Measurement(
                    data[_Fx]/self.config['counts_per_force'],
                    data[_Fy]/self.config['counts_per_force'],
                    data[_Fz]/self.config['counts_per_force'],
                    data[_Mx]/self.config['counts_per_torque'],
                    data[_My]/self.config['counts_per_torque'],
                    data[_Mz]/self.config['counts_per_torque'],
                    t
                )
                self.rdt_queue.append(measurement)

def child_write_data(path, queue):
    data = {
            'Fx' : [],
            'Fy' : [],
            'Fz' : [],
            'Mx' : [],
            'My' : [],
            'Mz' : [],
            't' : []}
    for item in queue:
        data['Fx'].append(item.Fx)
        data['Fy'].append(item.Fy)
        data['Fz'].append(item.Fz)
        data['Mx'].append(item.Mx)
        data['My'].append(item.My)
        data['Mz'].append(item.Mz)
        data['t'].append(item.t)
    df = pd.DataFrame(data)
    df.to_csv(path)
