from time import perf_counter
from multiprocessing import Process, JoinableQueue, Event
import queue
import NetFT
import socket

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

class SensorTrigger():
	def __init__(self, N):
		"""
		Event to request N measurements from the sensor
		"""
		self.N = N

class SensorTare():
	def __init__(self, n = 10):
		self.n = n

class StopProcess():
	def __init__(self):
		pass

class ForceSensorProcess(Process):
	def __init__(self, config):
		self.config = config
		self.cmd_queue = JoinableQueue()
		self.rtd_queue = JoinableQueue(maxsize=2*config['rdt_output_rate'])
		self.cmd_complete = Event()
		super().__init__(target=self.target)

	def set_stop(self):
		self.cmd_queue.put(StopProcess())

	def trigger_sensor(self, N):
		""" Command child process to get N measurements from the sensor"""
		self.cmd_queue.put(SensorTrigger(N))

	def tare_sensor(self, n = 10):
		cmd = SensorTare(n=n)
		self.cmd_queue.put(cmd)
	
	def target(self):
		done = False
		self.sensor = NetFT.Sensor(self.config['ip'], self.config['socket_timeout'])
		while not done:
			if not self.cmd_queue.empty():
				self.cmd_complete.clear()
				cmd = self.cmd_queue.get()
				if type(cmd) == SensorTrigger:
					self.sensor.getMeasurements(cmd.N)
					for i in range(cmd.N):
						try:
							data = self.sensor.receive()
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
							self.rtd_queue.put(measurement, timeout=1.)
						finally:
							continue
					self.cmd_complete.set()
				elif type(cmd) == SensorTare:
					self.sensor.tare(n = cmd.n)
					self.cmd_complete.set()
				elif type(cmd) == StopProcess:
					done = True
				self.cmd_queue.task_done()

	def measurements(self):
		while True:
			try:
				measurement = self.rtd_queue.get_nowait()
				self.rtd_queue.task_done()
				yield measurement
			except queue.Empty:
				break # end this generator when a queue.Empty exception is thrown