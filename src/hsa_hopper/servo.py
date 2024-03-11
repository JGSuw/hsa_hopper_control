import serial

_ZERO = b'\x00'
def encode(pulse):
    if type(pulse) != int:
        raise ValueError(f"Pulse to encode must be int, not {type(pulse)}")
    data = (pulse).to_bytes(length=2, byteorder='little', signed=False)
    return data + _ZERO

class Servo:
    _MIN_PULSE = 1200
    _MAX_PULSE = 1900
    _ZERO_SETPOINT = 1500
    _ANGLE_RANGE = 1800

    def __init__(self, port, baudrate, gear_ratio):
        self._port = port
        self._baudrate = baudrate
        self._gear_ratio = gear_ratio
        self._serial = serial.Serial(port, baudrate=self._baudrate, timeout=1)
        if not (self._serial.is_open):
            raise Exception(f"Failed to open serial port at {port}")
        self._setpoint = self._ZERO_SETPOINT
        self._angle = self.pulse_to_angle(self._ZERO_SETPOINT)
    
    def _send_pulse(self):
        data = encode(self._setpoint)
        if not (self._serial.is_open):
            raise Exception(f'Serial port at {self._port} is not opened.')
        self._serial.write(data)

    def open_serial(self):
        if not (self._serial.is_open):
            self._serial.open()

    def close_serial(self):
        if self._serial.is_open:
            self._serial.close()
    
    def get_setpoint(self):
        return self._setpoint

    def write_setpoint(self, setpoint):
        if (type(setpoint) != int):
            raise ValueError(f'setpoint must be int, not {type(setpoint)}')
        if (self._MIN_PULSE <= setpoint <= self._MAX_PULSE):
            self._setpoint = setpoint
            self._angle = self.pulse_to_angle(setpoint)
            self._send_pulse()
        else:
            raise ValueError(f'setpoint {setpoint} is out of bounds [{self._MIN_PULSE}, {self._MAX_PULSE}]')
        
    def pulse_to_angle(self, pulse):
        mid_pulse = 1500
        return self._ANGLE_RANGE / 2000 * self._gear_ratio * (pulse - mid_pulse)
    
    def angle_to_pulse(self, angle):
        mid_pulse = 1500
        return int((2000) / self._ANGLE_RANGE / self._gear_ratio * angle + mid_pulse)