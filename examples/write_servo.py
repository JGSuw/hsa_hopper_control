if __name__ == "__main__":
    import sys
    import hsa_hopper.hardware
    servo = hsa_hopper.hardware.Servo('/dev/ttyACM0', 115200, .25)
    servo.write_setpoint(int(sys.argv[1]))
