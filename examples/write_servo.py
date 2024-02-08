if __name__ == "__main__":
    import sys
    import hsa_hopper
    servo = hsa_hopper.Servo('/dev/ttyACM0', 115200, .25)
    servo.write_setpoint(int(sys.argv[1]))