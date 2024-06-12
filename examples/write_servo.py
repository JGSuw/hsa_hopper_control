if __name__ == "__main__":
    import sys
    import hsa_hopper.hardware
    device = sys.argv[1]
    servo = hsa_hopper.hardware.Servo(device, 115200, .25)
    servo.write_setpoint(int(sys.argv[2]))
