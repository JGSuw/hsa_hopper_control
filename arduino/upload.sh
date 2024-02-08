#!/usr/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DEVICE=adafruit:avr:feather32u4
arduino-cli compile -b $DEVICE -p $1 --upload $SCRIPT_DIR/servo_driver/servo_driver.ino