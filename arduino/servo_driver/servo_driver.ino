#include <Servo.h>

#define SERVO_PIN 9
#define BAUD 115200

const int min_pulse = 1000;
const int max_pulse = 2000;
int16_t setpoint = 1500;
char buf[3];

Servo servo;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        // wait
    };
    servo.attach(SERVO_PIN);
    servo.writeMicroseconds(setpoint);
}

void handle_input() {
    if (Serial.available()) {
        size_t count = Serial.readBytes(buf, 3);
        if (3 == count && '\0' == buf[2]) {
            int16_t pulse = *(int16_t*)(buf);
            if (min_pulse <= pulse <= max_pulse) { 
                setpoint = pulse;
        }
    }
}

void loop() {
    handle_input();
    servo.writeMicroseconds(setpoint);
    delay(15);
}