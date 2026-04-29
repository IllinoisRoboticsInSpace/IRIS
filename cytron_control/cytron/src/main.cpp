#include <Arduino.h>

int scooper_pins[4] = {46, 48, 50, 52};
int dumper_pins[4] = {47, 49, 51, 53};

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(scooper_pins[i], OUTPUT);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(dumper_pins[i], OUTPUT);
    }

    Serial.begin(9600);
}

void setPin(int pins[4], bool is_pwm, bool set_to) {
    if (is_pwm) {
        analogWrite(pins[1], set_to ? 255 : 0);
        analogWrite(pins[3], set_to ? 255 : 0);
    } else {
        analogWrite(pins[0], set_to ? 255 : 0);
        analogWrite(pins[2], set_to ? 255 : 0);
    }
}

void runCommand(char command[3]) {
    bool is_scoop = command[0] == 'S';
    bool is_pwm = command[1] == 'P';
    bool set_to = command[2] == 'T';

    if (is_scoop) {
        setPin(scooper_pins, isprint, set_to);
    } else {
        setPin(dumper_pins, is_pwm, set_to);
    }
}

void loop() {
    char buf[3];

    char val;
    int count;
    while (true) {
        count = Serial.readBytes(&val, 1);
        if (count == 1 && val == 'M') break;
    }

    Serial.readBytes(buf, 3);

    runCommand(buf);
}

// void loop() {
//     analogWrite(scooper_pins[0], 0);
//     analogWrite(scooper_pins[1], 0);
//     analogWrite(scooper_pins[2], 0);
//     analogWrite(scooper_pins[3], 0);

//     analogWrite(dumper_pins[0], 0);
//     analogWrite(dumper_pins[1], 0);
//     analogWrite(dumper_pins[2], 0);
//     analogWrite(dumper_pins[3], 0);

//     delay(15000);

//     analogWrite(scooper_pins[0], 255);
//     analogWrite(scooper_pins[1], 255);
//     analogWrite(scooper_pins[2], 255);
//     analogWrite(scooper_pins[3], 255);

//     delay(8000);

//     analogWrite(scooper_pins[0], 0);
//     analogWrite(scooper_pins[1], 0);
//     analogWrite(scooper_pins[2], 0);
//     analogWrite(scooper_pins[3], 0);

//     delay(2500);

//     analogWrite(scooper_pins[0], 0);
//     analogWrite(scooper_pins[1], 255);
//     analogWrite(scooper_pins[2], 0);
//     analogWrite(scooper_pins[3], 255);

//     delay(8000);

//     analogWrite(scooper_pins[0], 0);
//     analogWrite(scooper_pins[1], 0);
//     analogWrite(scooper_pins[2], 0);
//     analogWrite(scooper_pins[3], 0);

//     delay(10000);

//     analogWrite(dumper_pins[0], 0);
//     analogWrite(dumper_pins[1], 255);
//     analogWrite(dumper_pins[2], 0);
//     analogWrite(dumper_pins[3], 255);

//     delay(8000);

//     analogWrite(dumper_pins[0], 0);
//     analogWrite(dumper_pins[1], 0);
//     analogWrite(dumper_pins[2], 0);
//     analogWrite(dumper_pins[3], 0);

//     delay(3000);

//     analogWrite(dumper_pins[0], 255);
//     analogWrite(dumper_pins[1], 255);
//     analogWrite(dumper_pins[2], 255);
//     analogWrite(dumper_pins[3], 255);

//     delay(8000);

//     analogWrite(dumper_pins[0], 0);
//     analogWrite(dumper_pins[1], 0);
//     analogWrite(dumper_pins[2], 0);
//     analogWrite(dumper_pins[3], 0);

//     delay(30000);
// }
