#include <Arduino.h>

int scooper_pins[4] = {46, 48, 50, 52};
int dumper_pins[4] = {47, 49, 51, 53};
int bab_limit_switch_pin = A2; // A2 is for bab bottom limit, A1 is for BAB fully extended limit
// int bab_forward_switch_pin = A4;
int bab_limit_switch_read;

bool going_fwd = 0;

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(scooper_pins[i], OUTPUT);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(dumper_pins[i], OUTPUT);
    }

    pinMode(bab_limit_switch_pin, OUTPUT);

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

    if (command[1] == 'D' && command[2] == 'F') going_fwd = 1;
    else if (command[1] == 'D' && command[2] == 'T') going_fwd = 0;

    if (is_scoop) {
        Serial.print("Scooper detected, set_to = ");
        Serial.print(set_to);
        Serial.print(", limit_switch_reads[0]: ");
        Serial.print(bab_limit_switch_read);
        Serial.print(", going_fwd: ");
        Serial.print(going_fwd);
        Serial.println();

        // safe to move BAB if both switches are not in contact, bottom switch is in contact and moving up, or top switch in contact and retracting   
        if ((bab_limit_switch_read == 0) ||
            (bab_limit_switch_read > 0 && going_fwd)) 
            setPin(scooper_pins, is_pwm, set_to);
        else
            setPin(scooper_pins, 1, 0);
        
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

    bab_limit_switch_read = analogRead(bab_limit_switch_pin);

    Serial.println(bab_limit_switch_read);

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
