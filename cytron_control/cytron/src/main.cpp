#include <Arduino.h>

int dumper_pins[4] = {46, 48, 50, 52};
int scooper_pins[4] = {47, 49, 51, 53};
int bab_limit_switch_pin = A2; // A2 is for bab bottom limit, A1 is for BAB fully extended limit
// int bab_forward_switch_pin = A4;

bool scooper_pwm = false;
bool scooper_direction = false;
bool dumper_pwm = false;
bool dumper_direction = false;

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(scooper_pins[i], OUTPUT);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(dumper_pins[i], OUTPUT);
    }

    pinMode(bab_limit_switch_pin, INPUT);

    Serial.begin(9600);
}

void setPin(int pins[4], bool pwm, bool direction) {
    analogWrite(pins[1], pwm ? 255 : 0);
    analogWrite(pins[3], pwm ? 255 : 0);
    analogWrite(pins[0], direction ? 255 : 0);
    analogWrite(pins[2], direction ? 255 : 0);
}

void parseCommand(char command[3]) {
    bool is_scoop = command[0] == 'S';
    bool is_pwm = command[1] == 'P';
    bool set_to = command[2] == 'T';

    if (is_scoop && is_pwm) scooper_pwm = set_to;
    if (is_scoop && !is_pwm) scooper_direction = !set_to;
    if (!is_scoop && is_pwm) dumper_pwm = set_to;
    if (!is_scoop && !is_pwm) dumper_direction = !set_to;
}

void runActuators(bool bab_lower_limit) {
    bool dumper_going_up = !dumper_direction && dumper_pwm;

    if (dumper_going_up || !bab_lower_limit) {
        setPin(dumper_pins, dumper_pwm, dumper_direction);
    } else {
        setPin(dumper_pins, false, false);
    }

    setPin(scooper_pins, scooper_pwm, scooper_direction);
    
    // if (!is_scoop) {
    //     Serial.print("Dumper detected, set_to = ");
    //     Serial.print(set_to);
    //     Serial.print(", limit_switch_reads[0]: ");
    //     Serial.print(bab_lower_limit);
    //     Serial.print(", going_up: ");
    //     Serial.print(!is_pwm && set_to);
    //     Serial.println();

    //     if (going_up || !bab_lower_limit)
    //         setPin(dumper_pins, is_pwm, set_to);
    //     else
    //         setPin(dumper_pins, 1, 0);
        
    // } else {
    //     setPin(scooper_pins, is_pwm, set_to);
    // }
}

void loop() {
    Serial.println(analogRead(bab_limit_switch_pin));
    bool bab_lower_limit = analogRead(bab_limit_switch_pin) < 900;

    runActuators(bab_lower_limit);

    char val;
    int count = Serial.readBytes(&val, 1);
    if (count != 1 || val != 'M') return;

    char buf[3];
    Serial.readBytes(buf, 3);

    parseCommand(buf);
}