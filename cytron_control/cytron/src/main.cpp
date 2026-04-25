#include <Arduino.h>

int input_pins1[4] = {13, 12, 11, 10};
int scooper_pins[4] = {46, 48, 50, 52};
int input_pins2[4] = {5, 4, 3, 2};
int dumper_pins[4] = {47, 49, 51, 53};

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(input_pins1[i], INPUT);
        pinMode(scooper_pins[i], OUTPUT);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(input_pins2[i], INPUT);
        pinMode(dumper_pins[i], OUTPUT);
    }

    // for (int i = 0; i < 4;; i++) {
    //     analogWrite(scooper_pins[i], 0);
    //     analogWrite(dumper_pins[i], 0);
    // }
}

// void loop() {
//     for (int i = 0; i < 4; i++) {
//         if (digitalRead(input_pins1[i]) == HIGH) analogWrite(scooper_pins[i], 255);
//         else analogWrite(scooper_pins[i], 0);
//         if (digitalRead(input_pins2[i]) == HIGH) analogWrite(dumper_pins[i], 255);
//         else analogWrite(dumper_pins[i], 0);
//     }
// }

void loop() {
    analogWrite(scooper_pins[0], 0);
    analogWrite(scooper_pins[1], 0);
    analogWrite(scooper_pins[2], 0);
    analogWrite(scooper_pins[3], 0);

    analogWrite(dumper_pins[0], 0);
    analogWrite(dumper_pins[1], 0);
    analogWrite(dumper_pins[2], 0);
    analogWrite(dumper_pins[3], 0);

    delay(15000);

    analogWrite(scooper_pins[0], 255);
    analogWrite(scooper_pins[1], 255);
    analogWrite(scooper_pins[2], 255);
    analogWrite(scooper_pins[3], 255);

    delay(8000);

    analogWrite(scooper_pins[0], 0);
    analogWrite(scooper_pins[1], 0);
    analogWrite(scooper_pins[2], 0);
    analogWrite(scooper_pins[3], 0);

    delay(2500);

    analogWrite(scooper_pins[0], 0);
    analogWrite(scooper_pins[1], 255);
    analogWrite(scooper_pins[2], 0);
    analogWrite(scooper_pins[3], 255);

    delay(8000);

    analogWrite(scooper_pins[0], 0);
    analogWrite(scooper_pins[1], 0);
    analogWrite(scooper_pins[2], 0);
    analogWrite(scooper_pins[3], 0);

    delay(10000);

    analogWrite(dumper_pins[0], 0);
    analogWrite(dumper_pins[1], 255);
    analogWrite(dumper_pins[2], 0);
    analogWrite(dumper_pins[3], 255);

    delay(8000);

    analogWrite(dumper_pins[0], 0);
    analogWrite(dumper_pins[1], 0);
    analogWrite(dumper_pins[2], 0);
    analogWrite(dumper_pins[3], 0);

    delay(3000);

    analogWrite(dumper_pins[0], 255);
    analogWrite(dumper_pins[1], 255);
    analogWrite(dumper_pins[2], 255);
    analogWrite(dumper_pins[3], 255);

    delay(8000);

    analogWrite(dumper_pins[0], 0);
    analogWrite(dumper_pins[1], 0);
    analogWrite(dumper_pins[2], 0);
    analogWrite(dumper_pins[3], 0);

    delay(30000);
}
