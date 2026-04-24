#include <Arduino.h>

// void setup() {
//     // LED_BUILTIN is pin 13 on the Due
//     pinMode(LED_BUILTIN, OUTPUT);
// }

// void loop() {
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(500);
//     digitalWrite(LED_BUILTIN, LOW);
//     delay(500);
// }

// void setup() {
//     pinMode(36, OUTPUT);
//     pinMode(38, OUTPUT);
// }

// void loop() {
//     digitalWrite(36, HIGH);
//     analogWrite(38, 255);
// }

void setup() {
    pinMode(10, INPUT);
    pinMode(38, OUTPUT);
}

void loop() {
    if (digitalRead(10) == HIGH) {
        analogWrite(38, 255);
    }
    else {
        analogWrite(38, 0);
    }
}


// int input_pins1[4] = {10, 11, 12, 13};
// int output_pins1[4] = {46, 48, 50, 52};
// int input_pins2[4] = {10, 11, 12, 13};
// int output_pins2[4] = {46, 48, 50, 52};

// void setup() {
//     for (int i = 0; i < 4; i++) {
//         pinMode(input_pins1[i], INPUT);
//         pinMode(output_pins1[i], OUTPUT);
//     }

//     pinMode(47, OUTPUT);
//     pinMode(49, OUTPUT);
//     pinMode(51, OUTPUT);
//     pinMode(53, OUTPUT);
// }

// void loop() {
//     for (int i = 0; i < 4; i++) {
//         if (digitalRead(input_pins1[i]) == HIGH) analogWrite(output_pins1[i], 255);
//     }
// }