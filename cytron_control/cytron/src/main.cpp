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

// void setup() {
//     pinMode(10, INPUT);
//     pinMode(38, OUTPUT);
// }
//
// void loop() {
//     if (digitalRead(10) == HIGH) {
//         analogWrite(38, 255);
//     }
//     else {
//         analogWrite(38, 0);
//     }
// }


int input_pins1[4] = {13, 12, 11, 10};
int output_pins1[4] = {46, 48, 50, 52};
int input_pins2[4] = {5, 4, 3, 2};
int output_pins2[4] = {47, 49, 51, 53};

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(input_pins1[i], INPUT);
        pinMode(output_pins1[i], OUTPUT);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(input_pins2[i], INPUT);
        pinMode(output_pins2[i], OUTPUT);
    }

    Serial.begin(9600);
}

void loop() {
    for (int i = 0; i < 4; i++) {
        // if (digitalRead(input_pins1[i]) == HIGH) analogWrite(output_pins1[i], 255);
        // else analogWrite(output_pins1[i], 0);
        // if (digitalRead(input_pins2[i]) == HIGH) analogWrite(output_pins2[i], 255);
        // else analogWrite(output_pins2[i], 0);

        // analogWrite(output_pins1[i], 255);
        
        // analogWrite(output_pins2[i], 255);
    }

    Serial.print("In1 : ");
    for (int i = 0; i < 4; i++) {
        Serial.print(digitalRead(input_pins1[i]));
        Serial.print(" ");
    }
    Serial.println("");

    Serial.print("In2: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(digitalRead(input_pins2[i]));
        Serial.print(" ");
    }
    Serial.println("");


    // delay(2000);

    // analogWrite(output_pins1[0], 0);
    // analogWrite(output_pins1[1], 255);
    // analogWrite(output_pins1[2], 0);
    // analogWrite(output_pins1[3], 255);

    // analogWrite(output_pins2[0], 0);
    // analogWrite(output_pins2[1], 255);
    // analogWrite(output_pins2[2], 0);
    // analogWrite(output_pins2[3], 255);

    delay(200);
}
