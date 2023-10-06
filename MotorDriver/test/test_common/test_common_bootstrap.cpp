#include <unity.h>

// Include here the bootstraps for testing packages
#include "example_package/example_package_bootstrap.hpp"

void RUN_UNITY_TESTS() {
    UNITY_BEGIN();
    RUN_EXAMPLE_PACKAGE_TESTS();
    UNITY_END();
}

#ifdef ARDUINO

#include <Arduino.h>
void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // NOTE!!! Wait for >2 secs
    delay(2000);

    RUN_UNITY_TESTS();
}

void loop() {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}

#else

int main(int argc, char **argv) {
    RUN_UNITY_TESTS();
    return 0;
}

#endif