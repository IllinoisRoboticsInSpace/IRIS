#include <unity.h>

// Include here the bootstraps for testing packages
#include "example_package/example_package_bootstrap.hpp"
#include "example_proto/example_proto_bootstrap.hpp"
#include "irismotordriver/irismotordriver_bootstrap.hpp"

// Must be included last so no macro and function name definition conflict
// happens between dependencies. This fix needs to be applied to all dependencies.
// To learn more read this: https://community.platformio.org/t/nrf52832-include-set-breaks-compilation-on-platformio-but-compiles-fine-on-arduinoide/12237/2
#include <Arduino.h>

void RUN_UNITY_TESTS() {
    UNITY_BEGIN();
    RUN_EXAMPLE_PACKAGE_TESTS();
    RUN_EXAMPLE_PROTO_TESTS();
    RUN_IRISMOTORDRIVER_TESTS();
    UNITY_END();
}

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