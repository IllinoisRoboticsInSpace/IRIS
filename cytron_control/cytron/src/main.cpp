#include <Arduino.h>

const int dumper_pins[4] = {8, 9, 7, 6};
const int scooper_pins[4] = {4, 5, 2, 3};

const int bab_limit_switch_pin = A0;
const int LIMIT_THRESHOLD = 900;

bool scooper_pwm = false;
bool scooper_direction = false;
bool dumper_pwm = false;
bool dumper_direction = false;

constexpr uint32_t SLOW_BLINK_MS = 600;
constexpr uint32_t FAST_BLINK_MS = 150;
constexpr uint32_t LIMIT_BLINK_MS = 65;
constexpr uint32_t MESSAGE_FLASH_MS = 100;

constexpr uint32_t ACTUATOR_DISPLAY_MS = 1000;

void setLED(bool red, bool green, bool blue) {
    digitalWrite(LEDR, red ? LOW : HIGH);
    digitalWrite(LEDG, green ? LOW : HIGH);
    digitalWrite(LEDB, blue ? LOW : HIGH);
}

char commandBuffer[3];
uint8_t commandIndex = 0;
bool receivingCommand = false;

uint32_t lastSerialByteMs = 0;
uint32_t lastMessageMs = 0;
bool messageFlashPending = false;

constexpr uint32_t SERIAL_TIMEOUT_MS = 100;

void setPin(const int pins[4], bool pwm, bool direction) {
    // Original code only writes 0 or 255.
    // digitalWrite is equivalent for these values.
    digitalWrite(pins[1], pwm ? HIGH : LOW);
    digitalWrite(pins[3], pwm ? HIGH : LOW);
    digitalWrite(pins[0], direction ? HIGH : LOW);
    digitalWrite(pins[2], direction ? HIGH : LOW);
}

void parseCommand(const char command[3]) {
    bool is_scoop = command[0] == 'S';
    bool is_pwm = command[1] == 'P';
    bool set_to = command[2] == 'T';

    if (is_scoop && is_pwm)
        scooper_pwm = set_to;

    if (is_scoop && !is_pwm)
        scooper_direction = !set_to;

    if (!is_scoop && is_pwm)
        dumper_pwm = set_to;

    if (!is_scoop && !is_pwm)
        dumper_direction = !set_to;
}

void handleSerial(uint32_t now) {
    // Recover from an incomplete packet.
    if (receivingCommand &&
        (uint32_t)(now - lastSerialByteMs) >= SERIAL_TIMEOUT_MS) {
        receivingCommand = false;
        commandIndex = 0;
    }

    // Only consume bytes that are already available.
    while (Serial.available() > 0) {
        char val = (char)Serial.read();
        lastSerialByteMs = millis();

        if (!receivingCommand) {
            if (val == 'M') {
                receivingCommand = true;
                commandIndex = 0;
            }
            continue;
        }

        commandBuffer[commandIndex++] = val;

        if (commandIndex == 3) {
            parseCommand(commandBuffer);

            receivingCommand = false;
            commandIndex = 0;

            // Trigger the message indicator.
            lastMessageMs = millis();
            messageFlashPending = true;
        }
    }
}

void runActuators(bool bab_lower_limit) {
    bool dumper_going_up = !dumper_direction && dumper_pwm;

    if (dumper_going_up || !bab_lower_limit) {
        setPin(dumper_pins, dumper_pwm, dumper_direction);
    } else {
        setPin(dumper_pins, false, false);
    }

    setPin(scooper_pins, scooper_pwm, scooper_direction);
}

void updateStatusLED(uint32_t now, bool bab_lower_limit) {
    const bool dumperRaising = dumper_pwm && !dumper_direction;
    const bool dumperLowering = dumper_pwm && dumper_direction;

    const bool scooperRaising = scooper_pwm && !scooper_direction;
    const bool scooperLowering = scooper_pwm && scooper_direction;

    const bool dumperMoving = dumperRaising || dumperLowering;
    const bool scooperMoving = scooperRaising || scooperLowering;

    const bool limitBlocked = dumperLowering && bab_lower_limit;

    if (limitBlocked) {
        bool ledOn = ((now / LIMIT_BLINK_MS) % 2) == 0;
        setLED(ledOn, false, false);
        return;
    }

    if (messageFlashPending) {
        if ((uint32_t)(now - lastMessageMs) < MESSAGE_FLASH_MS) {
            setLED(true, true, false);
            return;
        }

        messageFlashPending = false;
    }

    if (!dumperMoving && !scooperMoving) {
        setLED(false, false, false);
        return;
    }

    bool showDumper = dumperMoving;

    if (dumperMoving && scooperMoving) {
        showDumper =
            ((now / ACTUATOR_DISPLAY_MS) % 2) == 0;
    }

    if (showDumper) {
        uint32_t interval =
            dumperRaising ? SLOW_BLINK_MS : FAST_BLINK_MS;

        bool ledOn = ((now / interval) % 2) == 0;

        setLED(false, ledOn, false);
    } else {
        uint32_t interval =
            scooperRaising ? SLOW_BLINK_MS : FAST_BLINK_MS;

        bool ledOn = ((now / interval) % 2) == 0;

        setLED(false, false, ledOn);
    }
}

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(scooper_pins[i], OUTPUT);
        pinMode(dumper_pins[i], OUTPUT);
    }

    pinMode(bab_limit_switch_pin, INPUT);

    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    setLED(false, false, false);

    setPin(dumper_pins, false, false);
    setPin(scooper_pins, false, false);

    Serial.begin(9600);
}

void loop() {
    handleSerial(millis());

    bool bab_lower_limit =
        analogRead(bab_limit_switch_pin) < LIMIT_THRESHOLD;

    runActuators(bab_lower_limit);

    updateStatusLED(millis(), bab_lower_limit);
}
