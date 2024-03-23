#include <Arduino.h>
#include "DebugTools.h"
#include "generated/commands.h"

void setup() {
  Serial.begin(115200);//For printing

  // Must set ENABLE_DEBUG in DebugTools.h to true
  DEBUG_PRINTLN("DEBUG_PRINTLN");
  DEBUG_PRINT("DEBUG_PRINT\n");
  DEBUG_PRINTF("Print twenty four: %d\n", 24);
  Sabertooth_Config_Data proto;
  proto.set_motorID(2);
  proto.set_enabled(true);
  DEBUG_PRINT_MESSAGE(proto);
}

void loop() {
  
}