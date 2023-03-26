// Jolty Sample for Packet Serial
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// The Sabertooth is on address 128. We'll name its object ST.
// If you've set up your Sabertooth on a different address, of course change
// that here. For how to configure address, etc. see the DIP Switch Wizard for
//   Sabertooth - http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
//   SyRen      - http://www.dimensionengineering.com/datasheets/SyrenDIPWizard/start.htm
// Be sure to select Packetized Serial Mode for use with this library.
//
// In this sample, arduino due hardware serial TX0 connects to sabertooth S1.

Sabertooth sabertooth(128, Serial); 
                                        
void setup()
{
  Serial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  sabertooth.autobaud();        // Send the autobaud command to the Sabertooth controller(s).
}


void loop()
{
  ST.motor(1, 20);   // Go forward
  delay(2000);       // Wait 2 seconds
  ST.motor(1, 0);    // Stop
  delay(2000);       // Wait 2 seconds
  ST.motor(1, -20);  // Reverse
  delay(2000);       // Wait 2 seconds
  ST.motor(1, 0);    // Stop.
  delay(2000);
}
