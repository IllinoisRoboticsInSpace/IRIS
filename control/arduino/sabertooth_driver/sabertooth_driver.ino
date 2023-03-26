#include <Sabertooth.h>
#include "motor_pin_config.h"

#define DEBUG false

/* available addresses for packetized serial mode are 128-135
 * i.e. switches 1-3 are DOWN and the rest are up to user choice
 *
 * Description from http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm:
 * Packetized serial mode will allow you to control many motors using just one shared serial wire
 * Bytes are 8N1, and the data rate is automatically detected when you send 0xAA to the Sabertooth
 * Switch 3 enables lithium cutoff mode
 */

// Abbreviations: st -> Sabertooth, dr -> drive, bl -> rbucket/ladder
// serial input -> odroid
Sabertooth drive_sabertooth(DRIVE_SABER_ADDRESS, DRIVE_SABER_SERIAL);
Sabertooth S2_sabertooth(S2_SABER_ADDRESS, S2_SABER_SERIAL);
Sabertooth S3_sabertooth(S3_SABER_ADDRESS, S3_SABER_SERIAL);

// Old
byte odroid_dr;
Sabertooth st_bl(129, Serial1);
byte odroid_bl;
byte serial_in_buffer[2];


void setup()
{
  DRIVE_SABER_SERIAL.begin(9600);
  drive_sabertooth.autobaud();

  drive_sabertooth.motor(DRIVE_LEFT_MOTOR, 0);
  drive_sabertooth.motor(DRIVE_RIGHT_MOTOR, 0);

  // Init the 10 amp board
  pinMode(A_PWM,OUTPUT);
  pinMode(A_DIRECTION,OUTPUT);

  DRIVE_SABER_SERIAL.begin(9600);
  drive_sabertooth.autobaud();
  S2_SABER_SERIAL.begin(9600);
  S2_sabertooth.autobaud();
  S3_SABER_SERIAL.begin(9600);
  S3_sabertooth.autobaud();
}

// 0 means that no error
int CRC_check()
{
  // split into separate function to avoid confusion
  unsigned int serial_in_buffer_concat = serial_in_buffer[1] + serial_in_buffer[0] * 256; // concatenates serial_in_buffer into a 16-bit int
  unsigned int CRC = 18;                                             // CRC error checking (black box?)
  unsigned int msg = serial_in_buffer_concat % 2048;
  unsigned int checksum = serial_in_buffer_concat / 2048;
  unsigned int curr = msg * 32 + checksum;

  int a, index;
  while (curr >= 32)
  {
    index = largestOneIndex(curr); // CRC checking based on the index of high (1) bits found
    a = CRC << (index - 4);
    curr ^= a;
  }
  if (curr)
  {
    // if CRC came back bad
    // do some error handling here??
    return 1;
  }

  return 0;

}

void loop()
{
  Serial.readBytes(serial_in_buffer, 2);
  if (CRC_check() == 1)
  {
    // if CRC came back bad
    // do some error handling here??
    return;
  }

  // First byte: contains motorVal and checksum, in the Order Checksum:[7:3],motorVal[2:0]
  // Second byte: is the power value.
  /*
   * Input[0],[1],[2] correspond to motor
   * KEY:
   * Motor ID:
   * 0: Left Drive
   * 1: Right Drive
   * 2: Bucket Ladder M
   * 3: dump conveyor LA
   * 4: bucket ladder LA
   * 5: agitator
   * 6: dump conveyor M
   * 7: stops motors
   */
  int motorVal = serial_in_buffer[0] % 8; // bits 2:0
  if (motorVal == 7) // stop code
  {
    stop();
  }
  int power = serial_in_buffer[1] % 128; // bits 7:3
  int invert = serial_in_buffer[1] / 128; // bit 8
  if (motorVal < 3)
  { // call a sabertooth for the action
    if (invert) // checking if power (2s complement) is negative (most significant bit is 1)
    {
      power *= -1;
    }

    if (((motorVal % 4) / 2) == 0) {
      drive_sabertooth.motor(motorVal % 2 + 1, power);
    } else {
      st_bl.motor(motorVal % 2 + 1, power);
    }
  }
  else
  { // call a linAC
    linAC(power, motorVal);
  }
}

void linAC(int power, int motor)
{
  switch (motor)
  {
  case 3:
    digitalWrite(A_DIRECTION, (int)(serial_in_buffer[1] / 128)); // controls the direction the motor;
    analogWrite(A_PWM, power * 2);
    break;

  default:
    return;
  }
}
void stop()
{
  for (int i = 0; i < 7; i++)
  {
    linAC(0, i);
  }
  drive_sabertooth.stop();
  S2_sabertooth.stop();
  S3_sabertooth.stop();
}

int largestOneIndex(unsigned int curr)
{
  int i = 15;
  while (i >= 0)
  {
    if (curr & (1 << i))
    {
      break;
    }
    i--;
  }
  return i;
}
