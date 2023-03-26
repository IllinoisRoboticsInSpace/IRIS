// !!UNFINISHED!!

#include <Sabertooth.h>

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
Sabertooth st_dr(128, Serial1);
byte odroid_dr;
Sabertooth st_bl(129, Serial1);
byte odroid_bl;
byte odroid_in[2];

#define dir_1 22
#define pwm_1 2
#define dir_2 23
#define pwm_2 3

#define dir_3 24
#define pwm_3 4
#define dir_4 25
#define pwm_4 5

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  st_dr.autobaud();

  // apportion correct motor values using define statements
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(dir_2, OUTPUT);

  pinMode(pwm_3, OUTPUT);
  pinMode(dir_3, OUTPUT);
  pinMode(pwm_4, OUTPUT);
  pinMode(dir_4, OUTPUT);

  st_dr.motor(1, 0);
  st_dr.motor(2, 0);
}

// 0 means that no error
int CRC_check()
{
  // split into separate function to avoid confusion
  unsigned int odroid_in_concat = odroid_in[1] + odroid_in[0] * 256; // concatenates odroid_in into a 16-bit int
  unsigned int CRC = 18;                                             // CRC error checking (black box?)
  unsigned int msg = odroid_in_concat % 2048;
  unsigned int checksum = odroid_in_concat / 2048;
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

  Serial.readBytes(odroid_in, 2);

  if (CRC_check())
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
  int motorVal = odroid_in[0] % 8; // bits 2:0
  if (motorVal == 7) // stop code
  {
    Stop();
  }
  int power = odroid_in[1] % 128; // bits 7:3
  int invert = odroid_in[1] / 128; // bit 8
  if (motorVal < 3)
  { // call a sabertooth for the action
    if (invert) // checking if power (2s complement) is negative (most significant bit is 1)
    {
      power *= -1;
    }

    if (((motorVal % 4) / 2) == 0) {
      st_dr.motor(motorVal % 2 + 1, power);
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
    digitalWrite(dir_1, (int)(odroid_in[1] / 128)); // controls the direction the motor;
    analogWrite(pwm_1, power * 2);
    break;
  case 4:
    digitalWrite(dir_2, (int)(odroid_in[1] / 128)); // controls the direction the motor;
    analogWrite(pwm_2, power * 2);
    break;
  case 5:
    digitalWrite(dir_3, (int)(odroid_in[1] / 128)); // controls the direction the motor;
    analogWrite(pwm_3, power * 2);
    break;

  case 6:
    digitalWrite(dir_4, (int)(odroid_in[1] / 128)); // controls the direction the motor;
    analogWrite(pwm_4, power * 2);
    break;

  default:
    return;
  }
}
void Stop()
{
  for (int i = 3; i < 7; i++)
  {
    linAC(0, i);
  }
  st_dr.stop();
  st_bl.stop();
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
