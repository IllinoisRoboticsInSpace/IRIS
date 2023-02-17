#include <Sabertooth.h>

SoftwareSerial mySerial1(19,18);
SoftwareSerial mySerial2(17,16);
Sabertooth ST[2] = { Sabertooth(128,mySerial1), Sabertooth(129,mySerial2) }; //128 ->Left and right Drive, 129->Bucket Ladder
byte OdroidIn[2];

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
  SabertoothTXPinSerial.begin(9600);
  Sabertooth::autobaud(SabertoothTXPinSerial);
  mySerial1.begin(9600);
  Serial.begin(9600); //serial input from Odroid.
  

pinMode(pwm_1,OUTPUT);
pinMode(dir_1,OUTPUT);
pinMode(pwm_2,OUTPUT);
pinMode(dir_2,OUTPUT);

pinMode(pwm_3,OUTPUT);
pinMode(dir_3,OUTPUT);
pinMode(pwm_4,OUTPUT);
pinMode(dir_4,OUTPUT);


}

void loop()
{
   
  Serial.readBytes(OdroidIn,2); 
   unsigned int OdroidInInt = OdroidIn[1]+OdroidIn[0]*256;
   unsigned int CRC = 18;
   unsigned int msg = OdroidInInt % 2048;
   unsigned int checksum = OdroidInInt / 2048;
   unsigned int curr = msg*32 + checksum;

   int a, index;
   while (curr >= 32) {
    index = largestOneIndex(curr);
    a = CRC << (index - 4);
    curr ^= a;
   }
   if (curr) {
    // CRC came back bad
    // do some error handling here??
    return;
   }
   
  //First byte: contains motorVal and checksum, in the Order Checksum:[7:3],motorVal[2:0]
  //Second byte: is the power value.
  /*
   * Input[0],[1],[2] correspond to motor
  KEY:
  Motor ID: 
0: Left Drive
1: Right Drive
2: Bucket Ladder M
3: dump conveyor LA
4: bucket ladder LA
5: agitator
6: dump conveyor M
7: stops motors
  */
  int motorVal = OdroidIn[0] % 8; 
  int power = OdroidIn[1] %128;
  if(OdroidIn[1]/128 == 1 && motorVal<3){
      power*=-1;
    }
    if(motorVal == 7){
      Stop();
      } 
  else if(motorVal<3){ //call a sabertooth for the action
    ST[(motorVal%4)/2].motor(motorVal%2+1,power);
    }
  else{//call a linAC
    linAC(power,motorVal);
    }
 
}

 void linAC(int power, int motor){
    switch(motor) {

   case 3 :
      digitalWrite(dir_1, (int)(OdroidIn[1]/128)); //controls the direction the motor;
      analogWrite(pwm_1, power*2); 
      break;   case 4 :
      digitalWrite(dir_2, (int)(OdroidIn[1]/128)); //controls the direction the motor;
      analogWrite(pwm_2, power*2);
      break; 
  case 5 :
      digitalWrite(dir_3, (int)(OdroidIn[1]/128)); //controls the direction the motor;
      analogWrite(pwm_3, power*2);
      break;
  
   case 6 :
      digitalWrite(dir_4, (int)(OdroidIn[1]/128)); //controls the direction the motor;
      analogWrite(pwm_4, power*2
      break;  
  
  default : 
   return;
}
  
  }
void Stop(){
  for(int i = 3; i<7; i++){
    linAC(0,i);
    }
  ST[0].stop();
  ST[1].stop();
  
  }

int largestOneIndex(unsigned int curr) {
  int i = 15;
  while (i >= 0) {
    if (curr & (1 << i)) {
      break;
    }
    i--;
  }
  return i;
}
