/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, bool use_proportional_measurement, bool do_compute)
{
    output_ptr = Output;
    input_ptr = Input;
    setpoint_ptr = Setpoint;

    this->use_proportional_measurement = use_proportional_measurement;
    this->do_compute = do_compute;

    outMin = DEFAULT_MOTOR_MIN;
    outMax = DEFAULT_MOTOR_MAX;
												
    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - SampleTime;
    lastInput = 0;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute() {
  if(!do_compute){
    return false;
  }
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime){
      /*Compute all the working error variables*/
      float error = (*setpoint_ptr) - (*input_ptr);
      float dInput = (*input_ptr) - lastInput;
      // outputSum+= (ki_time_adjusted * error);
      outputSum+= (ki_given * error);

    
      if(use_proportional_measurement){
        outputSum-= kp * dInput;
      } 

      if(outputSum > outMax) {
        outputSum = outMax;
      }
      else if(outputSum < outMin){
        outputSum = outMin;
      } 

	  float output;
      if(!use_proportional_measurement){
        output = kp * error;
      }
      else{
        output = 0;
      } 
      
      // outputSum -= kp * error * use_measurement_ratio;
      // float output = kp * error * (1 - use_measurement_ratio); change to use a ratio of muesurement and error 

      /*Compute Rest of PID Output*/
      // output += outputSum - (kd_time_adjusted * dInput);
      
      output += outputSum - (kd_given * dInput);

	    if(output > outMax) {
            output = outMax;
        }
        else if(output < outMin){
           output = outMin; 
        } 
	    *output_ptr = output;

      /*Remember some variables for next time*/
        lastInput = *input_ptr;
        lastTime = now;
	    return true;
   }
   return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd) {
   kp = Kp;
   ki_given = Ki; 
   kd_given = Kd;

  //  ki_time_adjusted = (Ki / 1000) * SampleTime; // adjusting the constants to keep the same behavior when the sample time changes (might change as we won't change the sample rate). ddivides by 1000 since the Sample time is in milliseconds and we don't want the actual pid constants to be change by to much 
  //  kd_time_adjusted = (Kd * 1000) / SampleTime;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(unsigned NewSampleTime){
      float ratio  = (float)NewSampleTime / (float)SampleTime;
      ki_time_adjusted *= ratio;
      kd_time_adjusted /= ratio;
      SampleTime = NewSampleTime;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

	   if(*output_ptr > outMax) *output_ptr = outMax;
	   else if(*output_ptr < outMin) *output_ptr = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
// void PID::SetMode(bool Mode)
// {
//     bool newAuto = (Mode == AUTOMATIC);
//     if(newAuto && !inAuto)
//     {  /*we just went from manual to auto*/
//         PID::Initialize();
//     }
//     inAuto = newAuto;
// }

void PID::SetDoCompute(bool set_do_compute){
  if(set_do_compute == do_compute){
    return;
  }
  do_compute = set_do_compute;
  outputSum = 0;
  lastInput = 0;
  lastTime = millis();

}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *output_ptr;
   lastInput = *input_ptr;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

float PID::GetKp() const { return kp; }
float PID::GetKi() const { return ki_given;}
float PID::GetKd() const { return kd_given;}
float PID::GetKiReal() const {return ki_time_adjusted;}
float PID::GetKdReal() const {return kd_time_adjusted;}

bool PID::GetDoCompute() const {return do_compute;}


