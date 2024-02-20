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
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, bool use_proportional_measurement)
{
    output_ptr = Output;
    input_ptr = Input;
    setpoint_ptr = Setpoint;

    this->use_proportional_measurement = use_proportional_measurement;

    outMin = DEFAULT_MOTOR_MIN;
    outMax = DEFAULT_MOTOR_MAX;
												
    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - SampleTime;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute() {
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime){
      /*Compute all the working error variables*/
      double input = *input_ptr;
      double error = *setpoint_ptr - *input_ptr;
      double dInput = (*input_ptr - lastInput);
      outputSum+= (ki_time_adjusted * error);

      //if(error < 10) outputSum = 0; resets error count if close enough to setpoint

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(use_proportional_measurement){
        outputSum-= kp * dInput;
      } 

      if(outputSum > outMax) {
        outputSum = outMax;
      }
      else if(outputSum < outMin){
        outputSum = outMin;
      } 

      /*Add Proportional on Error, if P_ON_E is specified*/
	  double output;
      if(!use_proportional_measurement){
        output = kp * error;
      }
      else{
        output = 0;
      } 

      /*Compute Rest of PID Output*/
      output += outputSum - (kd_time_adjusted * dInput);

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
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd) {
   kp = Kp;
   ki_given = Ki; 
   kd_given = Kd;

   ki_time_adjusted = (Ki / 1000) * SampleTime; // adjusting the constants to keep the same behavior when the sample time changes (might change as we won't change the sample rate). ddivides by 1000 since the Sample time is in milliseconds and we don't want the actual pid constants to be change by to much 
   kd_time_adjusted = (Kd * 1000) / SampleTime;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(unsigned NewSampleTime){
      double ratio  = (double)NewSampleTime / (double)SampleTime;
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
void PID::SetOutputLimits(double Min, double Max)
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

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp() const { return kp; }
double PID::GetKi() const { return ki_given;}
double PID::GetKd() const { return kd_given;}

