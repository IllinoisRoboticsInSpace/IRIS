#ifndef _PID_
#define _PID_

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	true
  #define MANUAL false
  #define DIRECT  0
  #define REVERSE  1
  #define CALCULATE_BY_MEASUREMENT true
  #define CALCULATE_BY_ERROR false
  #define DEFAULT_MOTOR_MIN -1
  #define DEFAULT_MOTOR_MAX 1


  /*
   Might want to add the ability to have both proportion on meausurement and proportoin on error for mixed processes
  */

  //commonly used functions **************************************************************************
    PID(float*, float*, float*,        // * constructor.  links the PID to the Input, Output, and 
        float, float, float, bool, bool);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
    
    void SetOutputLimits(float, float); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	
  //available but not commonly used functions ********************************************************
    void SetTunings(float, float,       // * While most users will set the tunings once in the 
                    float);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control       	  

    void SetSampleTime(unsigned);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
		void SetDoCompute(bool);

  //Display functions ****************************************************************
	float GetKp() const ;						  // These functions query the pid for interal values.
	float GetKi() const ;						  //  they were created mainly for the pid front-end,
	float GetKd() const ;						  // where it's important to know what is actually inside the PID

  bool GetDoCompute() const ;

  float GetKiReal() const ;
  float GetKdReal() const ;

  private:
	void Initialize();
    
	float kp;                  // * (P)roportional Tuning Parameter
  float ki_given;                  // * (I)ntegral Tuning Parameter given by the user
  float kd_given;                  // * (D)erivative Tuning Parameter given by the yser

  float ki_time_adjusted; //these are so the frequency of updates don't effect the effective ki and kd values but it might not be nessisary if we're never changing the sample rate 
  float kd_time_adjusted;

  float *input_ptr;              // * Pointers to the Input, Output, and Setpoint variables
  float *output_ptr;             //   This creates a hard link between the variables and the 
  float *setpoint_ptr;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
	
  //float use_measurement_ratio; // for if we want a mixed proportion on measurement and proportion on error 


	unsigned long lastTime;
	float outputSum, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool use_proportional_measurement; 
  bool do_compute;
};
#endif

