#ifndef IRIS_PID_h
#define IRIS_PID_CPP

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

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(bool);               // * sets PID to either Manual (false) or Auto (true)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);         	  

	// void SetControllerDirection(bool);	  // * Sets the Direction, or "Action" of the controller. DIRECT
	// 									  //   means the output will increase when error is positive. REVERSE
	// 									  //   means the opposite.  it's very unlikely that this will be needed
	// 									  //   once it is set in the constructor.
    void SetSampleTime(unsigned);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp() const ;						  // These functions query the pid for interal values.
	double GetKi() const ;						  //  they were created mainly for the pid front-end,
	double GetKd() const ;						  // where it's important to know what is actually 
	bool get_in_auto() const;						  //  inside the PID.
	//bool is_forward() const;					  //

  private:
	void Initialize();
	
	// double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	// double dispKi;				//   format for display purposes
	// double dispKd;				//
    
	double kp;                  // * (P)roportional Tuning Parameter
  double ki_given;                  // * (I)ntegral Tuning Parameter given by the user
  double kd_given;                  // * (D)erivative Tuning Parameter given by the yser

  double ki_time_adjusted;
  double kd_time_adjusted;

	//int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, calculate_by_measurement;
};
#endif

