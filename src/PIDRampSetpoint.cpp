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

#include <PIDRampSetpoint.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint, double* RampedSetPoint, double RiseTime,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    myRampedSetpoint = RampedSetPoint;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 3000;							

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, RiseTime, POn);

    newstart = 1; 
    booststart = 1;

}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

// PID::PID(double* Input, double* Output, double* Setpoint, 
//         double Kp, double Ki, double Kd, int ControllerDirection)
//     :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
// {

// }


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute(){

   if(inAuto){

      unsigned long now = millis();
      timeChange = (now - lastTime);
      
      if(timeChange>=SampleTime){

         if (newstart){  
            Serial.println("PID: compute intialise settings");
            PID::Initialize();
            newstart = 0;
         }      
               
         /*Compute all the working error variables*/
         double input = *myInput;
         TimeFromStart += timeChange; 

         double dInput = (input - lastInput);

         // sigmoid
         //RampedSetpoint = *myTempInitial + (*mySetpoint - *myTempInitial) * (1.0/(1.0 + exp(-GrowthRate * (TimeFromStartSeconds - GrowthOffset)))) ;
         Serial.print("dinput ");
         Serial.println(dInput);

         if(booststart && dInput <= 0){

            Serial.print("mysetpoing before setting static setpoint ");
            Serial.println(*mySetpoint);

            // calculate setpoint to ensure maxouput
            SPMaxoutputT0 = outMax/kp + input;

            if (SPMaxoutputT0 > *mySetpoint){
               *myRampedSetpoint = *mySetpoint;
               EnableRamp = 0;
            } else{
               *myRampedSetpoint = SPMaxoutputT0;
               EnableRamp = 1;
            }
            Serial.print("static setpoint ");
            Serial.println(*myRampedSetpoint);
            Serial.print("mysetpoint after setting static setpoint ");
            Serial.println(*mySetpoint);
         } else if (booststart && dInput > 0){
            booststart = 0;
            PID::InitializeRamp();
            Serial.print("time from start");
            Serial.println(TimeFromStart);
            *myRampedSetpoint = *mySetpoint - EnableRamp * exp(-1.0 * (GrowthRate * TimeFromStart - GrowthOffset));
            Serial.print("Ramp enabled is: ");
            Serial.println(EnableRamp);
            Serial.print("1st ramp setpoint ");
            Serial.println(*myRampedSetpoint);
            Serial.print("time from start");
            Serial.println(TimeFromStart);
         } else {
            *myRampedSetpoint = *mySetpoint - EnableRamp * exp(-1.0 * (GrowthRate * TimeFromStart - GrowthOffset));
            Serial.print("Ramp enabled is: ");
            Serial.println(EnableRamp);
            Serial.print("ramp setpoint ");
            Serial.println(*myRampedSetpoint);
            Serial.print("time from start");
            Serial.println(TimeFromStart);
            Serial.print("GrowthRate");
            Serial.println(GrowthRate);
            Serial.print("GrowthOffset");
            Serial.println(GrowthOffset);
         }


         double error = *myRampedSetpoint - input;

         outputSum += (ki * error);

         /*Add Proportional on Measurement, if P_ON_M is specified*/
         if(!pOnE) outputSum-= kp * dInput;

         if(outputSum > sumoutMax) outputSum = sumoutMax;
         else if(outputSum < outMin) outputSum = outMin;

         /*Add Proportional on Error, if P_ON_E is specified*/
         double output;
         if(pOnE) output = kp * error;
         else output = 0;

         /*Compute Rest of PID Output*/
         output += outputSum - kd * dInput;

         if(output > outMax) output = outMax;
         else if(output < outMin) output = outMin;
         *myOutput = output;

         // //REMOVE! ONLY FOR PARAMETER FINDING
         // *myOutput = 0.05 * outMax;

         /*Remember some variables for next time*/
         lastInput = input;
         lastTime = now;
         
         return true;
            
      } else {
      return false;
      }
   } else {
      return false;
   } 
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, double RiseTime, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

    // set exponential rise time to be faster than the time th
    // that the pid is in coldstart settings
    // TODO - pass parameter over to user
    double TimeAtSetpoint = 1;
    riseTime = RiseTime - TimeAtSetpoint;

    // growth rate from 20C to setpoint,
    // if above 20C to begin with will reach setpoint faster
    if (riseTime <= 0.1 || (*mySetpoint - 20) <= 0){
       GrowthRate = 1;
       Serial.println("invalid parameters for growth rate");
       Serial.print("rise time ");
       Serial.println(riseTime);
       Serial.print("set point - 20");
       Serial.println(*mySetpoint - 20);

    } else{
      GrowthRate = (1.5 + log(*mySetpoint - 20))/(riseTime * 60 * 1000);
    }

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, riseTime, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
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

   // TODO: give user access to this parameter
   sumoutMax = 0.15 * outMax;
   
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > sumoutMax) outputSum = sumoutMax;
	   else if(outputSum < outMin) outputSum = outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
      newstart = 1;
      booststart = 1;
    }
    inAuto = newAuto;
}

/* Reset()****************************************************************
 *	function available outside class to trigger a newstart, 
 * initalising all parameters again
 ******************************************************************************/
void PID::Reset(){
   newstart = 1;
   booststart = 1;
}


/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = 0;
   lastInput = *myInput;

}

void PID::InitializeRamp(){
   
   TimeFromStart = 0;
   lastTime = millis() - SampleTime;
   timeChange = 0;

   Serial.print("current temp");
   Serial.println(*myInput);
   
   // deal with log(-x) -> -infinity
   if (EnableRamp && (*mySetpoint - SPMaxoutputT0) > 0){
      GrowthOffset = log(*mySetpoint - SPMaxoutputT0);
   } else{
      Serial.println("invalid growth offset ");
      Serial.print("ramp enabled is ");
      Serial.println(EnableRamp);
      Serial.print("SPmaxoutputT0 ");
      Serial.println(SPMaxoutputT0);
      Serial.print("setpoint ");
      Serial.println(*mySetpoint);
      Serial.print("setpoint - SPMaxoutputT0: ");
      Serial.println(*mySetpoint - SPMaxoutputT0);
      GrowthOffset = 0;
   }

}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

