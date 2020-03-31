/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jason                                                     */
/*    Created:      Monday July 29 2019                                        */
/*    Description:  V5 project                                                */
/*                                                                             */
/*----------------------------------------------------------------------------*/
/*                                                                             */
/*   ALL FUNCTION PROTOTYPES PLACED IN DEFINES.H                               */
/*   ALL GLOBAL VARIABLES PLACED IN NAMESPACE 'G' IN DEFINES.H                 */
/*   ALL FUNCTIONS IN FUNCTIONS.H                                              */
/*                                                                             */
/*                                                                             */
/*                                                                             */
/*                                                                             */
/*----------------------------------------------------------------------------*/

#include "defines.h"            //variable and function initialization and definitions
vex::competition Competition;   //tell robot to use competition file format
#include "functions.h"          //Functions / tasks 
using namespace vex;            //using vex namespace (vex provided)
using namespace G;              //using global namespace

// ---- PRE AUTO ---- ///
void pre_auton(void) {
  preautoL=true;                //set pre auto latch
#include "pre_auton.h"          //include pre auto code 
}
/////////////////////////


// ---- AUTO ---- //
void auton(void) { 
  preautoL=false;                         //reset pre auto latch
  G::MATCHTIMER=0;                        //reset Match timer
//#include "autonincludes.h"                //include auto code
//#include "RedBlue1.h"
//#include "Blue1.h"  //8points working 11/16
//#include "Blue2.h"    //9 points working, dropping 1 cube 11/16
//#include "Blue3.h"  //12 points 44 seconds working 11/16
//#include "Blue4.h"  //12 points 37 seconds working 11/16
//#include "Blue5.h"
//#include "Red1.h"
#include "Red2.h"
}

//////////////////////

// ------ USER CONTROL -------///
void usercontrol(void) { 
  preautoL=false;                         //reset pre auto latch
  MATCHTIMER=0;                           //reset timer
#include "usercontrol.h"
} 
////////////////////////////////////

// ----- MAIN -----//
int main() 
{
  
  LF.setMaxTorque(100, percentUnits::pct);
  RF.setMaxTorque(100, percentUnits::pct);
  LB.setMaxTorque(100, percentUnits::pct);
  RB.setMaxTorque(100, percentUnits::pct);
  LM.setMaxTorque(100, percentUnits::pct);
  RM.setMaxTorque(100, percentUnits::pct);
  
  ramp=bwrd;                              //ramp direction starts as 'backward', the next step of enable will cause ramp to move forward
  AutoRunning = 0;                        //auto latch off
  RampR.resetRotation();
  RampL.resetRotation();                  //Reset ramp rotations
  ArmL.resetRotation();
  ArmR.resetRotation();                   //Reset arm rotations
  wait(500);                             //wait (X) ms
  Gyro.startCalibration();                //start Gyro Calibration
  wait(2000);                             //wait (X) ms
	pre_auton();                            //Run the pre-autonomous function. 
	//Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(auton);
	Competition.drivercontrol(usercontrol);
  vex::task rampcontroller (RampControl);      //start ramp control task
	//Prevent main from exiting with an infinite loop.                        
	while (1) {
		wait(100);//Sleep the task for a short amount of time to prevent wasted resources.
	if (bL1&&bL2&&bR1&&bR2)
  {
    arm.suspend();
    rampcontroller.suspend();
    IntakeControler.suspend();
    timer2.suspend();
    gyrotrack.suspend();
    rampwheel.suspend();
    printscreen.suspend();
      intake= off;
  RunRamp=off;
  CubeTrack =off;
   OTrack=off;
   PTrack=off;
   GTrack=off;
  ToCube=off;
  DontLiftStack=off;
  DontDropStack=off;
ramp = bwrd;
    wait(500);
    arm.resume();
    rampcontroller.resume();
    IntakeControler.resume();
    timer2.resume();
    gyrotrack.resume();
    rampwheel.resume();
    printscreen.resume();
  }
  }
  return 0;
}
//////////////////////////////////////////