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
  task starttimer (ENDAUTOTIMER);         //start timer task
  task rampcontroller (RampControl);      //start Ramp control task
  task controllerprint (PrintController); //start print to task
#include "autonincludes.h"                //include auto code
}
//////////////////////

// ------ USER CONTROL -------///
void usercontrol(void) { 
  preautoL=false;                         //reset pre auto latch
  MATCHTIMER=0;                           //reset timer
  task starttimer (ENDAUTOTIMER);         //start timer task
  task rampcontroller (RampControl);      //start ramp control task
  task controllerprint (PrintController);
#include "usercontrol.h"
} 
////////////////////////////////////

// ----- MAIN -----//
int main() 
{
  ramp=bwrd;                              //ramp direction starts as 'backward', the next step of enable will cause ramp to move forward
  AutoRunning = 0;                        //auto latch off
  RampR.resetRotation();
  RampL.resetRotation();                  //Reset ramp rotations
  ArmL.resetRotation();
  ArmR.resetRotation();                   //Reset arm rotations
  wait(1000);                             //wait (X) ms
  Gyro.startCalibration();                //start Gyro Calibration
  wait(3000);                             //wait (X) ms
	pre_auton();                            //Run the pre-autonomous function. 
	//Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(auton);
	Competition.drivercontrol(usercontrol);

	//Prevent main from exiting with an infinite loop.                        
	while (1) {
		wait(100);//Sleep the task for a short amount of time to prevent wasted resources.
	}
  return 0;
}
//////////////////////////////////////////