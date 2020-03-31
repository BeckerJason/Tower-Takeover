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

#include "defines.h"
vex::competition Competition;  
#include "functions.h" 
using namespace vex;
using namespace G;

//
void pre_auton(void) {
  preautoL=true;
#include "pre_auton.h" 
}

//
void auton(void) { 
  preautoL=false;
  G::MATCHTIMER=0;
  task starttimer (ENDAUTOTIMER);
  task rampcontroller (RampControl);
  task controllerprint (PrintController);
#include "autonincludes.h"
}


void usercontrol(void) {
  preautoL=false;
  MATCHTIMER=0; 
  task starttimer (ENDAUTOTIMER);
  task rampcontroller (RampControl);
  task controllerprint (PrintController);
  #include "usercontrol.h"
} 

int main() 
{
  ramp=fwrd;
  AutoRunning = 0;
  RampR.resetRotation();
  RampL.resetRotation();//Reset ramp rotations
  ArmL.resetRotation();
  ArmR.resetRotation();//Reset arm rotations
  wait(1000);
  Gyro.startCalibration();
  wait(3000);
	//Run the pre-autonomous function. 
	pre_auton();

	//Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(auton);
	Competition.drivercontrol(usercontrol);

	//Prevent main from exiting with an infinite loop.                        
	while (1) {
		wait(100);//Sleep the task for a short amount of time to prevent wasted resources.
	}
  return 0;
}