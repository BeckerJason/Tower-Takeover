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
#ifndef DEBUG
//#define DEBUG
#endif

#include "defines.h"            //variable and function initialization and definitions
vex::competition Competition;   //tell robot to use competition file format
#include "functions.h"          //Functions / tasks 
using namespace vex;            //using vex namespace (vex provided)
using namespace G;              //using global namespace

// ---- PRE AUTO ---- ///
void pre_auton(void) {
  preautoL=true;                //set pre auto latch
  ArmL.resetRotation();
 ArmR.resetRotation();
 LF.setMaxTorque(100, percentUnits::pct);
  RF.setMaxTorque(100, percentUnits::pct);
  LB.setMaxTorque(100, percentUnits::pct);
  RB.setMaxTorque(100, percentUnits::pct);
  LM.setMaxTorque(100, percentUnits::pct);
  RM.setMaxTorque(100, percentUnits::pct);
  RampL.resetRotation();
 RampR.resetRotation();
 ramp=bwrd;                              //ramp direction starts as 'backward', the next step of enable will cause ramp to move forward
#include "pre_auton.h"          //include pre auto code 
}
/////////////////////////


// ---- AUTO ---- //
void auton(void) { 
  preautoL=false;                         //reset pre auto latch
  G::MATCHTIMER=0;                        //reset Match timer
  AutoRunning=1;
  // printscreen.resume();                  
  // rampcontroller.resume();
  // rampwheel.resume();
  // IntakeController.resume();
  // arm.suspend();
  // cubes.resume();

//#include "autonincludes.h"                //include auto code
//Color=Red;
#include "RedBlue1.h"
//#include "Blue1.h"  
//#include "Blue2.h"    
//#include "Blue3.h"  
//#include "Blue4.h"  
//#include "Blue5.h"
//#include "Red1.h"
//#include "Red2.h"
//Move(60,72,1,brake,100000);
//Move(80,20.01,1,coast,5000);

//#include "Blue6.h" //Working 11 points 11/30/19


}

//////////////////////

// ------ USER CONTROL -------///
void usercontrol(void) { 
  preautoL=false;                         //reset pre auto latch
  MATCHTIMER=0;                           //reset timer
  //   printscreen.resume();                  
  // rampcontroller.resume();
  // rampwheel.resume();
  // IntakeController.resume();
  // arm.resume();
  // cubes.suspend();
       
//#include "usercontrol.h"
#include "ArcadeControl.h"
} 
////////////////////////////////////

// ----- MAIN -----//
int main() 
{
  // LF.setMaxTorque(100, percentUnits::pct);
  // RF.setMaxTorque(100, percentUnits::pct);
  // LB.setMaxTorque(100, percentUnits::pct);
  // RB.setMaxTorque(100, percentUnits::pct);
  // LM.setMaxTorque(100, percentUnits::pct);
  // RM.setMaxTorque(100, percentUnits::pct);
  
  // ramp=bwrd;                              //ramp direction starts as 'backward', the next step of enable will cause ramp to move forward
  // AutoRunning = 0;                        //auto latch off

  // ArmL.resetRotation();
  // ArmR.resetRotation();
                     //wait (X) ms
	pre_auton();                            //Run the pre-autonomous function. 
	//Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(auton);
	Competition.drivercontrol(usercontrol);
  //vex::task rampcontroller (RampControl);      //start ramp control task
	//Prevent main from exiting with an infinite loop.                        
	while (1) {
		wait(100);//Sleep the task for a short amount of time to prevent wasted resources.
	
  }
  return 0;
}
//////////////////////////////////////////