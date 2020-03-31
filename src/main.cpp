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
#define SKILL
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
//   arm.suspend();
// 	rampwheel.suspend();
// cubeload.suspend();
//   stack.suspend();
ramp=bwrd;                              //ramp direction starts as 'backward', the next step of enable will cause ramp to move forward
#ifdef DEBUG
Color=Red;
#endif  
  RampR.resetRotation();
  RampL.resetRotation();                  //Reset ramp rotations
  ArmL.resetRotation();
  ArmR.resetRotation();
#include "pre_auton.h"          //include pre auto code 
}
/////////////////////////
 

// ---- AUTO ---- //
void auton(void) { 
  AutoRunning=1;
  preautoL=false;                         //reset pre auto latch
  G::MATCHTIMER=0;                        //reset Match timer
  // arm.suspend();
	// rampwheel.suspend();
  // cubeload.suspend();
//#include "autonincludes.h"                //include auto code
//Color=Red;
//#include "RedBlue1.h"   //12 points copied from Blue5
//#include "RedBlue2.h"   // 12 points 42 seconds working 2/23/20
#include "RedBlue3.h"
/*#ifdef SKILL
#include "Skills.h"
#endif
#ifndef SKILL
#include "RedBlue3.h"   //
#endif*/

//#include "Blue1.h"  //8points working 11/16
//#include "Blue2.h"    //9 points working, dropping 1 cube 11/16
//#include "Blue3.h"  //12 points 44 seconds working 11/16
//#include "Blue4.h"  //12 points 37 seconds working 11/16
//#include "Blue5.h" //12 points 32 seconds working 11/26
//#include "Red1.h"
//#include "Red2.h"
//#include "SkillsAuto.h"

}

//////////////////////

// ------ USER CONTROL -------///
void usercontrol(void) { 
  preautoL=false;                         //reset pre auto latch
  MATCHTIMER=0;                           //reset timer
  AutoRunning=0;
#include "usercontrol.h"
} 
////////////////////////////////////

// ----- MAIN -----//
int main() 
{
  //std::cout<<"help";
  //AutoRunning=1;
DriveTorque(100);

  
  //AutoRunning = 0;                        //auto latch off
  //stack.suspend();

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