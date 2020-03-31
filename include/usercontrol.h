#include "defines.h"
using namespace G;
using namespace std;
using namespace vex;

#ifdef DEBUG // if DEBUG is not defined earlier this section will not compile
/*while(DEBUG){
if (bUp == 1) { GLOBALP += .01; wait(100); }
if (bDown == 1) { GLOBALP -= .01; wait(100); }
if (bLeft== 1) { GLOBALI += .0000001; wait(100); }
if (bB == 1) {GLOBALI -= .0000001; wait(100); }
if (bR1== 1) { GLOBALD += .05; wait(100); }
if (bR2 == 1) {GLOBALD -= .05; wait(100); }
wait(10);
}  */
#endif
 RampL.resetRotation();
RampR.resetRotation();  
ArmL.resetRotation();
ArmR.resetRotation();
AutoRunning=1;
leftDrive(-10);
rightDrive(-10);
DontLiftStack=on;
ManualSpeed=-100;
intake =manual;
//Resume controller tasks
IntakeController.resume();
rampwheel.resume();
rampcontroller.resume();
arm.suspend();
wait(50);
ArmL.setVelocity(80,vex::velocityUnits::pct);
ArmR.setVelocity(80,vex::velocityUnits::pct); 
ArmR.startRotateTo(180,rotationUnits::deg);
ArmL.rotateTo(180,rotationUnits::deg);
//run(ArmL,70);
//run(ArmR,70);
wait(500);
intake=on;
ArmL.startRotateTo(66,rotationUnits::deg);
ArmR.rotateTo(66,rotationUnits::deg);
wait(200);
RampR.startRotateTo(-260,rotationUnits::deg); 
RampL.rotateTo(-260,rotationUnits::deg);
RampR.startRotateTo(-200,rotationUnits::deg); 
RampL.rotateTo(-200,rotationUnits::deg);
run(RampR,40);
run(RampL,40);
wait(300);
DontLiftStack=off; 
BRAKE(RampL,coast);
BRAKE(RampR,coast);
wait(500);
StopArm(hold);
ArmL.resetRotation();
ArmR.resetRotation();
RampL.resetRotation();
RampR.resetRotation();  
DontLiftStack=off;
intake=on;
DontDropStack=off;
DontLiftStack=off;
AutoRunning = 0;
intake=off;
// SnapToCube = off;
GlobalCubeOffset = 160;
MATCHTIMER = 0;
OTrack=on;
GTrack=off;
PTrack=on;
cubeload.suspend();
IntakeController.resume();
rampcontroller.resume();
rampwheel.resume();
arm.resume();

/*if (!bX)
{
RampR.startRotateTo(-140,rotationUnits::deg);
RampL.rotateTo(-140,rotationUnits::deg);
RampR.resetRotation();
RampL.resetRotation();
}*/

while (1) 
{
	if (bUp==1)
	{ StopDrive(hold);}
	else if (AutoRunning == 0 && (abs(ch3) > 10 || abs(ch1) > 10)) {
		CubeTrack=off;
		ToCube=off;
		run(LF, (ch3 + ch1*.3)); //(Axis3+Axis4)/2
		run(LM, (ch3 + ch1*.3));
		run(LB, (ch3 + ch1*.3)); //(Axis3+Axis4)/2
		run(RF, (ch3 - ch1*.3)); //(Axis3-Axis4)/2
		run(RM, (ch3 - ch1*.3));
		run(RB, (ch3 - ch1*.3)); //(Axis3-Axis4)/2
	}
	else if (AutoRunning==0)
	{
		StopDrive(brake);
	}

	//ARM

	if (ramp == fwrd) {if (bA && !G::rampprev) {ramp = bwrd;RunRamp=on;}}
	else if (ramp == bwrd) {if ( bA && !G::rampprev) {ramp = fwrd;RunRamp=on;}}
	G::rampprev=bA ;
	if (bLeft||bDown){RunRamp=off;}

	if(bL1){DontLiftStack=on;DontDropStack=on;}//dont drop stack while scoring//dont lift stack while stacking
	else {DontLiftStack=off;DontDropStack=off;}

	//if (bL2==1){CubeTrack=on;}
	if (bX){vex::task cubeload (CubeLoad);}
	else if (bLeft){cubeload.suspend();}
	if (bRight){vex::task stack (AutoStack);}
	else if (bX){stack.stop();}
	else{}
	if (bR1||bR2){arm.resume();}
	while(bDown||manualprev==true)
	{
		if(bDown)
		{
			if(manualprev==false)
			{
				StopDrive(brake);
        IntakeController.suspend();
				arm.suspend();
				//rampcontroller.suspend();
        RunRamp=off;
			}

			if(bX)
			{
				run(RampR,-20);run(RampL,-20);
				if(abs(enc(RampL))<200){RampL.resetRotation();RampR.resetRotation();}
			}
			else if(bB)
			{
				run(RampR,20);run(RampL,20);
				if(abs(enc(RampL))<200){RampL.resetRotation();RampR.resetRotation();}
			}
			else{StopRamp(coast);}

			if (bR1)
			{
				ArmL.resetRotation();ArmR.resetRotation();
				run(ArmR,60);run(ArmL,60);
			}
			else if (bR2)
			{
				ArmL.resetRotation();ArmR.resetRotation();
				run(ArmR,-60);run(ArmL,-60);
			}
			else{StopArm(hold);}

			manualprev=true;
		}
    else if(manualprev==true)
    {
      manualprev=false;
      StopRamp(coast);
      arm.resume();
      IntakeController.resume();
    }
    if (bL1&&bL2&&bR1&&bR2)
  {
    StopDrive(hold);
    arm.suspend();
    rampcontroller.suspend();
    IntakeController.suspend();
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
    IntakeController.resume();
    timer2.resume();
    gyrotrack.resume();
    rampwheel.resume();
    printscreen.resume();
  }
    wait(15);
  }
  wait(20);
}