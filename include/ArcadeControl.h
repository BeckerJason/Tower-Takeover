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
  { 
    StopDrive(hold);        //Defend Position
  }

  else if (AutoRunning == 0 && (abs(ch3) > 10 || abs(ch1) > 10)) 
  { 
    FSpeed =ch3;
    TSpeed =0.33*ch1;
    CubeTrack=off; ToCube=off;
    run(LF, (FSpeed + TSpeed)); 
    run(LM, (FSpeed + TSpeed));
    run(LB, (FSpeed + TSpeed)); 
    run(RF, (FSpeed - TSpeed)); 
    run(RM, (FSpeed - TSpeed));
    run(RB, (FSpeed - TSpeed));
  }

  else if (AutoRunning == 0 && CubeTrack==off && ToCube==off) 
  {
    StopDrive(brake);
    FSpeed = 0;
    TSpeed = 0;
  } 
  // else if (AutoRunning == 0 && CubeTrack==off && ToCube==off)
  // {
  //   StopDrive(brake);
  // }

  else {} 

	//ARM

	if (ramp == fwrd) {if (bA && !G::rampprev) {ramp = bwrd;RunRamp=on;}}
	else if (ramp == bwrd) {if ( bA && !G::rampprev) {ramp = fwrd;RunRamp=on;}}
	G::rampprev=bA ;
	if (bLeft||bDown){RunRamp=off;}

	if(bL1){DontLiftStack=on;DontDropStack=on;}//dont drop stack while scoring//dont lift stack while stacking
	else {DontLiftStack=off;DontDropStack=off;}

	//if (bL2==1){CubeTrack=on;}
	if (bX){task cubeload (CubeLoad);}
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
    wait(5);
  }
  wait(5);
}