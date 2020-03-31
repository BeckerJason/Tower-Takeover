#include "defines.h"
using namespace G;
using namespace std;
using namespace vex;

#ifdef DEBUG // if DEBUG is not defined earlier this section will not compile
/*while(DEBUG){
    if (bUp == 1) { GLOBALP += .01; wait(100); }
            if (bDown == 1) { GLOBALP -= .01; wait(100); }
    if (bX== 1) { GLOBALI += .0000001; wait(100); }
            if (bB == 1) {GLOBALI -= .0000001; wait(100); }
    if (bR1== 1) { GLOBALD += .05; wait(100); }
            if (bR2 == 1) {GLOBALD -= .05; wait(100); }
            wait(10);
}  */
#endif 
DontDropStack=off;
DontLiftStack=off;
AutoRunning = 0;
// SnapToCube = off;
GlobalCubeOffset = 160;
MATCHTIMER = 0;
OTrack=on;
GTrack=off;
PTrack=on;
cubeload.suspend();
IntakeControler.resume();
rampwheel.resume();
  /*if (!bLeft)
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
      run(LF, (ch3 + ch1/3)); //(Axis3+Axis4)/2
      run(LM, (ch3 + ch1/3));
      run(LB, (ch3 + ch1/3)); //(Axis3+Axis4)/2
      run(RF, (ch3 - ch1/3)); //(Axis3-Axis4)/2
      run(RM, (ch3 - ch1/3));
      run(RB, (ch3 - ch1/3)); //(Axis3-Axis4)/2
  }
   else if (AutoRunning == 0&&CubeTrack==off&&ToCube==off) {
    StopDrive(brake);
  } else {
  } 
//ARM
  
  if (ramp == fwrd) {if (bA && !G::rampprev) {ramp = bwrd;RunRamp=on;}} 
  else if (ramp == bwrd) {if ( bA && !G::rampprev) {ramp = fwrd;RunRamp=on;}}
    G::rampprev=bA ;
    if (bX==1){RunRamp=off;}

  if(bL1){DontLiftStack=on;DontDropStack=on;}//dont drop stack while scoring//dont lift stack while stacking
  else {DontLiftStack=off;DontDropStack=off;}

  //if (bL2==1){CubeTrack=on;}
  if (bLeft){task cubeload (CubeLoad);}
  if (bDown){cubeload.suspend();}
if (bRight)
{if(CubeSense2.pressing()==1){DontDropStack=on;}

ramp=fwrd;
RunRamp=on;
while(RunRamp==on){wait(20);}
//Move(40,-2.1,1,coast,10000);
arm.suspend();
ArmL.setVelocity(20,vex::velocityUnits::pct);
ArmR.setVelocity(20,vex::velocityUnits::pct);
ArmL.startRotateTo(120,rotationUnits::deg);
ArmR.startRotateTo(120,rotationUnits::deg);
ramp=bwrd;
Move(15,-5.8,1,brake,10000);
RunRamp=on;
Move(15,-6.8,1,brake,10000);
arm.resume();}
 wait(20);
}