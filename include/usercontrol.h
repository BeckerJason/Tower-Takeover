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
AutoRunning = 0;
// SnapToCube = off;
GlobalCubeOffset = 160;
MATCHTIMER = 0;
OTrack=on;
GTrack=off;
PTrack=on;
  if (!bLeft)
  {
  RampR.startRotateTo(-140,rotationUnits::deg);
  RampL.rotateTo(-140,rotationUnits::deg);
  RampR.resetRotation();
  RampL.resetRotation();  
  }
  
while (1) 
{
  if (bUp==1)
  { StopDrive(hold);}
  else if (AutoRunning == 0 && (abs(ch3) > 10 || abs(ch1) > 10)) {
      CubeTrack=off;
      ToCube=off;
      run(LF, (ch3 + ch1/2)); //(Axis3+Axis4)/2
      run(LM, (ch3 + ch1/2));
      run(LB, (ch3 + ch1/2)); //(Axis3+Axis4)/2
      run(RF, (ch3 - ch1/2)); //(Axis3-Axis4)/2
      run(RM, (ch3 - ch1/2));
      run(RB, (ch3 - ch1/2)); //(Axis3-Axis4)/2
  }
   else if (AutoRunning == 0&&CubeTrack==off&&ToCube==off) {
    StopDrive(brake);
  } else {
  } 
//ARM
  if (bR1==1&&enc(ArmL)<500)//Chain bar arm control
    {
      run(ArmL, 70);
      run(ArmR, 70);
      //ArmL.resetRotation();
    }
    else if (bR2==1&&enc(ArmL)>0)
    {
      run(ArmL, -70);
      run(ArmR, -70);
    }
  else
    {
        
      /*if(abs(enc(ArmL)) <10)
      {
        run(ArmL, 0);
        run(ArmR, 0);
      }                                   
      else
      {*/
        BRAKE(ArmL, hold);
        BRAKE(ArmR, hold);
      //}
    }
/*//CASCADE LIFT
if ((bR1 || bR2) == 1)
{
  CascadeControl();
}
else
{
  StopLift(brake);
}*/






  if (ramp == fwrd) {if (bA && !G::rampprev) {ramp = bwrd;RunRamp=on;}} 
  else if (ramp == bwrd) {if ( bA && !G::rampprev) {ramp = fwrd;RunRamp=on;}}
    G::rampprev=bA ;
    if (bX==1){RunRamp=off;}

  if(enc(RampR)<-100 /*RampLimitBottom.pressing()==0*/||RunRamp==on){}
  else if (bY==1&&bL1==0){run(RampWheelL,-100);run(RampWheelR,-100);}
  else if ( (CubeSense.pressing() || CubeSense2.pressing()) &&  bL1==0){run(RampWheelL,100);run(RampWheelR,100);}
  else {BRAKE(RampWheelL,hold);BRAKE(RampWheelR,hold);}
  
 wait(20);
  if (bL2==1){CubeTrack=on;}
  

}