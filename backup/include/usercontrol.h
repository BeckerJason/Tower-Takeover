#include "C:/Program Files (x86)/VEX Robotics/VEXcode/sdk/vexv5/include/v5_apitypes.h"
#include "C:/Program Files (x86)/VEX Robotics/VEXcode/sdk/vexv5/include/vex_global.h"
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
GlobalFlagOffset = 160;
MATCHTIMER = 0;
while (1) {
  if (AutoRunning == 0 && (abs(ch3) > 10 || abs(ch1) > 10)) {
    
      run(LF, (ch3 + ch1/2)); //(Axis3+Axis4)/2
      run(LM, (ch3 + ch1/2));
      run(LB, (ch3 + ch1/2)); //(Axis3+Axis4)/2
      run(RF, (ch3 - ch1/2)); //(Axis3-Axis4)/2
      run(RM, (ch3 - ch1/2));
      run(RB, (ch3 - ch1/2)); //(Axis3-Axis4)/2

       
    
  } else if (AutoRunning == 0) {
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
  else if ( (CubeSense.value(percentUnits::pct)<68 || CubeSense2.value(percentUnits::pct)<68) &&  bL1==0){run(RampWheelL,100);run(RampWheelR,100);}
  else {BRAKE(RampWheelL,hold);BRAKE(RampWheelR,hold);}
  
 if(enc(RampR)<-100 /*RampLimitBottom.pressing()==0*/||RunRamp==on){} 
  else if (intake == off) {if (bB && !G::intakeprev) intake = on;run(RightRoller, 100); run(LeftRoller, 100);} 
  else if (intake == on) { 

    if ( bB && !G::intakeprev) intake = off;run(RightRoller, 0); run(LeftRoller, 0);}
       G::intakeprev = bB;
       //INTAKE
if (bY==1){run(RightRoller, -80); run(LeftRoller, -80);}
  wait(20);

}