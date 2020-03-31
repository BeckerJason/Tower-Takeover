#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
Color=Red;

AutoRunning=1;
GlobalGyro=0;
// intake=off;
// DontLiftStack=on;
//  rampwheel.suspend();
// RampWheelL.resetRotation();
//  RampWheelR.resetRotation();
 rightDrive(-10);
 leftDrive(-10);
arm.suspend();
      ArmL.setVelocity(100,vex::velocityUnits::pct);
      ArmR.setVelocity(100,vex::velocityUnits::pct);
     
      ArmR.startRotateTo(300,rotationUnits::deg);
      ArmL.rotateTo(300,rotationUnits::deg);
      ManualSpeed=-100;
      intake=manual;
      ArmR.startRotateTo(600,rotationUnits::deg);
      ArmL.rotateTo(600,rotationUnits::deg);
      intake=on;
      RampWheelL.setVelocity(60,vex::velocityUnits::pct);
      RampWheelR.setVelocity(60,vex::velocityUnits::pct);
      RampWheelR.startRotateTo(-150,rotationUnits::deg);
      RampWheelL.startRotateTo(-150,rotationUnits::deg);
//      //ArmL.setVelocity(50,vex::velocityUnits::pct);
//      //ArmR.setVelocity(50,vex::velocityUnits::pct);
      ArmR.startRotateTo(0,rotationUnits::deg);
      ArmL.rotateTo(0,rotationUnits::deg);
      wait(200);
      //arm.suspend();
      //StopArm(hold);
// intake=on;
// DontLiftStack=on;
intake=manual;
ManualSpeed=60;
DontLiftStack=on;

MoveG(50,14,1,0,brake,2500);
intake=off;
wait(200);
ArmL.setVelocity(65,vex::velocityUnits::pct);
ArmR.setVelocity(65,vex::velocityUnits::pct);
ArmR.startRotateTo(430, rotationUnits::deg);
ArmL.startRotateTo(430, rotationUnits::deg);
Turn(90*Color,30,6000);
MoveG(60,5,1,90*Color,brake,8000);
ManualSpeed=-80;
intake=manual;
wait(300);
MoveG(60,-5,1,90*Color,brake,8000);
ArmR.startRotateTo(0, rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
Turn(-90*Color,30,6000);
//wait(200);
MoveG(50,-6,1,0,coast,8000);
MoveG(50,6,1,0,coast,8000);
ramp = fwrd;
RunRamp=on;
MoveG(20,13,0,0,brake,8000);
T3=0;
while(RunRamp&&T3<1500){wait(10);}
RunRamp=off;
ramp = bwrd;
RunRamp=on;
MoveG(60,15,0,0,brake,8000); ///Ram into 4 stack
intake=on;
DontLiftStack=off;
wait(300);

/*if (Color==Blue){ArcTurnG(45,-7);}
else{ArcTurnG(-45,-7);}
DontLiftStack=on;
if(Color==Blue){ArcTurnG(-45,-7);}
else {ArcTurnG(45,-7);}
if (Color==Blue){ArcTurnG(-45,7);}
else{ArcTurnG(45,7);}
DontLiftStack=on;
if(Color==Blue){ArcTurnG(45,7);}
else {ArcTurnG(-45,7);}
MoveG(30,-2,0,0,brake,8000);
*/
MoveG(80,-6,1,0,brake,8000);
//wait(200);
Turn(60*Color,30,6000);
//wait(200);
MoveG(80,-8,1,60*Color,brake,8000);
//wait(200);
Turn(-60*Color,30,6000);
//wait(200);
DontLiftStack=on;
MoveG(40,14,1,0,brake,8000);
intake=off;
//wait(200);
MoveG(80,-10.5,1,0,brake,8000);
ArmR.startRotateTo(325,rotationUnits::deg);
ArmL.startRotateTo(325,rotationUnits::deg);
while(fabs(enc(ArmR))<325&&fabs(enc(ArmL))<325){wait(10);}
MoveG(40,2,1,0,brake,8000);
ManualSpeed=-80;
intake=manual;
wait(300);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.startRotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
MoveG(60,-6,1,0,brake,8000);
//wait(200);
Turn(-30*Color,25,6000);
//wait(200);
MoveG(60,-16,1,-30*Color,brake,8000);
//wait(200);
Turn(120*Color,50,6000);
//wait(200);
MoveG(35,7,1,90*Color,brake,8000);
//wait(200);
MoveG(60,-7,1,90*Color,brake,8000);
//wait(200);
Turn(-180*Color,55,6000);
//wait(200);
MoveG(35,20,1,-90*Color,brake,8000);
//intake=on;
//DontLiftStack=off;
Turn(-25*Color,20,5000);
MoveG(40,12,1,-115*Color,brake,8000);
MoveG(10,4,1,-115*Color,brake,8000);
while(AutoStack()){wait(10);}

while(1){wait(100);}



ArmR.startRotateTo(350, rotationUnits::deg);
ArmL.rotateTo(350, rotationUnits::deg);
ManualSpeed=-40;
intake=manual;
MoveG(65,-12,1,-90*Color,brake,8000);

intake=on;
ramp = fwrd;
RunRamp=on;
MoveG(65,-12,1,-90*Color,brake,8000);
RunRamp=off;
ramp = bwrd;
RunRamp=on;
MoveG(65,12,1,0,brake,8000);
/*while(enc(ArmR)!=0){
  wait(100);
}*/
intake=on;
DontLiftStack=off;
Turn(-35*Color,20,5000);
MoveG(20,16,1,0,coast,8000);
