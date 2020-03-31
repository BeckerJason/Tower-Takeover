#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
//Color=Red;

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
intake=manual;
ManualSpeed=70;
DontLiftStack=on;

MoveG(50,9,1,0,brake,2500);
ManualSpeed=30;
intake=off;
MoveG(50,5,1,0,brake,2500);
intake=off;
wait(200);
ArmL.setVelocity(65,vex::velocityUnits::pct);
ArmR.setVelocity(65,vex::velocityUnits::pct);
ArmR.startRotateTo(430, rotationUnits::deg);
ArmL.startRotateTo(430, rotationUnits::deg);
Turn(88*Color,30,6000);
MoveG(60,5,1,88*Color,brake,8000);
ManualSpeed=-80;
intake=manual;
wait(300);
MoveG(60,-4.5,1,88*Color,brake,8000);
ArmR.startRotateTo(0, rotationUnits::deg);
ArmL.startRotateTo(0,rotationUnits::deg);
wait(125);
Turn(-86*Color,30,6000);
// wait(150);
MoveG(50,-6,1,0,brake,8000);
MoveG(40,6,1,0,coast,8000);
leftDrive(30);
rightDrive(30);
ramp = fwrd;
RunRamp=on;
MoveG(15,12,0,0,brake,8000);//13
T3=0;
while(RunRamp&&T3<1000){wait(10);}//1500
RunRamp=off;
ramp = bwrd;
RunRamp=on;
MoveG(60,18,0,0,brake,8000); ///Ram into 4 stack//15
//intake=on;
ManualSpeed=50;
intake=manual;
DontLiftStack=off;
wait(300);
MoveG(60,-27,0,0,brake,8000);
// wait(125);
Turn(88*Color,30,6000);
DontLiftStack=off;//on
// wait(125);
MoveG(75,8,1,88*Color,brake,8000);
// wait(100);
MoveG(75,-7,1,88*Color,brake,8000);
ManualSpeed=40;
// wait(125);
Turn(-120*Color,30,6000);
// wait(125);
MoveG(60,18,1,-120*Color,brake,8000);
// wait(125);
Turn(20*Color,30,6000);
// wait(125);
// intake=off;
// ArmR.startRotateTo(325,rotationUnits::deg);
// ArmL.startRotateTo(325,rotationUnits::deg);
MoveG(60,6.5,1,0,brake,8000);
// while(fabs(enc(ArmR))<325&&fabs(enc(ArmL))<325){wait(10);}
MoveG(40,2,1,0,brake,8000);
// ManualSpeed=-80;
// intake=manual;
// wait(300);
// ArmR.startRotateTo(0,rotationUnits::deg);
// ArmL.startRotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
// MoveG(60,-3.5,1,0,brake,8000);
MoveG(35,7,1,0,coast,8000);
MoveG(60,-7,1,0,brake,8000);//-3.5
// wait(125);
Turn(-18*Color,30,6000);
// wait(125);
MoveG(60,-19,1,-18*Color,brake,8000);
// wait(125);
Turn(-65*Color,30,6000);
// wait(125);
MoveG(50,25,1,-90*Color,brake,8000);
MoveG(50,-2,1,-90*Color,brake,8000);
// wait(125);
ArcTurnG(65*Color, -12.75);
ArcTurnG(-70*Color, -14.75);
// Turn(-25*Color,30,6000);
// wait(200);
MoveG(55,18.5,1,-130*Color,coast,8000);
MoveG(30,8,0,-130*Color,coast,8000);
MoveG(10,3,0,-130*Color,brake,2000);
rightDrive(10);
wait(350);
StopDrive(brake);
wait(200);
AutoStack();

