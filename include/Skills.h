#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
GlobalGyroT=0;
GlobalGyro=0;
wait(200);

RampL.resetRotation();
RampR.resetRotation();  
ArmL.resetRotation();
ArmR.resetRotation();
AutoRunning=1;
//INITIAL START
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
/////////INITIAL END  
DontLiftStack=off;
intake=on;
MoveG(50,10.75,1,0,coast,3000);
leftDrive(60);
rightDrive(60);
GlobalGyroT=0;
MoveG(25,22.56,0,0,brake,6000);
MoveG(35,-9,1,0,brake,6000);
//wait(1000);//leave time for ramp deploy
// wait(250);
// T(Color*45,25,5000);
// wait(250);
// Move(50,-9,1,brake,5000);
// wait(250);
// Turn(Color*-45,25,5000);
if (Color==Blue){ArcTurnG(-45,14);}
else{ArcTurnG(45,14);}
DontLiftStack=on;
if(Color==Blue){ArcTurnG(45,14);}
else {ArcTurnG(-45,14);}
MoveG(40,-2,1,0,brake,6000);
//float tempturn=Inertial.heading();
//Turn(-tempturn,20,2000);
MoveG(25,9,1,0,brake,6000);
// Move(45,14,1,brake,5000);
//wait(250);
MoveG(40,-8,1,0,brake,5000);
intake=off;
//wait(200);
ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(420,rotationUnits::deg);
ArmL.startRotateTo(420,rotationUnits::deg);
T4=0;
while (fabs(enc(ArmL))<400 && T4<2000)
{wait(20);}
StopArm(hold);
ManualSpeed=-60;
intake=manual;
MoveG(25,6,1,0,brake,5000);
wait(300);
MoveG(60,-29,1,0,coast,6000);
//tempturn=Inertial.heading();
//Turn(-tempturn,20,2000);
//leftDrive(-60);
//rightDrive(-60);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.startRotateTo(0,rotationUnits::deg);
//Move(60,-10,0,brake,6000);
//wait(250);
intake=on;
DontLiftStack=off;
// Turn(Color*55,25,5000);
// wait(200);
// Move(40,-17,1,brake,5000);
// wait(200);
// Turn(Color*-55,20,5000);
// wait(200);
//tempturn=Inertial.heading();
//Turn(-tempturn,20,2000);
Turn(Color*-90,30,5000);
MoveG(40,19,1,90,brake,5000);
MoveG(25,-3,1,90,brake,5000);
ArmR.startRotateTo(150,rotationUnits::deg);
ArmL.rotateTo(150,rotationUnits::deg);
Turn(Color*90,30,5000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
MoveG(30,10,1,0,brake,5000);
wait(200);
MoveG(30,-3.5,1,0,brake,5000);
intake=on;
ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(190,rotationUnits::deg);
ArmL.rotateTo(190,rotationUnits::deg);
MoveG(15,7.5,1,0,brake,5000);
DontLiftStack=off;
intake=off;
// ArmR.setVelocity(100,velocityUnits::pct);
// ArmL.setVelocity(100,velocityUnits::pct);
// ArmR.startRotateTo(150,rotationUnits::deg);
// ArmL.rotateTo(150,rotationUnits::deg);
//wait(500);
MoveG(35,-7.5,1,0,brake,5000);

ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
wait(300);

//wait(1000);
DontLiftStack=on;
MoveG(35,3,1,0,brake,5000);
T4=0;
while(CubeSense2.pressing()&&T4<700){wait(20);}
MoveG(35,9,1,0,brake,5000);
wait(250);
Turn(Color*120,30,10000);
MoveG(40,4,1,Color*120,coast,7000); //50dist
Turn(Color*-30,30,10000);
//ManualSpeed=0;
intake=off;
wait(300);

ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(420,rotationUnits::deg);
ArmL.startRotateTo(420,rotationUnits::deg);
wait(100);
DontLiftStack=off;
T3=0;//T3 gets reset in cube load anyway...but...
MoveG(40,26,1,Color*90,brake,7000);
while((abs(enc(ArmR))<400||abs(enc(ArmL)<400))&&T3<2500){wait(10);}
intake=manual;
ManualSpeed=-80;
//cubeload.suspend();
wait(300);
intake=off;
MoveG(40,-26,1,Color*90,brake,7000);
DontLiftStack=off;
ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
Turn(Color*30,30,10000);
MoveG(40,44,1,Color*120,coast,7000);
leftDrive(20);
rightDrive(20);
MoveG(10,4,0,Color*120,coast,3000);
leftDrive(10);
rightDrive(10);
MoveG(10,1,0,Color*120,brake,2000);
//intake=off;
//DontLiftStack=on;
//DontDropStack=off;
//Move(15,-1,0,brake,2000);
if(CubeSense2.pressing()==1){DontDropStack=on;}
      //24 19
ramp=fwrd;
AutoStack();                      //1st stack
/*double TempG=GlobalGyro;
Turn(-(1200-abs(TempG)),25,10000);
MoveG(35,-3,1,Color*120,brake,10000);
Turn(Color*-120,30,10000);
intake=on;
MoveG(60,36,1,0,coast,6000);
MoveG(35,60,1,0,brake,10000);
MoveG(35,-12,1,0,brake,10000);
CubeLoad();
arm.resume();
IntakeController.resume();
Turn(Color*90,30,10000);
ManualSpeed=-80;
intake=manual;
wait(300);
Turn(Color*-90,30,10000);
intake=off;
ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
MoveG(40,20,1,0,brake,7000);
Turn(Color*55,30,10000);
MoveG(40,12,1,Color*55,coast,7000);
leftDrive(20);
rightDrive(20);
MoveG(10,6,0,Color*55,brake,3000);
//intake=off;
//DontLiftStack=on;
//DontDropStack=off;
//Move(15,-a1,0,brake,2000);
if(CubeSense2.pressing()==1){DontDropStack=on;}

ramp=fwrd;

AutoStack();*/
//ArmR.startRotateTo(0,rotationUnits::deg);
//ArmL.startRotateTo(0,rotationUnits::deg);
/*Turn(142,40,2000);
wait(100);
intake=on;
DontLiftStack=off;
Move(60,27.18,1,brake,4000);
Move(60,-14.3,1,brake,4000);
Turn(162,40,4000);
task cubeload (CubeLoad);
Move(60,29.17,1,coast,5000);
ManualSpeed=-70;
intake=manual;
Move(30,4,0,brake,2000);
Move(30,-5,1,brake,2000);*/
while(1){wait(100);}