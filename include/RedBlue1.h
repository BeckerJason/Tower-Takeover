#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;

Color=Blue;
AutoRunning=1;

intake=off;
DontLiftStack=on;
 rampwheel.suspend();
RampWheelL.resetRotation();
 RampWheelR.resetRotation();
rightDrive(-10);
leftDrive(-10);
     ArmL.setVelocity(100,vex::velocityUnits::pct);
     ArmR.setVelocity(100,vex::velocityUnits::pct);
     arm.suspend();
     ArmR.startRotateTo(600,rotationUnits::deg);
     ArmL.rotateTo(600,rotationUnits::deg);
     RampWheelL.setVelocity(60,vex::velocityUnits::pct);
     RampWheelR.setVelocity(60,vex::velocityUnits::pct);
     RampWheelR.startRotateTo(-150,rotationUnits::deg);
     RampWheelL.startRotateTo(-150,rotationUnits::deg);
     ArmL.setVelocity(50,vex::velocityUnits::pct);
     ArmR.setVelocity(50,vex::velocityUnits::pct);
     ArmR.startRotateTo(0,rotationUnits::deg);
     ArmL.rotateTo(0,rotationUnits::deg);
     arm.suspend();
     StopArm(hold);
intake=on;
DontLiftStack=on;
wait(250);
Move(65,10.3555,1,coast,2000);
rampwheel.resume();
rightDrive(60);
leftDrive(60);
intake=off;
Move(55,33,0,brake,3000);//31.4311
wait(500);
DontLiftStack=off;intake=on;
Move(60,-15.525,1,brake,5000);//-14.025
wait(100);
T(-60*Color,20,5000);
wait(100);
Move(60,10,1,brake,5000);
DontLiftStack=on;
wait(100);
//Turn(60,20,5000);
T(0*Color,20,3000);
wait(100);
// GTrack=on;
// PTrack=off;
// OTrack=off;
// CubeTrack=on;
// while(CubeTrack==on){wait(10);}
Move(60,13,1,brake,5000);
// for(int i=0;i<2;i++)
// {
// Colors(on,on,on);
// if(GreenCube.exists[0])
// {
// Colors(off,off,on);
// DontLiftStack=on;
// ToCube=on;
// T4=0;
// while(ToCube==on&&T4<3000){wait(10);}
// wait(500);
// break;
// }
// else if(OrangeCube.exists[0]||PurpleCube.exists[0])
// {
// DontLiftStack=off;
// ToCube=on;
// T4=0;
// while(ToCube==on&&T4<3000){wait(10);}
// wait(500);
// }

// }
//////////////
wait(100);
Move(60,-5.5,1,brake,5000);//4-5cubes(3green, maybe 1purp)
    intake=off;
     ArmL.setVelocity(50,vex::velocityUnits::pct);
     ArmR.setVelocity(50,vex::velocityUnits::pct);
     ArmR.startRotateTo(400,rotationUnits::deg);
     ArmL.rotateTo(400,rotationUnits::deg);
     Move(45,4.2,1,coast,10000);
     ManualSpeed=-50;
     intake=manual;
     Move(45,2.1,0,brake,10000);
     //double tempG=-GlobalGyro/10;
     //Turn(tempG,20,10000);
     Move(45,-5.14104,0,brake,10000);
     ArmR.startRotateTo(0,rotationUnits::deg);
     ArmL.rotateTo(0,rotationUnits::deg);
     intake=on;
     DontLiftStack=off;
// Move(45,5.14104,0,brake,2000);
// wait(100);     
// T(60,30,5000);
// wait(100);
// Move(40,18.1575,1,coast,5000);
// wait(100);
// T(80,30,5000);
// wait(100);
// Move(20,8,1,brake,2000);
// wait(100);
// T(92,30,2000);
// wait(100);
// Move(20,4,1,brake,2000);
// wait(100);
// T(85,30,2000);
// Move(40,-12,1,brake,5000);
// wait(100);
// T(80,30,3000);
// wait(100);
// Move(60,-34,1,brake,8000);
// wait(100);
// wait(100);
T(80*Color,30,3000);
wait(100);
Move(60,-14,1,brake,3000);
T(180*Color,30,5000);
// GTrack=on;
// PTrack=off;
// OTrack=off;
// CubeTrack=on;
// while(CubeTrack==on){wait(100);}
wait(100);
Move(40,12,1,brake,8000);
wait(100);
T(130*Color,30,5000);//135
wait(100);
Move(60,12.25,1,brake,8000);
wait(100); 
Turn(-10*Color,30,5000);//-35
// GTrack=off;
// PTrack=on;
// OTrack=off;
// CubeTrack=on;
// while(CubeTrack==on){wait(100);}
// wait(100);
Move(50,17,1,brake,8000);
// CubeTrack=on;
// while(CubeTrack==on){wait(100);}
// wait(100);
Move(30,10,1,brake,8000);
wait(200);
Move(60, -3,1,brake,3000);
wait(100);
Turn(-45*Color,30,5000);
wait(100);
Move(60,-7,1,brake,8000);
wait(100);
Turn(50*Color,30,5000);
// GTrack=on;
// PTrack=off;
// OTrack=off;
// CubeTrack=on;
// while(CubeTrack==on){wait(100);}
wait(100);
Move(30, 15,1,brake,3000);
wait(100);
Move(60, -22,1,brake,3000);
wait(100);
//T(0*Color,30,5000);
//T(-88*Color,30,5000);//-92
Turn(-175,30,5000);
wait(100);
Move(25, 23,1,brake,3000);
AutoStack();

/*Move(60,-4.1575,1,brake,5000);
GTrack=off;
PTrack=off;
OTrack=on;
wait(100);
CubeTrack=on;
while(CubeTrack==on){wait(100);}
ArmR.startRotateTo(240,rotationUnits::deg);
ArmL.rotateTo(240,rotationUnits::deg); intake=manual;
CubeTrack=off;
ManualSpeed=40;
Move(15,8,1,brake,5000);
intake=off;
wait(100);
Move(60,-6.02236,1,brake,5000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
wait(500);
ArmR.startRotateTo(100,rotationUnits::deg);
ArmL.rotateTo(100,rotationUnits::deg);
wait(100);
Move(15,8.02236,1,brake,5000);intake=off;
wait(100);
Move(60,-6.02236,1,brake,5000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
wait(500);
Move(60,8.86696,1,brake,5000);
wait(100);
Move(60,-13.265,1,brake,5000);
wait(100);
Turn(-20,20,5000);
DontLiftStack=on;
wait(100);
Move(60,21.7536,1,brake,5000);
wait(100);
Move(60,-14.6887,1,brake,5000);
wait(100);
Turn(70,20,5000);

intake=off;
     ArmL.setVelocity(100,vex::velocityUnits::pct);
     ArmR.setVelocity(100,vex::velocityUnits::pct);
     ArmR.startRotateTo(565,rotationUnits::deg);
     ArmL.rotateTo(565,rotationUnits::deg);
     wait(200);
     Move(45,21.3608,1,coast,10000);
     ManualSpeed=-50;
     intake=manual;
     Move(45,2.1,0,brake,10000);
     wait(400);
     //double tempG=-GlobalGyro/10;
     //Turn(tempG,20,10000);
     Move(45,-9.84141,1,brake,10000);
     ArmR.startRotateTo(0,rotationUnits::deg);
     ArmL.rotateTo(0,rotationUnits::deg);
     intake=on;
     DontLiftStack=off;
     wait(100);
Turn(100,20,5000);
wait(100);
Move(60,36.281,1,brake,5000);
wait(100);
Turn(-15,20,5000);
wait(100);
Move(60,9.51092,1,brake,5000);
*/