#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
AutoRunning=1;
IntakeControler.resume();
rampwheel.resume();
arm.resume();
rampcontroller.resume();
/*DontLiftStack=on;
intake =on;
ArmL.setVelocity(100,vex::velocityUnits::pct);
ArmR.setVelocity(100,vex::velocityUnits::pct);
ArmR.startRotateTo(200,rotationUnits::deg);
ArmL.rotateTo(200,rotationUnits::deg);
*/
  //RampR.startRotateTo(-290,rotationUnits::deg); 
  //RampL.rotateTo(-290,rotationUnits::deg);
  //run(RampR,40);
  //run(RampL,40);
  //wait(300);
  BRAKE(RampL,coast);
  BRAKE(RampR,coast);
DontLiftStack=off;
//ArmR.startRotateTo(0,rotationUnits::deg);
//ArmL.startRotateTo(0,rotationUnits::deg);
//wait(500);
DontLiftStack=off;
intake=on;
Move(50,41.25,0,brake,6000);
wait(300);
T(20,60,5000);
wait(300);
Move(50,14,1,brake,10000);
wait(300);
T(0,60,5000);
wait(300);
Move(50,18,1,brake,10000);
wait(300);
T(-34,60,5000);
wait(300);
Move(50,5.55,1,brake,10000);
T(0,60,10000);
Move(50,20,1,brake,10000);
wait(300);
T(90,60,10000);
wait(300);
DontLiftStack=on;
task cubeload (CubeLoad);
wait(1000);
Move(30,12,1,coast,10000);
leftDrive(30);
rightDrive(30);
ManualSpeed=-70;
intake=manual;
Move(30,2,0,brake,10000);
wait(300);
Move(30,-9.72,1,brake,10000);
wait(300);
intake=on;
ArmL.startRotateTo(0,rotationUnits::deg);
ArmR.rotateTo(0,rotationUnits::deg);
T(0,60,5000);
wait(300);
Move(30,16.37,1,brake,10000);
wait(300);
T(45,60,10000);
wait(300);
Move(30,8.57,1,brake,5000);
wait(300);
if(CubeSense2.pressing()==1){DontDropStack=on;}

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
arm.resume();
 wait(20);