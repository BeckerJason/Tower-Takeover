#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
GlobalGyroT=0;
wait(200);
RampL.resetRotation();
RampR.resetRotation();  
AutoRunning=1;


DontLiftStack=on;
intake =on;

IntakeControler.resume();
rampwheel.resume();
rampcontroller.resume();

ArmL.setVelocity(80,vex::velocityUnits::pct);
ArmR.setVelocity(80,vex::velocityUnits::pct);
ArmL.resetRotation();
ArmR.resetRotation(); 
ArmR.startRotateTo(180,rotationUnits::deg);
ArmL.rotateTo(180,rotationUnits::deg);
wait(400);
ArmL.startRotateTo(66,rotationUnits::deg);
ArmR.rotateTo(66,rotationUnits::deg);
wait(300);
ArmR.startRotateTo(66,rotationUnits::deg);
ArmL.rotateTo(66,rotationUnits::deg);
ArmL.resetRotation();
  ArmR.resetRotation(); 
RampR.startRotateTo(-240,rotationUnits::deg); 
RampL.rotateTo(-240,rotationUnits::deg);
RampR.startRotateTo(-200,rotationUnits::deg); 
RampL.rotateTo(-200,rotationUnits::deg);
run(RampR,40);
run(RampL,40);
wait(400);
DontLiftStack=off;
 
BRAKE(RampL,coast);
BRAKE(RampR,coast);
wait(500);
RampL.resetRotation();
RampR.resetRotation();  
DontLiftStack=off;
intake=on;
Move(35,33.5,1,brake,5000);
wait(1000);//leave time for ramp deploy
Turn(Color*-42,40,5000);
wait(250);
Move(45,11.04,1,brake,5000);
wait(250);
Turn(Color*42,40,5000);
wait(250);
DontLiftStack=on;
Move(45,3.75,1,brake,5000);
wait(250);

Move(40,-2,1,brake,5000);
intake=off;
wait(200);
ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(420,rotationUnits::deg);
ArmL.startRotateTo(420,rotationUnits::deg);
T4=0;
while (fabs(enc(ArmL))<420 && T4<1750)
{wait(10);}
StopArm(hold);
ManualSpeed=-60;
intake=manual;
Move(25,4,1,brake,5000);
wait(500);
Move(35,-24.56,1,brake,6000);
wait(250);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
Turn(Color*55,30,5000);
wait(250);
Move(30,-16,1,brake,5000);
wait(250);
Turn(Color*-55,30,5000);
wait(250);
Move(30,14.5,1,brake,5000);
wait(750);
Move(30,-2,1,brake,5000);

ArmR.setVelocity(50,velocityUnits::pct);
ArmL.setVelocity(50,velocityUnits::pct);
ArmR.startRotateTo(150,rotationUnits::deg);
ArmL.rotateTo(150,rotationUnits::deg);
Move(20,5.5,1,brake,5000);
DontLiftStack=on;
intake=off;
wait(500);
Move(35,-7.5,1,brake,5000);
ArmR.setVelocity(50,velocityUnits::pct);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
wait(1000);
DontLiftStack=on;
Move(35,12,1,brake,5000);
wait(250);
Turn(Color*130,40,8000);
intake=off;
Move(40,50,1,coast,7000);
intake=on;
DontLiftStack=off;
Move(20,4,0,brake,2000);
intake=off;
DontLiftStack=on;
DontDropStack=off;
Move(15,-1,0,brake,2000);
if(CubeSense2.pressing()==1){DontDropStack=on;}
ramp=fwrd;
RunRamp=on;
while(RunRamp==on){wait(20);}
//Move(40,-2.1,1,coast,10000);
ArmL.setVelocity(50,vex::velocityUnits::pct);
ArmR.setVelocity(50,vex::velocityUnits::pct);
ArmL.startRotateTo(90,rotationUnits::deg);
ArmR.startRotateTo(90,rotationUnits::deg);
ramp=bwrd;
RunRamp=on;
Move(20,-15.8,1,brake,10000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.startRotateTo(0,rotationUnits::deg);
while(1){wait(100);}