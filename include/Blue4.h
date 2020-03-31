#include "defines.h"
using namespace std;
using namespace G;
    

AutoRunning=1;
/*DontLiftStack=on;
intake =on;
ArmL.setVelocity(100,vex::velocityUnits::pct);
ArmR.setVelocity(100,vex::velocityUnits::pct);
ArmR.startRotateTo(200,rotationUnits::deg);
ArmL.rotateTo(200,rotationUnits::deg);
RampR.startRotateTo(-160,rotationUnits::deg);
RampL.rotateTo(-160,rotationUnits::deg);
*/
BRAKE(RampL,coast);
BRAKE(RampR,coast); 
DontLiftStack=off;
//ArmR.startRotateTo(0,rotationUnits::deg);
//ArmL.startRotateTo(0,rotationUnits::deg);
//wait(500);

intake=on;
Move(32,34.7,1,brake,10000); //4 cubes
wait(500);//let ramp fold out
RampR.resetRotation();
RampL.resetRotation();
wait(200);
Turn(-60,25,10000);
wait(200);
Move(45,8,1,brake,10000);
wait(200);
Turn(60,25,10000);
wait(200);
DontLiftStack=on;
Move(45,9.5,1,brake,10000);
wait(200);
DontLiftStack=on;
intake=off;
Move(45,-6.3,1,brake,10000);
ArmL.setVelocity(100,vex::velocityUnits::pct);
ArmR.setVelocity(100,vex::velocityUnits::pct);
ArmR.startRotateTo(380,rotationUnits::deg);
ArmL.rotateTo(380,rotationUnits::deg);
wait(200);
Move(45,4.2,1,coast,10000);
ManualSpeed=-70;
intake=manual;
Move(45,2.1,0,brake,10000);
wait(400);
//double tempG=-GlobalGyro/10;
//Turn(tempG,20,10000);
Move(45,-16.8,0,brake,10000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
Turn(45,20,10000);

StopDrive(brake); 
wait(500);
Move(40,-22.7,1,brake,10000);
wait(500);
Turn(-45,20,10000);
StopDrive(brake); 
wait(500);
intake=on;
Move(25,13.3,1,brake,10000);
wait(300);
ArmL.setVelocity(50,vex::velocityUnits::pct);
ArmR.setVelocity(50,vex::velocityUnits::pct);
ArmR.startRotateTo(110,vex::rotationUnits::deg);
ArmL.rotateTo(110,vex::rotationUnits::deg);
wait(500);
Move(15,5.8,1,brake,10000);
intake=off;
Move(45,-7.4,1,brake,10000);
ArmL.setVelocity(100,vex::velocityUnits::pct);
ArmR.setVelocity(100,vex::velocityUnits::pct);
ArmR.startRotateTo(40,vex::rotationUnits::deg);
ArmL.rotateTo(40,vex::rotationUnits::deg);
ArmR.startRotateTo(-2,vex::rotationUnits::deg);
ArmL.startRotateTo(-2,vex::rotationUnits::deg);
wait(400);
intake=on;
wait(900);
DontLiftStack=on;
Move(45,10.5,1,brake,10000);
wait(500);
intake=off;
Turn(130,20,10000);
StopDrive(brake); 
wait(300);
if(CubeSense2.pressing()==1){DontDropStack=on;}
Move(60,42.1,1,coast,10000);
Move(35,8.4,0,brake,10000);
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
Move(35,-15.8,1,brake,10000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.startRotateTo(0,rotationUnits::deg);
while(1){wait(100);}