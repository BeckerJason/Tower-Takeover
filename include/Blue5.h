#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
    
AutoRunning=1;
/*DontLiftStack=on;
intake =on;
ArmL.setVelocity(100,vex::velocityUnits::pct);
ArmR.setVelocity(100,vex::velocityUnits::pct);
ArmR.startRotateTo(200,rotationUnits::deg);
ArmL.rotateTo(200,rotationUnits::deg);
*/
//RampR.startRotateTo(-160,rotationUnits::deg);
//RampL.rotateTo(-160,rotationUnits::deg);
BRAKE(RampL,coast);
BRAKE(RampR,coast); 
DontLiftStack=off;
//ArmR.startRotateTo(0,rotationUnits::deg);
//ArmL.startRotateTo(0,rotationUnits::deg);
//wait(500);
DontLiftStack=off;
intake=on;
Move(25,34,1,brake,5000);
wait(750);//leave time for ramp deploy
Turn(-42,25,5000);
wait(250);
Move(35,11.04,1,brake,5000);
wait(250);
Turn(42,25,5000);
wait(250);
DontLiftStack=on;
Move(35,3.75,1,brake,5000);
wait(250);
intake=off;
ArmR.setVelocity(100,velocityUnits::pct);
ArmL.setVelocity(100,velocityUnits::pct);
ArmR.startRotateTo(380,rotationUnits::deg);
ArmL.rotateTo(380,rotationUnits::deg);
ManualSpeed=-70;
intake=manual;
Move(25,2,1,brake,5000);
wait(500);
Move(35,-24.56,1,brake,6000);
wait(250);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
Turn(55,25,5000);
wait(250);
Move(30,-18.75,1,brake,5000);
wait(250);
Turn(-55,25,5000);
wait(250);
Move(30,13.5,1,brake,5000);
wait(750);
DontLiftStack=on;
ArmR.setVelocity(50,velocityUnits::pct);
ArmL.setVelocity(50,velocityUnits::pct);
ArmR.startRotateTo(150,rotationUnits::deg);
ArmL.rotateTo(150,rotationUnits::deg);
Move(20,5.5,1,brake,5000);
intake=off;
wait(500);
Move(35,-5.5,1,brake,5000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.rotateTo(0,rotationUnits::deg);
intake=on;
DontLiftStack=off;
wait(1000);
DontLiftStack=on;
Move(35,8.5,1,brake,5000);
wait(250);
intake=off;
Turn(130,20,8000);
Move(40,50,1,coast,7000);
Move(25,6,0,brake,7000);
wait(300);
DontDropStack=off;
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
Move(35,-15.8,1,brake,10000);
ArmR.startRotateTo(0,rotationUnits::deg);
ArmL.startRotateTo(0,rotationUnits::deg);
while(1){wait(100);}