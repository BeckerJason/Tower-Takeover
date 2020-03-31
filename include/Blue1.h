#include "defines.h"
using namespace std;
using namespace G;
    
//ArmL.start
AutoRunning=1;
RampR.startRotateTo(-180,rotationUnits::deg);
RampL.rotateTo(-180,rotationUnits::deg);
BRAKE(RampL,coast);
BRAKE(RampR,coast); 
wait(300);
intake=on;
Move(25,38.3,1,brake,10000); //4 cubes
StopDrive(brake);
RampR.resetRotation();
RampL.resetRotation();


  

//wait(1000);

/*Colors(off,off,on); //O G P
CubeTrack=on; T3=0;
while((CubeTrack==on||ToCube==on)&&T3<4000){wait(10);}
CubeTrack=off;ToCube=off;
*/
wait(500);
Turn(36,25,10000);

StopDrive(brake); 
wait(500);
Move(40,-37,1,brake,10000);
wait(500);
Turn(-36,25,10000);
StopDrive(brake); 
wait(500);
ManualSpeed=50;
intake=manual;
Move(15,23,1,brake,10000);
wait(100);
Move(40,-8,1,brake,10000);
wait(200);
intake=on;
Move(15,12.53,1,brake,10000);
intake=off;
wait(500);
Turn(137,25,10000);
StopDrive(brake); 
wait(300);
Move(40,58,1,brake,10000);
ramp=fwrd;
DontDropStack=off;
RunRamp=on;
while(RunRamp==on){wait(20);}
Move(40,-2,1,coast,10000);
ArmR.startRotateTo(50,rotationUnits::deg);
ramp=bwrd;
RunRamp=on;
Move(40,-15,1,brake,10000);
ArmR.startRotateTo(0,rotationUnits::deg);
while(1){wait(100);}