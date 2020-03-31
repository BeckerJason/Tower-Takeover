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
Move(20,38.3,1,brake,10000); //4 cubes
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
Turn(36,20,10000);

StopDrive(brake); 
wait(500);
Move(40,-36,1,brake,10000);
wait(500);
Turn(-36,20,10000);
StopDrive(brake); 
wait(500);
intake=on;
Move(15,15,1,brake,10000);
wait(100);
ArmR.startRotateTo(90,vex::rotationUnits::deg);
ArmL.rotateTo(90,vex::rotationUnits::deg);
wait(500);
Move(15,6.5,1,brake,10000);
intake=off;
Move(40,-8,1,brake,10000);
ArmR.startRotateTo(40,vex::rotationUnits::deg);
ArmL.rotateTo(40,vex::rotationUnits::deg);
ArmR.startRotateTo(-2,vex::rotationUnits::deg);
ArmL.startRotateTo(-2,vex::rotationUnits::deg);
wait(400);
intake=on;
wait(700);
Move(15,10,1,brake,10000);
DontLiftStack=on;
wait(500);
intake=off;
Turn(131,25,10000);
StopDrive(brake); 
wait(300);
if(CubeSense2.pressing()==1){DontDropStack=on;}
Move(40,48,1,brake,10000);
ramp=fwrd;
RunRamp=on;
while(RunRamp==on){wait(20);}
Move(40,-2,1,coast,10000);
ArmR.startRotateTo(90,rotationUnits::deg);
ramp=bwrd;
RunRamp=on;
Move(35,-15,1,brake,10000);
ArmR.startRotateTo(0,rotationUnits::deg);
while(1){wait(100);}