#include "defines.h"
using namespace std;
using namespace G;
    
//ArmL.start
AutoRunning=1;
intake=on;
Move(25,36,10,brake,10000); //4 cubes
StopDrive(brake);      
//wait(1000);
RampR.startRotateTo(-140,rotationUnits::deg);
RampL.rotateTo(-140,rotationUnits::deg);
RampR.resetRotation();
RampL.resetRotation();

Colors(off,off,on); //O G P
CubeTrack=on; T3=0;
while((CubeTrack==on||ToCube==on)&&T3<4000){wait(10);}
CubeTrack=off;ToCube=off;

Move(20,-20,10,brake,10000);
while(1){wait(100);}