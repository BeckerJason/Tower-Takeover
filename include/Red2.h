#include "defines.h"
using namespace std;
using namespace G; 
AutoRunning=1;
intake=on;
DontLiftStack=on;
Move(80,12,1,coast,10000);
leftDrive(80);
rightDrive(80);
intake=off;
Move(40,24,0,brake,10000);
wait(1000);
DontLiftStack=off;
intake=on;
Move(40,8,0,brake,10000);
while(1){wait(100);}