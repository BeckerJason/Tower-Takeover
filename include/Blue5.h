#include "defines.h"
using namespace std;
using namespace vex;
using namespace G;
#define y 60
#define MoveSpeed 80 
AutoRunning=1;
//BRAKE(RampL,coast);BRAKE(RampR,coast);

DontLiftStack=on;
intake=on;

Move(MoveSpeed,12.01,1,coast,5000);
rightDrive(MoveSpeed);
leftDrive(MoveSpeed);
intake=off;
//wait(1000);
Move(y,26,0,brake,5000);
wait(500);
intake = on;//3-4cubes(3green, maybe 1purp)
DontLiftStack=off;
wait(200);
Move(60,-3,1,brake,5000);
wait(300);
Turn(-52,20,5000);
wait(300);
Move(60,10.87,1,brake,5000);
wait(300);
Turn(52,20,5000);
DontLiftStack=on;
wait(300);
Move(20,3.07,1,brake,5000);
wait(300);
intake=off;
Move(40,-4.5,1,brake,5000);//4-5cubes(3green, maybe 1purp)
wait(300);
    intake=off;
     ArmL.setVelocity(100,vex::velocityUnits::pct);
     ArmR.setVelocity(100,vex::velocityUnits::pct);
     ArmR.startRotateTo(400,rotationUnits::deg);
     ArmL.rotateTo(400,rotationUnits::deg);
     wait(200);
     Move(45,4.2,1,coast,10000);
     ManualSpeed=-50;
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
Move(20,19.13,1,brake,5000);
wait(300);
Turn(20,20,5000);
wait(300);
Move(20,6.63,1,brake,5000);
wait(300);
Move(20,-6.63,1,brake,5000);
wait(300);
Turn(80,20,5000);






wait(300);
Move(20,11.38,1,brake,5000);
wait(300);
Turn(90,20,5000);
wait(300);
Move(20,2.83,1,brake,5000);
wait(300);
//cube shit
Turn(79,20,5000);
wait(300);
Move(20,-27,1,brake,5000);
wait(300);
//lift arm a little
Turn(79,20,5000);
wait(300);
//arm down
Move(20,2,1,brake,5000);
wait(300);
Turn(-73,20,5000);
wait(300);
Move(20,-6.84,1,brake,5000);
wait(300);
Turn(73,20,5000);
wait(300);
Move(20,8.66,1,brake,5000);
wait(300);
Move(20,-8.66,1,brake,5000);
wait(300);
Turn(-145,20,5000);
wait(300);
Move(20,17.86,1,brake,5000);
Turn(-35,20,5000);
wait(300);
Move(20,10.87,1,brake,5000);
wait(300);
Turn(-46,20,5000);
wait(300);
Move(20,9.33,1,brake,5000);
wait(300);