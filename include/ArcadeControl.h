#include "defines.h"
using namespace G;
using namespace std;
using namespace vex;

#ifdef DEBUG // if DEBUG is not defined earlier this section will not compile
/*while(DEBUG){
    if (bUp == 1) { GLOBALP += .01; wait(100); }
            if (bDown == 1) { GLOBALP -= .01; wait(100); }
    if (bX== 1) { GLOBALI += .0000001; wait(100); }
            if (bB == 1) {GLOBALI -= .0000001; wait(100); }
    if (bR1== 1) { GLOBALD += .05; wait(100); }
            if (bR2 == 1) {GLOBALD -= .05; wait(100); }
            wait(10);
}  */
#endif 
AutoRunning = 0;
// SnapToCube = off;
GlobalCubeOffset = 160;
MATCHTIMER = 0;
OTrack=on;
GTrack=off;
PTrack=on;
//ArmL.setVelocity(100, velocityUnits::pct);
//ArmR.setVelocity(100, velocityUnits::pct);


while (1) 
{
///////////////////////////////////////////   Drive Train   ///////////////////////////////////////////
  if (bUp==1)     
  { 
    StopDrive(hold);        //Defend Position
  }

  else if (AutoRunning == 0 && (abs(ch3) > 1 || abs(ch4) > 1)) 
  { 
    FSpeed = 0.5*ch3 - 0.001*pow(ch3,2) + 0.00006*pow(ch3,3);
    TSpeed = 0.5*ch4 - 0.001*pow(ch4,2) + 0.00006*pow(ch4,3);
    CubeTrack=off; ToCube=off;
    run(LF, (FSpeed + TSpeed)); 
    run(LM, (FSpeed + TSpeed));
    run(LB, (FSpeed + TSpeed)); 
    run(RF, (FSpeed - TSpeed)); 
    run(RM, (FSpeed - TSpeed));
    run(RB, (FSpeed - TSpeed));
  }

  else if (AutoRunning == 0 && CubeTrack==off && ToCube==off) 
  {
    StopDrive(coast);
    FSpeed = 0;
    TSpeed = 0;
  } 
  // else if (AutoRunning == 0 && CubeTrack==off && ToCube==off)
  // {
  //   StopDrive(brake);
  // }

  else {} 


///////////////////////////////////////////   ARM   ///////////////////////////////////////////



// if (AutoRunning==0&&abs(ch2)>20&&abs(enc(ArmR)-ArmTarget)<50&&abs(enc(ArmL)-ArmTarget)<50||bRight==1)//press bRight to oeveride limits
// {
//  if (ArmTarget+(ch2/10)<813)
//  {ArmTarget=813;}
//  else if (ArmTarget+(ch2/10)>0)
//  {ArmTarget=0;}
//  else
//  {
//    ArmSpeed=abs(ch2);ArmTarget+=floor(ch2/10);}
// }


///////////////////////////////////////////   Ramp   ///////////////////////////////////////////

  if (ramp == fwrd)
  {
    if (bA && !G::rampprev)
    {
      ramp = bwrd;
      RunRamp=on;
    }
  }

  else if (ramp == bwrd)
  {
    if ( bA && !G::rampprev) 
    {
      ramp = fwrd;
      RunRamp=on;
    }
  }
  G::rampprev=bA;
  if (bX==1)
  {
    RunRamp=off;
  }

if (bL1){DontDropStack=on;DontLiftStack=on;}
else {DontDropStack=off;DontLiftStack=off;}



  if( enc(RampR)<-100 || RunRamp==on )                                                                        {}
  else if ( bY==1 && DontDropStack==0 )                                                                                 {run(RampWheelL,-100);    run(RampWheelR,-100);}
  else if ( (CubeSense.pressing() || CubeSense2.pressing()) &&  DontDropStack==0)     {run(RampWheelL,100);     run(RampWheelR,100);}
  else                                                                                                        {BRAKE(RampWheelL,hold);  BRAKE(RampWheelR,hold);}
  
  // if( enc(RampR)<-100 || RunRamp==on){} 
  if (intake == off) 
  {
    if (bB && !G::intakeprev) {intake = on;}
    run(RightRoller, 100); 
    run(LeftRoller, 100);
  } 
  else if (intake == on) 
  {
    if (bB && !G::intakeprev) {intake = off;}
    run(RightRoller, 0);
    run(LeftRoller, 0);
  }

  G::intakeprev = bB;


///////////////////////////////////////////   INTAKE   ///////////////////////////////////////////

  if (bY==1)
  {
    run(RightRoller, -100); 
    run(LeftRoller, -100);
  }

  if (bL2==1){ContinueStack=on;}
  else{ContinueStack=off;}

if (bLeft){task cubeload (CubeLoad);}
else if (bDown){cubeload.suspend();}
if (bRight)
{task autostack (AutoStack);}

  wait(20);

///////////////////////////////////////////   TRACKING   ///////////////////////////////////////////
  //if (bL2==1){CubeTrack=on;}

}