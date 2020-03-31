#include "defines.h"
using namespace G;
using namespace std;
using namespace vex;
// changed
int ENDAUTOTIMER() {
  while (1) {
    wait(100);
    MATCHTIMER += 100;
  }
  return 0;
}
int GyroTrack() {
  float GyroAdd = 0;
  float GyroTCheck = 0;

  while (1) {
    GyroAdd = Gyro.value(vex::rotationUnits::raw) - GyroTCheck;
    GyroTCheck = Gyro.value(vex::rotationUnits::raw);
    GlobalGyro += GyroAdd;
    wait(5);
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/*float enc(vex::motor motorname) {
  return motorname.rotation(vex::rotationUnits::deg);
}*/
int PrintScreen() {
  while (1) {
    Brain.Screen.clearScreen(vex::color::black);
    Brain.Screen.setPenColor(vex::color::white);
    Brain.Screen.printAt(1, 60, "%f", Gyro.value(vex::rotationUnits::raw));
    Brain.Screen.printAt(340, 20, "ARM: %1.1f", ArmR.velocity(percentUnits::pct));
    Brain.Screen.printAt(100, 40, "P: H:%d    Y:%d", Vision.objects[FinalObject].height, Vision.objects[FinalObject].centerY);
    Brain.Screen.printAt(100, 80, "green %X ", GreenCube.centerX[0]);
    Brain.Screen.printAt(340, 140, "MT %d ", MATCHTIMER);
    Brain.Screen.printAt(340, 160, "Ramp %d ", ramp);
    Brain.Screen.printAt(340, 180, "RampEnc %f ", enc(RampR));
    Brain.Screen.printAt(340, 200, "cube %d ", CubeSense.pressing());
    Brain.Screen.printAt(340, 220, "cube %d ", CubeSense2.pressing());
    wait(75);
  }
  return 0;
}
int PrintController()
{
  while(1)
  {
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1,1);
  Controller.Screen.print(RampL.velocity(percentUnits::pct) /*enc(RampR)*/);
   Controller.Screen.setCursor(2,1);
   Controller.Screen.print(RunRamp) /*enc(RampR)*/;
  wait(100);
  }

  return 0;
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
int TIMER2() {
  while (1) {
    wait(1);
    T3 += 1;

    if(!bR1&&!bR2) T4 += 1;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void rightDrive(int power) {
  run(RF, power);
  run(RM, power);
  run(RB, power);
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void leftDrive(int power) {
  run(LF, power);
  run(LM, power);
  run(LB, power);
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////TURN BASED ON GYRO DEGREES EXAMPLE // TurnDegree(-90, 100,1000);//THIS TURNS LEFT 90 DEGREES AT 100 POWER
void Turn(double degrees, double speed, int TimeOut)//, int Timeout)
{
  double dir=1;
	GlobalGyro = 0;
	wait(50);
	if (degrees > 0){dir=-1;}
	else if (degrees < 0){dir=1;}
	//float ticks = abs(degrees*7.7);
	T3=0;
	LM.resetRotation();
	RM.resetRotation();
  double turnspeed=speed;
  degrees*=10;
	while(fabs(GlobalGyro) < fabs(degrees))
	{
    if(fabs(fabs(GlobalGyro)-fabs(degrees))<10)
    {
    turnspeed=speed*0.5;
    }
		////////////////////////////////////FAILSAFE TIMEOUT
		if(T3 > TimeOut  && TimeOut > 0){break;}
		else if(abs(enc(LM))>abs(enc(RM))+3)///////
		{run(LM,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
			run(LF,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
			run(LB,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
			run(RM,-dir*turnspeed);
			run(RB,-dir*turnspeed);
			run(RF,-dir*turnspeed);}
		else if(abs(enc(LM))<abs(enc(RM))-3)///////
		{run(LM,turnspeed*dir);
			run(LF,turnspeed*dir);
			run(LB,turnspeed*dir);
			run(RM,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));
			run(RB,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));
			run(RF,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));}
		else{
			run(LM,turnspeed*dir);
			run(LF,turnspeed*dir);
			run(LB,turnspeed*dir);
			run(RM,-dir*turnspeed);
			run(RB,-dir*turnspeed);
			run(RF,-dir*turnspeed);}
      wait(10);
	}
  StopDrive(hold);
}
////////////////////////////////////////////////////////////////////////////
int MoveCounter = 0;
int PID_MOTOR_SCALE = 1;
int PID_MOTOR_MAX = 100;
int PID_MOTOR_MIN = (-100);
int PID_INTEGRAL_LIMIT = 40;

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void pidTurn(float globalDegrees, float pid_Kp, float pid_Ki, float pid_Kd,
             int timeout) {
  LF.resetRotation();
  RF.resetRotation();
  int LASTDIR = 1;
  float RO = 1;
  int direction = 0;
  GlobalGyro = 0;
  Brain.resetTimer();
  float pidError = 0;
  float pidLastError = 0;
  float pidIntegral = 0;
  float pidDerivative = 0;
  float pidDriveR = 0;

  T3 = 0;
  while (T3 < timeout) {
    if (fabs(enc(RF)) - 5 > fabs(enc(LF))) {
      RO = .8;
    } else if (fabs(enc(LF)) - 5 > fabs(enc(RF))) {
      RO = 1.2;
    } else {
      RO = 1;
    }
    // Calculate Error//(Convert Dintance in inches to encoder ticks)
    pidError = fabs(globalDegrees * 10) - fabs(GlobalGyro);

    // integral - if Ki is not 0(can put threshold)
    if (pid_Ki != 0) {
      // If we are inside controlable window then integrate the error
      if (fabs(pidError) < PID_INTEGRAL_LIMIT)
        pidIntegral = pidIntegral + pidError;
      else
        pidIntegral = 0;
    } else
      pidIntegral = 0;
    ///////////////////////////////////////
    // CALCULATE DERIVATIVE/////////////////
    pidDerivative = pidError - pidLastError;
    pidLastError = pidError;
    //////////////////////////////////////
    // CALCULATE DRIVE/////////////////////
    pidDriveR =
        (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);
    ///////////////////////////////////////
    // LIMIT DRIVE//////////////////////////
    if (pidDriveR > PID_MOTOR_MAX)
      pidDriveR = PID_MOTOR_MAX;
    if (pidDriveR < PID_MOTOR_MIN)
      pidDriveR = PID_MOTOR_MIN;
    ///////////////////////////////////////
    // SEND POWER TO MOTORS/////////////////
    if (globalDegrees > 0)
      direction = 1;
    if (globalDegrees < 0)
      direction = -1;
    // if(gmoveandturn==false)
    rightDrive(direction * pidDriveR * PID_MOTOR_SCALE * RO);
    leftDrive(-direction * pidDriveR * PID_MOTOR_SCALE);
    if ((fabs(pidError) < 10) && (fabs(avgSpeed) < 10)) {
      StopDrive(brake);
      break;
    }
    if (Brain.timer(vex::timeUnits::msec) < 40) {
      avgError += pidError;
    } else {
      avgSpeed = avgError / 3;
      avgError = 0;
      Brain.resetTimer();
    }
    LASTDIR = direction;
    // REFRESH RATE 60Hz
    // while(fmod(Brain.timer(vex::timeUnits::msec),16)>2){wait(1);}
    wait(16);
  }
  StopDrive(brake);
  wait(300);
  leftDrive(LASTDIR * 2);
  rightDrive(-LASTDIR * 2);
  wait(300);

  StopDrive(brake);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
int move(float speed, float dist, bool rampspeed,int functiontimer) {
  dist=dist*.95;
  float dir;
  if (dist < 0) {
    dir = -1;
  } else {
    dir = 1;
  }
  float Tdir = dir;
  LM.resetRotation();
  RM.resetRotation();
  wait(20);
  T3 = 0;
  double counter = 100;
  if (rampspeed) {
    counter = 50;
  }
while (fabs(enc(LM)) < fabs((dist * 360.0 / (4.0 * 3.14159))) && T3 < functiontimer&&!bX) {

    float Roffset = 1.0;
    if (fabs(enc(LM)) < fabs((enc(RM)) + 2)) {
      Roffset = 0.925;
    } else if (fabs(enc(LM)) > fabs(enc(RM)) - 2) {
      Roffset = 1.075;
    } else {
    }
    if (counter < 100) {
      dir = Tdir * counter * 0.01;
      counter += .75;
    } else {
      dir = Tdir;
    }
    run(RM, speed * Roffset * dir);
    run(LM, speed * dir);
    run(RF, speed * Roffset * dir);
    run(LF, speed * dir);
    run(RB, speed * Roffset * dir);
    run(LB, speed * dir);
    wait(12);
  }
  return 1;

}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/*void ToWall(double vel) {
  T3 = 0;
  float TLF = 0, TRF = 0;
  SetDriveTorque(30);
  LF.resetRotation();
  RF.resetRotation();
  rightDrive(vel);
  leftDrive(vel);
  wait(300);
  do {
    TLF = enc(LF);
    TRF = enc(RF);
    rightDrive(vel);
    leftDrive(vel);
    wait(100);
  } while ((TLF != enc(LF) || TRF != enc(RF)) &&(RLine.value(vex::percentUnits::pct) < 65 ||LLine.value(vex::percentUnits::pct) < 65) &&T3 < 2500);
  StopDrive(coast);
  SetDriveTorque(100);
}*/
////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /*void Tile(double fact, int timeout, bool intile, vex::brakeType X) {
  T3 = 0;
  if (intile == 1) {
    while (T3 < timeout && (RLine.value(vex::percentUnits::pct) < TILE ||
                            RLine.value(vex::percentUnits::pct) < TILE)) {
      if (RLine.value(vex::percentUnits::pct) - TILE < 0 &&
          LLine.value(vex::percentUnits::pct) - TILE < 0) {
        rightDrive(-fact * (RLine.value(vex::percentUnits::pct) - TILE));
        leftDrive(-fact * (RLine.value(vex::percentUnits::pct) - TILE));
      } else {
        rightDrive(-1 * fact * (RLine.value(vex::percentUnits::pct) - TILE));
        leftDrive(-1 * fact * (LLine.value(vex::percentUnits::pct) - TILE));
      }
      wait(10);
    }
  } else {
    while (T3 < timeout && (RLine.value(vex::percentUnits::pct) > TILE ||
                            RLine.value(vex::percentUnits::pct) > TILE)) {
      if (RLine.value(vex::percentUnits::pct) - TILE > 0 &&
          LLine.value(vex::percentUnits::pct) - TILE > 0) {
        rightDrive(fact * (RLine.value(vex::percentUnits::pct) - TILE));
        leftDrive(fact * (RLine.value(vex::percentUnits::pct) - TILE));
      } else {
        rightDrive(fact * (RLine.value(vex::percentUnits::pct) - TILE));
        leftDrive(fact * (LLine.value(vex::percentUnits::pct) - TILE));
      }
      wait(10);
    }
  }
  StopDrive(X);
}*/

int AutoStack()
{
/*  AutoRunning=1;
if(CubeSense2.pressing()==1){DontDropStack=on;}
ramp=fwrd;
RunRamp=on;
while(RunRamp==on){ContinueStack=on; wait(500); ContinueStack=off;wait(500);}
arm.suspend();
ArmL.setVelocity(20,vex::velocityUnits::pct);
ArmR.setVelocity(20,vex::velocityUnits::pct);
if (!bX)
{
ArmL.startRotateTo(90,rotationUnits::deg);
ArmR.startRotateTo(90,rotationUnits::deg);
}
ramp=bwrd;
Move(15,-5.8,1,brake,10000);
RunRamp=on;
Move(30,-6.8,1,brake,10000);
arm.resume();
AutoRunning=0;*/
return 0;
}


int CubeLoad()
{
  intake=off;ControlIntake=on;
  IntakeController.suspend();
  rampwheel.suspend();
  //arm.suspend();
  //ArmL.setVelocity(50, velocityUnits::rpm);
        //ArmR.setVelocity(50, velocityUnits::rpm);
  //ArmL.startRotateTo(0, rotationUnits::deg);
  //ArmR.rotateTo(0, rotationUnits::deg);
  RampWheelL.setVelocity(100, velocityUnits::rpm);
        RampWheelR.setVelocity(100, velocityUnits::rpm);
          LeftRoller.setVelocity(100, velocityUnits::rpm);
        RightRoller.setVelocity(100, velocityUnits::rpm);
        RampWheelL.startRotateFor(directionType::fwd, -265, rotationUnits::deg);
        RampWheelR.rotateFor(directionType::fwd, -265, rotationUnits::deg); 
        RightRoller.startRotateFor(directionType::fwd, -30, rotationUnits::deg);
        LeftRoller.rotateFor(directionType::fwd, -30, rotationUnits::deg); 
        //arm.resume();
      IntakeController.resume();
  rampwheel.resume();

return 0;
}
int IntakeControl()
{
  while(true)
  {
    if(AutoRunning)
    { 
   if (intake == manual){run(RightRoller, ManualSpeed); run(LeftRoller, ManualSpeed);}
  else if (intake == off) {run(RightRoller,0); run(LeftRoller, 0);} 
  else if (intake == on){run(RightRoller, 80); run(LeftRoller, 80);}
    }
    else
    {
  if(enc(RampR)<-100 /*RampLimitBottom.pressing()==0*/||RunRamp==on){} 
  else if (intake == manual){run(RightRoller, ManualSpeed); run(LeftRoller, ManualSpeed);}
  else if (intake == off) {if ((bB||ch2) && !G::intakeprev) intake = on;run(RightRoller, 100); run(LeftRoller, 100);} 
  else if (intake == on) { 
    if ( (bB && !G::intakeprev)) intake = off;run(RightRoller, 0); run(LeftRoller, 0);}
       G::intakeprev = bB||ch2;
    
       //INTAKE
    if (bY==1){run(RightRoller, -80); run(LeftRoller, -80);}
    }
    wait(50);
  }
  return 0;
}

///////////////////////////////////////////
////////////////////////////////////////////////////
int RampWheels()
{
  while(1)
  {
     if(enc(RampR)<-100 /*RampLimitBottom.pressing()==0*/||RunRamp==on){}
  else if (bY==1&&DontLiftStack==0){run(RampWheelL,-100);run(RampWheelR,-100);}
  else if ( (CubeSense.pressing() || CubeSense2.pressing()) &&  DontLiftStack==0){run(RampWheelL,80);run(RampWheelR,80);}
  else {BRAKE(RampWheelL,hold);BRAKE(RampWheelR,hold);}
wait(50);

  }


  return 0;
}

int ArmControl()
{
  double Roffset=1;
  ArmL.setVelocity(100, velocityUnits::pct);
  ArmR.setVelocity(100, velocityUnits::pct);
  ArmL.resetRotation();
  ArmR.resetRotation(); 
  while(1)
  {
    if (AutoRunning==0)
    {
      if (ch2>1&&enc(ArmL)<500)
      {
        if (enc(ArmR)>enc(ArmL)+2){Roffset=.95;}
        else if (enc(ArmR)<enc(ArmL)-2){Roffset=1.05;}
        else{Roffset=1;}
        run(ArmR, ch2*Roffset/3);
        run(ArmL, ch2/3);
        T4=0;
      }
      else if (ch2<-1&&enc(ArmL)>0)
      {
        if (enc(ArmR)<enc(ArmL)-2){Roffset=.95;}
        else if (enc(ArmR)>enc(ArmL)+2){Roffset=1.05;}
        else{Roffset=1;}
        run(ArmR, ch2*Roffset/3);
        run(ArmL, ch2/3);
        T4=0;
      }
      else
      {
        if (T4>50)
        {
          BRAKE(ArmL,hold);
          ArmR.rotateTo(enc(ArmL), rotationUnits::deg);
        }
        else
        {
          BRAKE(ArmL, hold);
          BRAKE(ArmR, hold);
        }
      }
    }
    else{}
  }
  return 0;
}

int RampControl() //Function to be run as a task. this controls the ramp
{
  RunRamp=off;
  int oldval=ramp;
  RampL.setMaxTorque(100, percentUnits::pct);
  RampR.setMaxTorque(100, percentUnits::pct);
  RampLb.setMaxTorque(100, percentUnits::pct);
  RampRb.setMaxTorque(100, percentUnits::pct);
  RampR.resetRotation();
  RampL.resetRotation();
  RampRb.resetRotation();
  RampLb.resetRotation();
  while(1)
  {
    while(RunRamp==0){wait(10);}
    if (ramp==-1)//ramp up limit  //RAMP UP
    {
      if (DontDropStack==0)
      {
        RampWheelL.setVelocity(25, velocityUnits::rpm);
        RampWheelR.setVelocity(25, velocityUnits::rpm);
        RampWheelL.startRotateFor(directionType::fwd, -175, rotationUnits::deg);
        RampWheelR.startRotateFor(directionType::fwd, -175, rotationUnits::deg); 
      }
      double spd=0;
      while(enc(RampR)>-290&&ramp==-1) //MOVE UP
      {
        spd = 100 - 1.501881*enc(-1*RampR) + 0.008061975*pow(enc(-1*RampR),2) - 0.00001388072*pow(enc(-1*RampR),3);
        if(spd<=15){spd=10;}
        G::spd=spd;
        run(RampR,-spd);
        run(RampL,-spd);
        run(RampRb,-spd);
        run(RampLb,-spd);        
        wait(10);
        while(!ContinueStack){StopRamp(hold);wait(20);}
      }
      G::spd=0;
      if (ramp==-1)
      {
        RunRamp=off;
        Piston.set(0);
      }//open
      BRAKE(RampR,hold);
      BRAKE(RampL,hold);
      BRAKE(RampRb,hold);
      BRAKE(RampLb,hold); 
    }
    else 
    {
      while(enc(RampR)<-20&&ramp==1)
      {
        run(RampR,40);
        run(RampL,40);
        run(RampRb,40);
        run(RampLb,40);
        wait(10);
        G::spd=140;
        //while(!bX){StopRamp(hold);wait(20);}
      }
      if (ramp==1){RunRamp=off;Piston.set(on);}
      BRAKE(RampR,coast);
      BRAKE(RampL,coast);
      BRAKE(RampRb,coast);
      BRAKE(RampLb,coast); 
    }

    run(RampL,0);run(RampR,0);
    run(RampLb,0);run(RampRb,0);
  }
  return 0;
}

//O G P
void Colors(ToggleMode O, ToggleMode Gr,ToggleMode P)//sets colors for tracking
{
OTrack=O;
GTrack=Gr;
PTrack=P;
}

int TurnToCube()
{
  int offCounter=0;
  while (1)
  {
    if (CubeTrack==on||ToCube==on)
    {
        Vision.takeSnapshot(1,TOTALSNAPSHOTS);//take picture of orange cubes
        if (OTrack==off){OrangeCube.exists[0]=0;}
        else
        {
          for (int i=0;i<TOTALSNAPSHOTS;i++)
          {
            OrangeCube.id[i]=Vision.objects[i].id;
            OrangeCube.originX[i]=Vision.objects[i].originX;
            OrangeCube.originY[i]=Vision.objects[i].originY;
            OrangeCube.centerX[i]=Vision.objects[i].centerX;
            OrangeCube.centerY[i]=Vision.objects[i].centerY;
            OrangeCube.width[i]=Vision.objects[i].width;
            OrangeCube.height[i]=Vision.objects[i].height;
            OrangeCube.angle[i]=Vision.objects[i].angle;
            OrangeCube.exists[i]=Vision.objects[i].exists;
          }
        }
        if (PTrack==off){PurpleCube.exists[0]=0;}
        else
        {
          Vision.takeSnapshot(2,TOTALSNAPSHOTS);//take picture of purple cubes
          for (int i=0;i<TOTALSNAPSHOTS;i++)
          {
            PurpleCube.id[i]=Vision.objects[i].id;
            PurpleCube.originX[i]=Vision.objects[i].originX;
            PurpleCube.originY[i]=Vision.objects[i].originY;
            PurpleCube.centerX[i]=Vision.objects[i].centerX;
            PurpleCube.centerY[i]=Vision.objects[i].centerY;
            PurpleCube.width[i]=Vision.objects[i].width;
            PurpleCube.height[i]=Vision.objects[i].height;
            PurpleCube.angle[i]=Vision.objects[i].angle;
            PurpleCube.exists[i]=Vision.objects[i].exists;
          }
        }
        if(GTrack==off){GreenCube.exists[0]=0;}
        else
        {
          Vision.takeSnapshot(3,TOTALSNAPSHOTS);//take picture of green cubes
          for (int i=0;i<TOTALSNAPSHOTS;i++)
          {
            GreenCube.id[i]=Vision.objects[i].id;
            GreenCube.originX[i]=Vision.objects[i].originX;
            GreenCube.originY[i]=Vision.objects[i].originY;
            GreenCube.centerX[i]=Vision.objects[i].centerX;
            GreenCube.centerY[i]=Vision.objects[i].centerY;
            GreenCube.width[i]=Vision.objects[i].width;
            GreenCube.height[i]=Vision.objects[i].height;
            GreenCube.angle[i]=Vision.objects[i].angle;
            GreenCube.exists[i]=Vision.objects[i].exists;
          }
        }
      THeight=0;
      TWidth=0;
      TurnDiff=0;
      for (int i=0;OrangeCube.exists[i];i++)
      {
        if(OrangeCube.height[i]>THeight)
        { 
          THeight=OrangeCube.height[i];
          TWidth=OrangeCube.width[i];
          TurnDiff=GlobalCubeOffset-OrangeCube.centerX[i];
        }
      }
      for (int i=0;PurpleCube.exists[i];i++)
      {
        if(PurpleCube.height[i]>THeight)
        {
          THeight=PurpleCube.height[i];
          TWidth=PurpleCube.width[i];
          TurnDiff=GlobalCubeOffset-PurpleCube.centerX[i];
        }
      }
      for (int i=0;GreenCube.exists[i];i++)
      {
        if(GreenCube.height[i]*GreenCube.width[i]>THeight*TWidth)
        {
          THeight=GreenCube.height[i];
          TWidth=GreenCube.width[i];
          TurnDiff=GlobalCubeOffset-GreenCube.centerX[i];
        }
      }

      if(TurnDiff<-15)//right
      {TurnDir=-1;}
      else if (TurnDiff>15)//left
      {TurnDir=1;}
      if(abs(TurnDiff)<50)
      {CubeTrack=off;ToCube=on;} //Stop Turning if within a range
      if (!GreenCube.exists[0]&&!PurpleCube.exists[0]&&!OrangeCube.exists[0])
      {offCounter++;}
      else if (offCounter>0)
      {offCounter--;}
      if (offCounter==5)
      {CubeTrack=off;ToCube=off;offCounter=0;}
      

      if(CubeTrack==on)   //if cubetrack==on
      {
      leftDrive((0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*-TurnDir); //Run Motors
      rightDrive((0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*TurnDir);
      }
      else if (ToCube==on)
      {
        if (fabs(TurnDiff>15))
        {
        leftDrive(40+(0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*-TurnDir);
        rightDrive(40+(0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*TurnDir);
        }
        else
        {
          leftDrive(40);
          rightDrive(40);
        }
        if(CubeSense.pressing() || CubeSense2.pressing())
        {ToCube=off;CubeTrack=off;StopDrive(hold); wait(200);}
        wait(10);
      }
      else{StopDrive(hold);}
    }

    wait(15);
  }
  return 0;  
}
