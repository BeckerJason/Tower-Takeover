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
int Move(float speed, float dist, bool rampspeed, vex::brakeType brake,int functiontimer) {

  float dir;
  if (dist < 0) {
    dir = -1;
  } else {
    dir = 1;
  }
  float Tdir = dir;
  LF.resetRotation();
  RF.resetRotation();
  wait(20);
  T3 = 0;
  double counter = 100;
  if (rampspeed) {
    counter = 50;
  }
  while (fabs(enc(LF)) < abs((dist * 360.0 / (4.0 * 3.14159))) && T3 < functiontimer) {

    float Roffset = 1.0;
    if (fabs(enc(LF)) < fabs((enc(RF)) + 2)) {
      Roffset = 0.9;
    } else if (fabs(enc(LF)) > fabs(enc(RF)) - 2) {
      Roffset = 1.1;
    } else {
    }
    if (counter < 100) {
      dir = Tdir * counter * 0.01;
      counter += .75;
    } else {
      dir = Tdir;
    }
    run(RF, speed * Roffset * dir);
    run(LF, speed * dir);
    run(RF, speed * Roffset * dir);
    run(LF, speed * dir);
    run(RB, speed * Roffset * dir);
    run(LB, speed * dir);
    wait(12);
  }
  StopDrive(brake);
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

int IntakeControl()
{
  while(true)
  {
    while(AutoRunning)
    {
    run(RightRoller, intake*100); run(LeftRoller, intake*100);
    wait(50);
    }
  if(enc(RampR)<-100 /*RampLimitBottom.pressing()==0*/||RunRamp==on){} 
  else if (intake == off) {if (bB && !G::intakeprev) intake = on;run(RightRoller, 100); run(LeftRoller, 100);} 
  else if (intake == on) { 

    if ( bB && !G::intakeprev) intake = off;run(RightRoller, 0); run(LeftRoller, 0);}
       G::intakeprev = bB;
       //INTAKE
    if (bY==1){run(RightRoller, -80); run(LeftRoller, -80);}
    wait(20);
  }
  return 0;
}

int RampControl() //Function to be run as a task. this controls the ramp
{RunRamp=off;
  int oldval=ramp;
  RampL.setMaxTorque(100, percentUnits::pct);
  RampR.setMaxTorque(100, percentUnits::pct);
  RampR.resetRotation();
  RampL.resetRotation();

  while(1)
  {
      
      
      while(RunRamp==0){wait(10);}
      //Ramp -1 = up
      run(RightRoller,0);
      run(LeftRoller,0);
      //if (enc(RampR)>0 /*RampLimitBottom.pressing()==1*/){RampR.resetRotation();}
      if (ramp==-1)//ramp up limit  //RAMP UP
      {
        if (bL1==0)
        {
          RampWheelL.setVelocity(25, velocityUnits::rpm);
          RampWheelR.setVelocity(25, velocityUnits::rpm);
          RampWheelL.startRotateFor(directionType::fwd, -175, rotationUnits::deg);
          RampWheelR.startRotateFor(directionType::fwd, -175, rotationUnits::deg); 
        }
        double spd=0;
        while(enc(RampR)>-500&&bX==0&&ramp==-1) //MOVE UP
        {
          spd=5*pow(10,-10)*pow(enc(RampR),4)+3*pow(10,-7)*pow(enc(RampR),3)+0.0002*pow(enc(RampR),2)+0.1915*enc(RampR)+65;
          run(RampR,-spd);
          run(RampL,-spd);
          wait(10);
        }
        if (ramp==-1){RunRamp=off;}

        BRAKE(RampR,hold);
        BRAKE(RampL,hold); 
      }
      else 
      {
        while(enc(RampR)<-20&&bX==0&&ramp==1)
        {
          run(RampR,40);
          run(RampL,40);
          wait(10);
        }
        if (ramp==1){RunRamp=off;}
        run(RampR,0);
        run(RampL,0);
      }

      run(RampL,0);run(RampR,0);
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
