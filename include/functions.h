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
/*void run(vex::motor motorname, double speed) {
  if (speed != 0) {
    motorname.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
  } else {
    motorname.stop(vex::brakeType::brake);
  }
}*/
void runRPM(vex::motor motorname, double speed) {
  if (speed != 0) {
    motorname.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
  } else {
    motorname.stop(vex::brakeType::brake);
  }
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void StopArm(vex::brakeType brake) {
  BRAKE(ArmL, brake);
  BRAKE(ArmR, brake);
} 
void StopRamp(vex::brakeType brake) {
  BRAKE(RampL, brake);
  BRAKE(RampR, brake);
}
void StopDrive(vex::brakeType brake) {
  BRAKE(RF, brake);
  BRAKE(RM, brake);
  BRAKE(RB, brake);
  BRAKE(LF, brake);
  BRAKE(LM, brake);
  BRAKE(LB, brake);
}
////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void AutoStack()
{
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
    Brain.Screen.printAt(100, 40, "P: H:%d    Y:%d",
                         Vision.objects[FinalObject].height,
                         Vision.objects[FinalObject].centerY);
    Brain.Screen.printAt(340, 140, "MT %d ", MATCHTIMER);
    Brain.Screen.printAt(340, 160, "Ramp %d ", ramp);
    Brain.Screen.printAt(340, 180, "RampEnc %f ", enc(RampR));
    Brain.Screen.printAt(340, 200, "cube %d ", CubeSense.value(percentUnits::pct));
    Brain.Screen.printAt(340, 220, "cube %d ", CubeSense2.value(percentUnits::pct));
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
int Move(float speed, float dist, bool rampspeed, vex::brakeType X) {

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
  while (fabs(enc(LF)) < abs((dist * 360.0 / (4.0 * 3.14159))) && T3 < 4000) {

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
  StopDrive(X);
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

void SwapColor() {}

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
        while(enc(RampR)>-600&&bX==0&&ramp==-1) //MOVE UP
        {
          spd=5*pow(10,-10)*pow(enc(RampR),4)+3*pow(10,-7)*pow(enc(RampR),3)+0.0002*pow(enc(RampR),2)+0.1915*enc(RampR)+65;
          run(RampR,-spd);
          run(RampL,-spd);
          wait(10);
        }
        if (ramp==-1){RunRamp=off;}

        run(RampR,0);
        run(RampL,0); 
      }
      else 
      {
        while(enc(RampR)<-50&&bX==0&&ramp==1)
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