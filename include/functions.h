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
  float CurrentGyro=0;
  //std::ofstream file;
  //int counter=0;
    wait(3000);                             //wait (X) ms
  Gyro.startCalibration();                //start Gyro Calibration
  wait(2000);                             //wait (X) ms
  while (1) {
    CurrentGyro=Gyro.value(vex::rotationUnits::raw);
    //if going from 3600 to 0
    if(GyroTCheck>2500&&GyroTCheck<=3600&& CurrentGyro<1000)
    {GyroAdd=3600-GyroTCheck+ CurrentGyro;}
    //if going gtom -3600 to 0
    else if(GyroTCheck<-2500&&GyroTCheck>-3600&& CurrentGyro>-1000)
  {GyroAdd=-3600-GyroTCheck+ CurrentGyro;}
  //if going from 0 to 3600
  else if (GyroTCheck>-1000&&GyroTCheck<1000&&CurrentGyro>2500)
  {GyroAdd=-GyroTCheck- 3600-CurrentGyro;}
//going from 0 to -3600
else if (GyroTCheck<1000&&GyroTCheck>-1000&&CurrentGyro<-2500)
  {GyroAdd=-GyroTCheck-3600-CurrentGyro;}

  else {GyroAdd = GyroTCheck-CurrentGyro;}
    GyroTCheck = CurrentGyro;
    if (GyroAdd>0)
    {GlobalGyro += GyroAdd*0.96566;GlobalGyroT += GyroAdd*0.96566;}
    else{GlobalGyro+=GyroAdd*0.97826;GlobalGyroT+=GyroAdd*0.97826;}
        
  /*file.open("DATA.csv",ios::out | ios::app | ios::ate );
  file<<counter<<","<< Gyro.value(vex::rotationUnits::raw)<<","<<GlobalGyro<<endl;
  file.close();
      counter++;*/
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
    Brain.Screen.printAt(1, 60, "%f", GlobalGyro);
    Brain.Screen.printAt(340, 20, "ARM: %1.1f", ArmR.velocity(percentUnits::pct));
    Brain.Screen.printAt(100, 40, "P: H:%d    Y:%d", Vision.objects[FinalObject].height, Vision.objects[FinalObject].centerY);
    Brain.Screen.printAt(100, 80, "move %f ", enc(LM));
    Brain.Screen.printAt(340, 140, "MT %d ", MATCHTIMER);
    Brain.Screen.printAt(340, 160, "Ramp %d ", ramp);
    Brain.Screen.printAt(340, 180, "RampEnc %f ", enc(RampR));
    Brain.Screen.printAt(340, 200, "green %f ", GreenCube.centerX[0]);
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
    Controller.Screen.print("/");
    Controller.Screen.print(G::spd);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print(RunRamp) /*enc(RampR)*/;
    Controller.Screen.setCursor(3,1);
    Controller.Screen.print(enc(RampR));
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
// void T(double degrees, double speed, int TimeOut)//, int Timeout) similar to turn, suing start angle as 0 always
// {
//   double dir=1;
//   degrees*=10;

// 	//float ticks = abs(degrees*7.7);
// 	T3=0;
// 	LM.resetRotation();
// 	RM.resetRotation();
//   double turnspeed;
//   double TGyro;
//   double count;
//   for(int i=0;i<1;i++)
//   {	
//     count=.10;
//   if (degrees-GlobalGyroT > 1){dir=-1;}
// 	else if (degrees-GlobalGyroT < -1){dir=1;}
//   else{return;}
//   turnspeed=speed;
//   TGyro=GlobalGyroT;
// 	while(fabs(GlobalGyroT-TGyro) < fabs(degrees-TGyro))
// 	{
//     turnspeed=speed;
//     if(fabs(fabs(GlobalGyroT)-fabs(degrees))<1000)//<100 deg
//     {float val=fabs(fabs(GlobalGyroT)-fabs(degrees));
//     turnspeed=speed*(1*pow(10,-9)*pow(val,3)-1*pow(10,-6)*pow(val,2)+0.0008*val+0.2447);
//     }
//     turnspeed*=count;

// 		////////////////////////////////////FAILSAFE TIMEOUT
// 		if(T3 > TimeOut  && TimeOut > 0){StopDrive(hold);return;}
// 		else if(abs(enc(LM))>abs(enc(RM))+3)///////
// 		{run(LM,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
// 			run(LF,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
// 			run(LB,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
// 			run(RM,-dir*turnspeed);
// 			run(RB,-dir*turnspeed);
// 			run(RF,-dir*turnspeed);}
// 		else if(abs(enc(LM))<abs(enc(RM))-3)///////
// 		{run(LM,turnspeed*dir);
// 			run(LF,turnspeed*dir);
// 			run(LB,turnspeed*dir);
// 			run(RM,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));
// 			run(RB,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));
// 			run(RF,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));}
// 		else{
// 			run(LM,turnspeed*dir);
// 			run(LF,turnspeed*dir);
// 			run(LB,turnspeed*dir);
// 			run(RM,-dir*turnspeed);
// 			run(RB,-dir*turnspeed);
// 			run(RF,-dir*turnspeed);}
//       count+=0.01;
//       wait(10);
// 	}
//   StopDrive(hold);
//   speed*=0.5;
//   //wait(200);
// }
//   StopDrive(hold);
// }
// void Turn(double degrees, double speed, int TimeOut)//, int Timeout)
// {
//   double dir=1;
// 	GlobalGyro = 0;
// 	wait(50);
// 	if (degrees > 0){dir=-1;}
// 	else if (degrees < 0){dir=1;}
// 	//float ticks = abs(degrees*7.7);
// 	T3=0;
// 	LM.resetRotation();
// 	RM.resetRotation();
//   double turnspeed=speed;
//   degrees*=10;
// 	while(fabs(GlobalGyro) < fabs(degrees))
// 	{
//     if(fabs(fabs(GlobalGyro)-fabs(degrees))<10)
//     {
//     turnspeed=speed*0.5;
//     }
// 		////////////////////////////////////FAILSAFE TIMEOUT
// 		if(T3 > TimeOut  && TimeOut > 0){break;}
// 		else if(abs(enc(LM))>abs(enc(RM))+3)///////
// 		{run(LM,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
// 			run(LF,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
// 			run(LB,dir*(turnspeed*(abs(enc(RM))/abs(enc(LM)))));
// 			run(RM,-dir*turnspeed);
// 			run(RB,-dir*turnspeed);
// 			run(RF,-dir*turnspeed);}
// 		else if(abs(enc(LM))<abs(enc(RM))-3)///////
// 		{run(LM,turnspeed*dir);
// 			run(LF,turnspeed*dir);
// 			run(LB,turnspeed*dir);
// 			run(RM,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));
// 			run(RB,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));
// 			run(RF,-dir*(turnspeed*(abs(enc(LM))/abs(enc(RM)))));}
// 		else{
// 			run(LM,turnspeed*dir);
// 			run(LF,turnspeed*dir);
// 			run(LB,turnspeed*dir);
// 			run(RM,-dir*turnspeed);
// 			run(RB,-dir*turnspeed);
// 			run(RF,-dir*turnspeed);}
//       wait(10);
// 	}
//   StopDrive(hold);
// }
// ////////////////////////////////////////////////////////////////////////////
// int MoveCounter = 0;
// int PID_MOTOR_SCALE = 1;
// int PID_MOTOR_MAX = 100;
// int PID_MOTOR_MIN = (-100);
// int PID_INTEGRAL_LIMIT = 40;

// ////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////

// void pidTurn(float globalDegrees, float pid_Kp, float pid_Ki, float pid_Kd,
//              int timeout) {
//   LF.resetRotation();
//   RF.resetRotation();
//   int LASTDIR = 1;
//   float RO = 1;
//   int direction = 0;
//   GlobalGyro = 0;
//   Brain.resetTimer();
//   float pidError = 0;
//   float pidLastError = 0;
//   float pidIntegral = 0;
//   float pidDerivative = 0;
//   float pidDriveR = 0;

//   T3 = 0;
//   while (T3 < timeout) {
//     if (fabs(enc(RF)) - 5 > fabs(enc(LF))) {
//       RO = .8;
//     } else if (fabs(enc(LF)) - 5 > fabs(enc(RF))) {
//       RO = 1.2;
//     } else {
//       RO = 1;
//     }
//     // Calculate Error//(Convert Dintance in inches to encoder ticks)
//     pidError = fabs(globalDegrees * 10) - fabs(GlobalGyro);

//     // integral - if Ki is not 0(can put threshold)
//     if (pid_Ki != 0) {
//       // If we are inside controlable window then integrate the error
//       if (fabs(pidError) < PID_INTEGRAL_LIMIT)
//         pidIntegral = pidIntegral + pidError;
//       else
//         pidIntegral = 0;
//     } else
//       pidIntegral = 0;
//     ///////////////////////////////////////
//     // CALCULATE DERIVATIVE/////////////////
//     pidDerivative = pidError - pidLastError;
//     pidLastError = pidError;
//     //////////////////////////////////////
//     // CALCULATE DRIVE/////////////////////
//     pidDriveR =
//         (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);
//     ///////////////////////////////////////
//     // LIMIT DRIVE//////////////////////////
//     if (pidDriveR > PID_MOTOR_MAX)
//       pidDriveR = PID_MOTOR_MAX;
//     if (pidDriveR < PID_MOTOR_MIN)
//       pidDriveR = PID_MOTOR_MIN;
//     ///////////////////////////////////////
//     // SEND POWER TO MOTORS/////////////////
//     if (globalDegrees > 0)
//       direction = 1;
//     if (globalDegrees < 0)
//       direction = -1;
//     // if(gmoveandturn==false)
//     rightDrive(direction * pidDriveR * PID_MOTOR_SCALE * RO);
//     leftDrive(-direction * pidDriveR * PID_MOTOR_SCALE);
//     if ((fabs(pidError) < 10) && (fabs(avgSpeed) < 10)) {
//       StopDrive(brake);
//       break;
//     }
//     if (Brain.timer(vex::timeUnits::msec) < 40) {
//       avgError += pidError;
//     } else {
//       avgSpeed = avgError / 3;
//       avgError = 0;
//       Brain.resetTimer();
//     }
//     LASTDIR = direction;
//     // REFRESH RATE 60Hz
//     // while(fmod(Brain.timer(vex::timeUnits::msec),16)>2){wait(1);}
//     wait(16);
//   }
//   StopDrive(brake);
//   wait(300);
//   leftDrive(LASTDIR * 2);
//   rightDrive(-LASTDIR * 2);
//   wait(300);

//   StopDrive(brake);
// }

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
  double val=1;
	while(fabs(GlobalGyro) < fabs(degrees))
	{
    val=fabs((GlobalGyro)/(degrees));
    //turnspeed=speed*(7.19*pow(val,4)-14.388*pow(val,3)+5.659*pow(val,2)+1.5349*val+0.1419);
    turnspeed=speed*(13.721896*pow(val,4) -27.443791*pow(val,3) +14.160677*pow(val,2) -0.438782*val +0.3568);
    if(13.721896*pow(val,4) -27.443791*pow(val,3) +14.160677*pow(val,2) -0.438782*val +0.2568>.7)
    {turnspeed=speed*.7;}

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
void T(double degrees, double speed, int TimeOut)//, int Timeout) similar to turn, suing start angle as 0 always
{
  double dir=1;
  degrees*=10;

	//float ticks = abs(degrees*7.7);
	T3=0;
	LM.resetRotation();
	RM.resetRotation();
  double turnspeed;
  double TGyro;
  for(int i=0;i<1;i++)
  {	
  if (degrees-GlobalGyroT > 1){dir=-1;}
	else if (degrees-GlobalGyroT < -1){dir=1;}
  else{return;}
  turnspeed=speed;
  TGyro=GlobalGyroT;
  float val=1;
	while(fabs(GlobalGyroT-TGyro) < fabs(degrees-TGyro))
	{
    val=fabs((GlobalGyroT-degrees)/(TGyro-degrees));
    //turnspeed=speed*(7.19*pow(val,4)-14.388*pow(val,3)+5.659*pow(val,2)+1.5349*val+0.1419);
    turnspeed=speed*(13.721896*pow(val,4) -27.443791*pow(val,3) +14.160677*pow(val,2) -0.438782*val +0.3568);
    if(13.721896*pow(val,4) -27.443791*pow(val,3) +14.160677*pow(val,2) -0.438782*val +0.2568>.7)
    {turnspeed=speed*.7;}
    

		////////////////////////////////////FAILSAFE TIMEOUT
		if(T3 > TimeOut  && TimeOut > 0){StopDrive(hold);return;}
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
  speed*=0.5;
  //wait(200);
}
  StopDrive(hold);
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
    wait(20);
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
  AutoRunning=1;
if(CubeSense2.pressing()==1){DontDropStack=on;}
ramp=fwrd;
RunRamp=on;
while(RunRamp==on){ContinueStack=on; wait(800); ContinueStack=off;wait(150);}
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
AutoRunning=0;
return 0;
}


int CubeLoad()
{

	intake=off;ControlIntake=on;
	IntakeController.suspend();
	rampwheel.suspend();
	arm.suspend();
	ArmL.setVelocity(90, velocityUnits::rpm);
	ArmR.setVelocity(90, velocityUnits::rpm);
	ArmL.startRotateTo(0, rotationUnits::deg);
	ArmR.rotateTo(0, rotationUnits::deg);
	run(LeftRoller,0);
	run(RightRoller,0);
	RampWheelL.setVelocity(60, velocityUnits::pct);
	RampWheelR.setVelocity(60, velocityUnits::pct);

	RampWheelL.startRotateFor(directionType::fwd, -265, rotationUnits::deg);
	RampWheelR.rotateFor(directionType::fwd, -265, rotationUnits::deg);
  rampwheel.resume();
	LeftRoller.setVelocity(100, velocityUnits::rpm);
	RightRoller.setVelocity(100, velocityUnits::rpm);
	RightRoller.startRotateFor(directionType::fwd, -10, rotationUnits::deg);
	LeftRoller.rotateFor(directionType::fwd, -10, rotationUnits::deg);
  wait(100);
  	ArmL.setVelocity(90, velocityUnits::rpm);
	ArmR.setVelocity(90, velocityUnits::rpm);
  	T3=0;
  ArmR.startRotateTo(400,rotationUnits::deg);
	ArmL.rotateTo(400,rotationUnits::deg);
  StopArm(hold);
	arm.resume();
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
      if (ch2>1&&enc(ArmL)<580)
      {
        if (enc(ArmR)>enc(ArmL)+2){Roffset=.9;}
        else if (enc(ArmR)<enc(ArmL)-2){Roffset=1.1;}
        else{Roffset=1;}
        run(ArmR, ch2*Roffset);
        run(ArmL, ch2);
        T4=0;
      }
      else if (ch2<-1&&enc(ArmL)>0)
      {
        if (enc(ArmR)<enc(ArmL)-2){Roffset=.9;}
        else if (enc(ArmR)>enc(ArmL)+2){Roffset=1.1;}
        else{Roffset=1;}
        run(ArmR, ch2*Roffset);
        run(ArmL, ch2);
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
      while(enc(RampR)>-300&&ramp==-1) //MOVE UP
      {
        spd = 100 - 1.501881*enc(-1*RampR) + 0.008061975*pow(enc(-1*RampR),2) - 0.00001388072*pow(enc(-1*RampR),3);
        //spd=5*pow(10,-10)*pow(enc(RampR),4)+3*pow(10,-7)*pow(enc(RampR),3)+0.0002*pow(enc(RampR),2)+0.1915*enc(RampR)+40;
        
        if (enc(-1*RampR)>=270) {spd=5;}
        else if(spd<=15){spd=10;}
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
        run(RampR,20);
        run(RampL,20);
        run(RampRb,20);
        run(RampLb,20);
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
      if (offCounter==20)
      {CubeTrack=off;ToCube=off;offCounter=0;}
      

      if(CubeTrack==on)   //if cubetrack==on
      {
      leftDrive((0.0005*pow(TurnDiff,2)+8*pow(10,-16)*TurnDiff+10.8019)-TurnDir); //Run Motors
      rightDrive((0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*TurnDir);
      }
      else if (ToCube==on)
      {
        if (fabs(TurnDiff>15))
        {
        leftDrive(20+(0.0005*pow(TurnDiff,2)+8*pow(10,-16)*TurnDiff+10.8019)-TurnDir);
        rightDrive(20+(0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*TurnDir);
        }
        else
        {
          leftDrive(20);
          rightDrive(20);
        }
        if(CubeSense.pressing() || CubeSense2.pressing())
        {ToCube=off;CubeTrack=off;StopDrive(hold); wait(200);}
        wait(10);
      }
      else{StopDrive(hold);}
    }
    
    wait(20);
  }
  return 0;  
}
