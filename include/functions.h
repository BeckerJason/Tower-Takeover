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
	int counter=0;
	//arm.suspend();
	//rampcontroller.suspend();
	//IntakeController.suspend();
	//timer2.suspend();
	//rampwheel.suspend();
	//printscreen.suspend();
	wait(3500);                             //wait (X) ms
	inertial::quaternion  Inertial_quaternion;
  Inertial.calibrate();
	wait(2000);                             //wait (X) ms
  
	//arm.resume();
	//rampcontroller.resume();
	//IntakeController.resume();
	//timer2.resume();
	//rampwheel.resume();
#ifdef DEBUG
	//printscreen.resume();
#endif
	while (1) {
		Inertial_quaternion = Inertial.orientation();
    CurrentGyro=Inertial.heading()*10;
		//if going from 3600 to 0
		if(GyroTCheck>2500&&GyroTCheck<=3600&& CurrentGyro<1000)
		{GyroAdd=3600-GyroTCheck+ CurrentGyro;}
		//if going gtom -3600 to 0
		else if(GyroTCheck<-2500&&GyroTCheck>-3600&& CurrentGyro>-1000)
		{GyroAdd=-3600-GyroTCheck+ CurrentGyro;}
		//if going from 0 to 3600
		else if (GyroTCheck>-1000&&GyroTCheck<1000&&CurrentGyro>2500)
		{GyroAdd=-GyroTCheck+ 3600-CurrentGyro;}
		//going from 0 to -3600
		else if (GyroTCheck<1000&&GyroTCheck>-1000&&CurrentGyro<-2500)
		{GyroAdd=-GyroTCheck+3600-CurrentGyro;}

		else {GyroAdd = GyroTCheck-CurrentGyro;}  
		GyroTCheck = CurrentGyro;
		if (GyroAdd>0)
		{GlobalGyro += GyroAdd;GlobalGyroT += GyroAdd;}
		else{GlobalGyro+=GyroAdd;GlobalGyroT+=GyroAdd;}

		/*file.open("DATA8.csv",ios::out | ios::app | ios::ate );
		file<<counter<<","<< Inertial.heading()*10<<","<<GlobalGyro<<","<<GyroTCheck<<endl;
		file.close();
		counter++;
		*/
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
		
		Brain.Screen.printAt(1, 60, "Gyro:  %1.2f", GlobalGyro);
    Brain.Screen.printAt(1, 80, "GyroT:  %1.2f", GlobalGyroT);
    Brain.Screen.printAt(1, 100, "ArmR:  %1.2f", enc(ArmR));
    Brain.Screen.printAt(1, 120, "ArmL:  %1.2f", enc(ArmL));
    Brain.Screen.printAt(1, 140, "intake:  %1.2f", intake);
    Brain.Screen.printAt(1, 160, "RunRamp:  %1.2f", RunRamp);
    Brain.Screen.printAt(1, 180, "ramp:  %d", ramp);
    Brain.Screen.printAt(1, 200, "RampL:  %1.2f", enc(RampL));
    Brain.Screen.printAt(1, 220, "Color (R -1, B 1):  %d", Color);
    //Brain.Screen.printAt(1, 240, "NULL:  %1.2f", false);
		//Brain.Screen.printAt(1, 90, "%f",  Gyro.value(vex::rotationUnits::raw));
		//Brain.Screen.printAt(340, 20, "ARM: %1.1f", enc(ArmR));
		//Brain.Screen.printAt(100, 40, "P: H:%d    Y:%d", Vision.objects[FinalObject].height, Vision.objects[FinalObject].centerY);
		//Brain.Screen.printAt(100, 80, "green %X ", GreenCube.centerX[0]);
		Brain.Screen.printAt(340, 140, "MT %d ", MATCHTIMER);
		//Brain.Screen.printAt(340, 160, "Ramp %d ", ramp);
		//Brain.Screen.printAt(340, 180, "RampEnc %f ", enc(RampR));
		//Brain.Screen.printAt(340, 200, "cube %d ", CubeSense.pressing());
		//Brain.Screen.printAt(340, 220, "cube %d ", CubeSense2.pressing());
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
int TurnG(double degree, double speed, int TimeOut)
{ 
  double slack=0.5;// degrees of slack left and right
  double dir=1;
  double turnspeed=speed;
	degree*=10;
  slack*=10;
  double initgyro=GlobalGyro;
  double val=1;
  while(GlobalGyro>degree+slack||GlobalGyro<degree-slack)
  {
    while(GlobalGyro>degree+slack||GlobalGyro<degree-slack)
    {
      val=fabs(GlobalGyro/(fabs(initgyro+.01)-fabs(GlobalGyro)));
      turnspeed=speed*(7.19*pow(val,4)-14.388*pow(val,3)+5.659*pow(val,2)+1.5349*val+0.1419);
      if(GlobalGyro>degree) //need to go rigt
      {
        leftDrive(turnspeed);
        rightDrive(-turnspeed);//go right
      }
      else
      {
        leftDrive(-turnspeed);
        rightDrive(turnspeed);//go left
      }
     wait(10); 
    }
    StopDrive(brake);
    wait(150);
  }
  return 1;
}

void Turn(double degrees, double speed, int TimeOut)//, int Timeout)
{
	double dir=1;
	//GlobalGyro = 0;
	Inertial.resetRotation();
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
	while(fabs(Inertial.rotation()*10) < fabs(degrees))
	{
		val=fabs((Inertial.rotation()*10)/(degrees));
		turnspeed=speed*(7.19*pow(val,4)-14.388*pow(val,3)+5.659*pow(val,2)+1.5349*val+0.1419);
		if(7.19*pow(val,4)-14.388*pow(val,3)+5.659*pow(val,2)+1.5349*val+0.1419>.8)
		{turnspeed=speed*.8;}

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
			turnspeed=speed*(7.19*pow(val,4)-14.388*pow(val,3)+5.659*pow(val,2)+1.5349*val+0.1419);



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
///////////////////////////////////////////////////////////////////////////
int moveg(float speed, float dist, bool rampspeed, double gyroheading, int functiontimer)///Setting gyroheading to 10000 keeps robot in direction its facing 
{
  gyroheading*=10;
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

  double TGyro=0;
  if(gyroheading==10000){TGyro=GlobalGyro;}
  else {TGyro=gyroheading;}
	while (fabs(enc(LM)) < fabs((dist * 360.0 / (4.0 * 3.14159))) && T3 < functiontimer&&!bLeft&&!bDown) {
  
		float Roffset = 1.0;
		if (GlobalGyro>TGyro+5) {
			Roffset = 1-dir*0.1;
			}
       else if (GlobalGyro<TGyro-5) {
			Roffset = 1+dir*0.1;
			}
       else {
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
	while (fabs(enc(LM)) < fabs((dist * 360.0 / (4.0 * 3.14159))) && T3 < functiontimer&&!bLeft&&!bDown) {

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
			else if (intake == manual){run(RightRoller, ManualSpeed); run(LeftRoller, ManualSpeed);if(bB){intake=off;}}
			else if (intake == off) {if ((bB||bR1||ControlIntake==on) && !G::intakeprev) {intake = on;ControlIntake=off;}run(RightRoller, 100); run(LeftRoller, 100);}
			else if (intake == on) {
				if ( ((bB||ControlIntake==on) && !G::intakeprev)) {intake = off;ControlIntake=off;}run(RightRoller, 0); run(LeftRoller, 0);}
			G::intakeprev = bB||bR1;

			//INTAKE
			if (bY==1){run(RightRoller, -80); run(LeftRoller, -80);}
		}
		wait(10);
	}
	return 0;
}
int AutoStack()
{
  // 
  
  AutoRunning=1;
  StopDrive(hold);
  arm.suspend();
  intake=off;
  DontLiftStack=on;
  DontDropStack=off;
  if(CubeSense2.pressing()==1){DontDropStack=on;}
  ramp=fwrd;
  RunRamp=on;
  while(RunRamp==on){wait(20);}
  while(bLeft){wait(10);}
  wait(200);
  //ArmL.setVelocity(50,vex::velocityUnits::pct);
  //ArmR.setVelocity(50,vex::velocityUnits::pct);
  //ArmL.startRotateTo(90,rotationUnits::deg);
  //ArmR.startRotateTo(90,rotationUnits::deg);
  ManualSpeed=-35;
  intake=manual;
  MoveG(40,-8,1,1000,coast,3000);
  ramp=bwrd;
  RunRamp=on;
  //MoveG(40,-7.8,1,1000,brake,3000);
  MoveG(40,-12.8,1,1000,brake,3000);
  ArmR.startRotateTo(0,rotationUnits::deg);
  ArmL.startRotateTo(0,rotationUnits::deg);
  if (AutoRunning==1){wait(1000);}
  AutoRunning=0;
  arm.resume();
  intake=off;
  return 0;
}
///////////////////////////////////////////
////////////////////////////////////////////////////
int RampWheels()
{ 
	while(1)
	{
		if(enc(RampR)<-100 /*RampLimitBottom.pressing()==0*/||RunRamp==on){}
		else if (bY==1&&DontLiftStack==0&abs(enc(ArmL))<100){run(RampWheelL,-100);run(RampWheelR,-100);}
		else if ( (CubeSense.pressing() || CubeSense2.pressing()) &&  DontLiftStack==0){run(RampWheelL,90);run(RampWheelR,90);}
		else {BRAKE(RampWheelL,hold);BRAKE(RampWheelR,hold);}
		wait(50);

	}


	return 0;
}

int ArmControl()
{ 
	while(1)
	{
    if(AutoRunning==0)
    {
		if (bR1==1&&enc(ArmR)<500)//Chain bar arm control
		{
			//intake=off;
      
			run(ArmR, 70);
			run(ArmL,70);
			T4=0;
			//ArmL.resetRotation();
		}
		else if (bR2==1&&enc(ArmR)>0)
		{
      
			run(ArmR, -70);
			run(ArmL, -70);
			T4=0;
		}
		else
		{

			if (T4>250&&T4<500)
			{
				BRAKE(ArmR, hold);
				ArmL.rotateTo(enc(ArmR), rotationUnits::deg);
			}
			else
			{
				BRAKE(ArmR, hold);
				BRAKE(ArmL, hold);
			}
    }
		}
    wait(10);
	}
	return 0;
}
std::ofstream file;
int RampControl() //Function to be run as a task. this controls the ramp
{ 
	RunRamp=off;
	int oldval=ramp;
	RampL.setMaxTorque(100, percentUnits::pct);
	RampR.setMaxTorque(100, percentUnits::pct);
	BRAKE(RampL,coast);
	BRAKE(RampR,coast);
  double speedmult=0;


	while(1)
	{


		while(RunRamp==0){wait(50);}
		//Ramp -1 = up
		run(RightRoller,0);
		run(LeftRoller,0);
		//if (enc(RampR)>0 /*RampLimitBottom.pressing()==1*/){RampR.resetRotation();}
    speedmult=0;
		if (ramp==-1)//ramp up limit  //RAMP UP
		{
			if (DontDropStack==0)
			{
				RampWheelL.setVelocity(25, velocityUnits::rpm);
				RampWheelR.setVelocity(25, velocityUnits::rpm);
				RampWheelL.startRotateFor(directionType::fwd, -175, rotationUnits::deg);
				RampWheelR.startRotateFor(directionType::fwd, -175, rotationUnits::deg);
			}
			double spd=.3;
			while(enc(RampR)>-440&&bLeft==0&&ramp==-1) //MOVE UP
			{
				//spd=5*pow(10,-10)*pow(enc(RampR),4)+3*pow(10,-7)*pow(enc(RampR),3)+0.0002*pow(enc(RampR),2)+0.1915*enc(RampR)+64;
        spd = 75 - .85/4.5*(((enc(-1*RampR)+enc(-1*RampR)))/2);
        spd*=speedmult;
        //file.open("DATA19.csv",ios::out | ios::app | ios::ate );
        if (spd <= 15&&speedmult>=1){spd=15;}
				if (bL2)
				{spd=spd*0.2;}
				run(RampR,-spd);
				run(RampL,-spd);
				wait(20);
        //file<<abs(enc(RampR))<<","<< spd<<","<<abs(RampR.velocity(percentUnits::pct))<<","<<abs(RampL.velocity(percentUnits::pct))<<endl;
		    //file.close();
        if(speedmult<1){speedmult+=.02;}
			}
			if (ramp==-1){RunRamp=off;}

			BRAKE(RampR,hold);
			BRAKE(RampL,hold);
		}
		else
		{
			while(enc(RampR)<-20&&bLeft==0&&ramp==1)
			{
				run(RampR,40);
				run(RampL,40);
				wait(20);
			}
			if (ramp==1){RunRamp=off;}
			BRAKE(RampR,coast);
			BRAKE(RampL,coast);
			wait(750);
			if(abs(enc(RampR))<100)
			{RampR.resetRotation();
				RampL.resetRotation();
			}
		}
		DontDropStack=off;
		run(RampL,0);run(RampR,0);
    wait(10);
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
				leftDrive((0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*-TurnDir); //Run Motors
				rightDrive((0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*TurnDir);
			}
			else if (ToCube==on)
			{
				if (fabs(TurnDiff>15))
				{
					leftDrive(20+(0.0005*pow(TurnDiff,2)*+8*pow(10,-16)*TurnDiff+10.8019)*-TurnDir);
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

void ArcTurn(float degree, float radius, char FBD, char LRD) //fbd fwd bkwd  lrd left right
{
  T3=0;
	LM.resetRotation();
  RM.resetRotation();
  double TGyro=GlobalGyro;
  TGyro=0;
	//90 degrees
  double BaseSpeed=35;
	float speedL, speedR,dirL,dirR;
	float Larcdist, Rarcdist,LR,dir,Loffset, Roffset, Rmult, Lmult; //LeftRight, Forward Backward

	if (LRD=='L' ||LRD=='l'){LR=-1.0;}   			////LEFT -1
	else{LR=1.0;}//

	if (FBD=='F'||FBD=='f'){dirL=1.0;dirR=1.0;}					////FORWARD 1
	else{dirL=-1.0;dirR=-1.0;}

	degree=degree*(M_PI/180.0);
	if(LR>0){Larcdist=degree*(radius+7.5); Rarcdist=degree*(radius-7.5); /*Lmult=(radius+7.5); Rmult=(radius-7.5);*/ }///DEtermine if left side or right side is travelling shorter or longer dist
	else {Larcdist=degree*(radius-7.5); Rarcdist=degree*(radius+7.5);/*Lmult=(radius-7.5); Rmult=(radius+7.5);*/}
	Larcdist= (Larcdist*360)/(4.1*M_PI);///Change 261 (IME Turbo) for 360 for external Motor controllers
	Rarcdist= (Rarcdist*360)/(4.1*M_PI);///Change 261 (IME Turbo) for 360 for external Motor controllers /////(degree*(radius+7.5)*360)/(4.1*M_PI)
	//Lmult=(Larcdist/Rarcdist); Rmult=(Rarcdist/Larcdist);
	//if (Lmult>1){Lmult=Lmult/3; Rmult=Rmult*3;}
	//  else{Lmult=Lmult*3; Rmult=Rmult/3;}
	while ((abs(enc(LM))<Larcdist||abs(enc(RM))<Rarcdist)&&T3<3000)
	{
		//if (abs(enc(RM))<abs(enc(LM))){Loffset=dir*10; Roffset=0;}
		//else if (abs(enc(LM))<abs(enc(RM))){Loffset=0; Roffset=-dir*10;}
		//else{Loffset=0; Roffset=0;}

		//xL=(Larcdist-abs(enc(LM)));
		//xR=(Rarcdist-abs(enc(RM)));
		//speedL=(-35.0)*cos(17.5*abs(xL)/(abs(Larcdist)*3.14)+5.75)+30.0;//vel;
		//speedR=(-35.0)*cos(17.5*abs(xR)/(abs(Rarcdist)*3.14)+5.75)+30.0;//vel;
		if(Rarcdist>Larcdist){speedL=dirL*BaseSpeed*(Larcdist/Rarcdist);speedR=dirR*BaseSpeed;}//4.75
		else{speedL=dirL*BaseSpeed;speedR=dirR*BaseSpeed*(Rarcdist/Larcdist);}

		// if (LRD=='L' ||LRD=='l')
		// {
		// 	if(abs(enc(LM))/Larcdist<abs(enc(RM))/Rarcdist)
		// 	{Loffset=1; Roffset=0.75;}
		// 	else if (abs(enc(LM))/Larcdist>abs(enc(RM))/Rarcdist)
		// 	{Loffset=0.75; Roffset=1;}
		// 	else{Loffset=1; Roffset=1;}
		// }   			////LEFT -1
		// else
		// {
			if((abs(enc(LM))/Larcdist)+(Larcdist*.01)<abs(enc(RM))/Rarcdist)
			{Loffset=1; Roffset=0.9;}
			else if (abs(enc(LM))/Larcdist-(Larcdist*.01)>abs(enc(RM))/Rarcdist)
			{Loffset=0.9; Roffset=1;}
			else{Loffset=1; Roffset=1;}
		// }//
if (abs(enc(LM))>Larcdist){dirL*=-1.0;}
else if(abs(enc(RM))>Rarcdist){dirR*=-1.0;}
		rightDrive(speedR*Roffset);
		leftDrive(speedL*Loffset);
    wait(20);
	}
	StopDrive(brake);
}

void ArcTurnG(float degree, float radius) //FL:d,r FR:-d,r BL:-d,-r BR:d,-r
{
  
  	LM.resetRotation();
  RM.resetRotation();
double slack=1.0; //Correction slack, smaller number means tighter turn tolerance
double wb=12.0; //wheelbase width
	double basespeed = 30.0;	//Base speed
	double dir=1.0, adir=1.0, LS, RS;
	if (degree > 0) { adir = 1.0; }
	else { adir = -1.0; }
	if (radius > 0) { dir = 1.0; } //FORWARD
	else { dir = -1.0; }			//BACKWARD
	GlobalGyroT = 0; //******************
	double Tenc=0;
	while (fabs(GlobalGyroT) / 10 < fabs(degree))
	{	

		if (degree*radius > 0) {Tenc=fabs(enc(RM)); }///LEFT
		else {Tenc=fabs(enc(LM)); }	//RIGHT

		//cout <<"Angle:"<< adir * (2 * fabs(Tenc) / (fabs(radius) + (wb / 2))) << endl;
		//cout << "Gyro:"; cin >> GlobalGyroT;
		//GlobalGyroT *= 10;
		RS = LS = 1;
		if (GlobalGyroT / 10.0 < adir*(2.0 *Tenc / (fabs(radius) + (wb / 2.0)) - slack)) //if Gyro< curve angle
		{
			//cout << "-" << endl;
			if (radius > 0) { LS =0.1/* 0.5/fabs(fabs(GlobalGyroT / 10)-fabs((2.0 *Tenc / (fabs(radius) + (wb / 2.0)))))*/ ; }//fwd
			else { RS=0.1/* 0.5/fabs(fabs(GlobalGyroT / 10)-fabs((2.0 *Tenc / (fabs(radius) + (wb / 2.0)))))*/ ; } //bwd
		}
		else if (GlobalGyroT / 10.0 > adir*(2.0 * Tenc / (fabs(radius) + (wb / 2.0)) + slack))
		{
			//cout << "+" << endl;
			if (radius > 0) { RS = 0.1/* 0.5/fabs(fabs(GlobalGyroT / 10)-fabs((2.0 *Tenc / (fabs(radius) + (wb / 2.0)))))*/ ; }//fwd
			else { LS =0.1/* 0.5/fabs(fabs(GlobalGyroT / 10)-fabs((2.0 *Tenc / (fabs(radius) + (wb / 2.0)))))*/ ; } //bwd
		}
		//cout<<"L:"<<dir*basespeed*LS<<"     R:"<<dir*basespeed*RS<<endl;
		/*Controller.LCD.clearScreen();
    Controller.LCD.setCursor(1,1);
    Controller.LCD.print("%1.2f  %1.2f",dir*basespeed*LS,dir*basespeed*RS);
    Controller.LCD.setCursor(2,1);
    Controller.LCD.print("%1.2f  %1.2f",GlobalGyroT/10,adir*(2.0 *Tenc / (fabs(radius) + (wb / 2.0))));*/
    leftDrive(dir*basespeed*LS);
		rightDrive(dir*basespeed*RS);
    //Tenc += 50;
    wait(20);
	}
StopDrive(brake);
}


/*void ArcTurnG(float degree, float radius) //fbd fwd bkwd  lrd left right
{
  T3=0;
	LM.resetRotation();
  RM.resetRotation();
  GlobalGyroT=0;
	//90 degrees
  double BaseSpeed=15;
	float speedL, speedR,dirL,dirR;
	float Larcdist, Rarcdist,LR,dir,Loffset, Roffset, Rmult, Lmult; //LeftRight, Forward Backward
	//	if(degree*radius>0){}///LEFT
	//else{}	//RIGHT
	if(radius>0){dir=1.0;} //FORWARD
	else {dir=-1.0;}			//BACKWARD
  //if (FBD=='F'||FBD=='f'){dir=1.0;}					////FORWARD 1
	//else{dir=-1.0;}
  while(fabs(GlobalGyroT)<fabs(degree*10))
  { Roffset=Loffset=1;
    if (degree*radius>0)//LEFT
    {LR=1.0;
      if (GlobalGyroT/10<2*fabs(enc(RM))/(radius+6))  //360.0 / (4.0 * 3.14159
      {Roffset=dir*1+dir*1;    }
      else if (GlobalGyroT/10>2*fabs(enc(RM))/(radius+6))
      {Roffset=dir*1-dir*1;    }   
    }   			////LEFT -1
    else
    {
      LR=-1.0;
     if (-GlobalGyroT/10<2*fabs(enc(LM))/(radius+6))  //360.0 / (4.0 * 3.14159
      {Loffset=dir*1+dir*1;    }
      else if (-dir*GlobalGyroT/10>2*fabs(enc(LM))/(radius+6))
      {Loffset=dir*1-dir*1;    }   
    }//
    leftDrive(BaseSpeed*dir*Loffset);
    rightDrive(BaseSpeed*dir*Roffset);
    wait(10);
  }
  StopDrive(brake);

  //T(degree)
}*/