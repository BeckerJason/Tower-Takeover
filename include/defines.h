/*----------------------------------------------------------------------------*/
/*                                                                            */
/*  This file includes standard libraries as well as the vex libraries        */
/*      GLOBAL VARIABLES ARE ENTERED IN THE 'G' NAMESPACE HERE                */
/*          ROBOT  PORT CONFIGURATIONS ARE ENTERED HERE                        */
/*            PROTOTYPES FOR FUNTIONS ARE ENTERED HERE                        */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <math.h>   
#include <cmath>
#include <cstdlib>
#include <fstream>
#include "v5.h"
#include "v5_vcs.h"

#ifndef DEBUG  
#define DEBUG
#endif

#ifndef ENUMS
#define ENUMS
typedef enum _Toggle {on=1,off=0,manual=6} ToggleMode;
ToggleMode intake= off;
ToggleMode RunRamp=off;
ToggleMode CubeTrack =off;
ToggleMode  OTrack=off;
ToggleMode  PTrack=off;
ToggleMode  GTrack=off;
ToggleMode ToCube=off;
ToggleMode DontLiftStack=off;
ToggleMode DontDropStack=off;
ToggleMode ContinueStack=off;
ToggleMode ControlIntake=off;
typedef enum _directional {fwrd=-1,bwrd=1} directional;
directional ramp = bwrd;
typedef enum _Alliance {Red=2,Blue=1} Alliance;
Alliance Color = Blue;  

#endif


#ifndef GLOBALS
#define GLOBALS 

namespace G{
  #define TOTALSNAPSHOTS 7
class VisionObject {
  public:
          int     id[TOTALSNAPSHOTS];
          int     originX[TOTALSNAPSHOTS];
          int     originY[TOTALSNAPSHOTS];
          int     centerX[TOTALSNAPSHOTS];
          int     centerY[TOTALSNAPSHOTS];
          int     width[TOTALSNAPSHOTS];
          int     height[TOTALSNAPSHOTS];
          double  angle[TOTALSNAPSHOTS];
          bool    exists[TOTALSNAPSHOTS];
} PurpleCube,GreenCube,OrangeCube;

bool rampprev = false;
bool intakeprev = false;
bool clampprev = false;
static float TurnDiff = 0, THeight = 0, TWidth = 0, TurnDir = 1, TXDist = 200, GlobalCubeOffset = 150,TYDist=0;
static int AutoRunning = 0;
static int MoveReturn=0;
static int T1 = 0, T3 = 0,T4=0;
static float avgSpeed = 0;
static float avgError = 0;
static float GlobalGyro = 0,GlobalGyroT=0;
static float FinalObject=20;
static float GLOBALP=0.7,GLOBALI=0.000001,GLOBALD=4.1;
static float objheight;
static int selection=0;
int MATCHTIMER=0;
static bool preautoL;
double ManualSpeed;   
static double spd;
double FSpeed=0;
double TSpeed=0; 
//float GLOBALP=1.4,GLOBALI=0.0000001,GLOBALD=8.1;
};
#endif

 

#ifndef ROBOTSETUP
#define ROBOTSETUP
vex::brain Brain;
vex::controller   Controller = vex::controller(vex::controllerType::primary);
vex::digital_out Piston=vex::digital_out(Brain.ThreeWirePort.A);
vex::gyro   Gyro = vex::gyro(Brain.ThreeWirePort.A);
vex::limit CubeSense = vex::limit(Brain.ThreeWirePort.F);
vex::limit CubeSense2 = vex::limit(Brain.ThreeWirePort.C);
vex::motor RampWheelR = vex::motor(vex::PORT6,vex::gearSetting::ratio18_1,false);//right Ramp wheel motor
vex::motor RampWheelL = vex::motor(vex::PORT17,vex::gearSetting::ratio18_1,true);//left Ramp wheel motor
vex::motor LF = vex::motor(vex::PORT18,vex::gearSetting::ratio18_1,false);//front left drivetrain motor
vex::motor LM = vex::motor(vex::PORT19,vex::gearSetting::ratio18_1,false);//middle left drivetrain motor
vex::motor LB = vex::motor(vex::PORT20,vex::gearSetting::ratio18_1,false);//back left drivetrain motor
vex::motor RF = vex::motor(vex::PORT8,vex::gearSetting::ratio18_1,true);//front right drivetrain motor
vex::motor RM = vex::motor(vex::PORT9,vex::gearSetting::ratio18_1,true);//middle right drivetrain motor
vex::motor RB = vex::motor(vex::PORT10,vex::gearSetting::ratio18_1,true);//back right drivetrain motor
vex::motor RightRoller = vex::motor(vex::PORT4,vex::gearSetting::ratio18_1,false);//front right intake motor
vex::motor LeftRoller = vex::motor(vex::PORT14,vex::gearSetting::ratio18_1,true);//front left intake motor
vex::motor ArmL = vex::motor(vex::PORT16,vex::gearSetting::ratio36_1,true);//left arm motor
vex::motor ArmR = vex::motor(vex::PORT3,vex::gearSetting::ratio36_1,false);//right arm motor
vex::motor RampL = vex::motor(vex::PORT11,vex::gearSetting::ratio36_1,true);//left Ramp lift motor
vex::motor RampR = vex::motor(vex::PORT1,vex::gearSetting::ratio36_1,false);//right Ramp lift motor
vex::motor RampLb = vex::motor(vex::PORT12,vex::gearSetting::ratio36_1,true);//left Ramp lift motor
vex::motor RampRb = vex::motor(vex::PORT2,vex::gearSetting::ratio36_1,false);//right Ramp lift motor
//Vision on port 13

#include "VisionDef.h"
#define   bL2  Controller.ButtonL2.pressing()
#define   bL1 Controller.ButtonL1.pressing()
#define   bA  Controller.ButtonA.pressing()
#define   bR2  Controller.ButtonR2.pressing()
#define   bR1  Controller.ButtonR1.pressing()
#define   bB  Controller.ButtonB.pressing()
#define   bX  Controller.ButtonX.pressing()
#define   bY  Controller.ButtonY.pressing()
#define   bLeft Controller.ButtonLeft.pressing()
#define   bRight  Controller.ButtonRight.pressing()
#define   bUp  Controller.ButtonUp.pressing()
#define   bDown  Controller.ButtonDown.pressing()
#define   ch3  Controller.Axis3.value()
#define   ch4  Controller.Axis4.value()
#define   ch1  Controller.Axis1.value()
#define   ch2  Controller.Axis2.value()
#define   wait vex::task::sleep

#define TOTALSNAPSHOTS 7
#define BRAKE(x,y) x.stop(vex::brakeType::y)
#define StopDrive(brake)  RF.stop(vex::brakeType::brake);\
                          RB.stop(vex::brakeType::brake);\
                          RM.stop(vex::brakeType::brake);\
                          LF.stop(vex::brakeType::brake);\
                          LB.stop(vex::brakeType::brake);\
                          LM.stop(vex::brakeType::brake)
#define StopRamp(brake)   RampL.stop(vex::brakeType::brake);\
                          RampR.stop(vex::brakeType::brake);\
                          RampRb.stop(vex::brakeType::brake);\
                          RampLb.stop(vex::brakeType::brake)
#define StopArm(brake)    ArmL.stop(vex::brakeType::brake);\
                          ArmR.stop(vex::brakeType::brake)
#define enc(x) x.rotation(vex::rotationUnits::deg)
#define run(x,y) x.spin(vex::directionType::fwd, y, vex::velocityUnits::pct);
#define runrpm(x,y) x.spin(vex::directionType::fwd, y, vex::velocityUnits::rpm);
#define tower 500 //arm encoder count to reach tower
#define Move(w,x,y,brake,z) move(w,x,y,z);\
                            RF.stop(vex::brakeType::brake);\
                          RB.stop(vex::brakeType::brake);\
                          RM.stop(vex::brakeType::brake);\
                          LF.stop(vex::brakeType::brake);\
                          LB.stop(vex::brakeType::brake);\
                          LM.stop(vex::brakeType::brake)
#endif



#ifndef FUNCTIONS
#define FUNCTIONS 
int GyroTrack();                //Task to keep track of gyro    *GlobalGyro                
int PrintScreen();              //Task to print to Brain      
int TurnToCube();               //Task to turn to Cube      *CubeTrack
int TIMER2();                   //Task for Timer            *T3
int ENDAUTOTIMER();             //Task for match timer      *MATCHTIMER
int RampControl();              //Task for ramp control     *
int PrintController();          //Task to print to controller
int IntakeControl();            //Task to control intake
void rightDrive(int);           //Right Drive
void leftDrive(int);            //Left drive                          
void pidTurn(float , float, float , float, int);  //Turn Function 
int move(float, float,bool,int);       //Move(speed , distance inches, ramp to max speed, end brake type)
void ToWall(double);                              //ToWall(speed)
int ArmControl();
int RampWheels();
int CubeLoad();
//void SetDriveTorque(double);
void Colors(ToggleMode,ToggleMode,ToggleMode);

	vex::task printscreen (PrintScreen); 
	//vex::task fifth (TurnToCube); 
	vex::task timer2 (TIMER2);
	vex::task gyrotrack (GyroTrack);
  //task starttimer (ENDAUTOTIMER);         //start timer task
  vex::task arm (ArmControl);
  vex::task rampcontroller (RampControl);      //start ramp control task
  vex::task IntakeController (IntakeControl);
  vex::task controllerprint (PrintController);
  vex::task cubes (TurnToCube);
  vex::task rampwheel (RampWheels);
  vex::task cubeload (CubeLoad);
  

#endif 
