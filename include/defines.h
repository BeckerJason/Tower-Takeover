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
#include "v5.h"
#include "v5_vcs.h"
  
 
#ifndef ENUMS
#define ENUMS
typedef enum _Toggle {on,off} ToggleMode;
//ToggleMode fly = off;
ToggleMode intake=off;
typedef enum _Alliance {Red=2,Blue=1} Alliance;
Alliance Color = Red;  

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
} GreenMarker;

//bool flyprev=false;
bool intakeprev=false;
static float TurnDiff = 0, THeight = 0, TWidth = 0, TurnDir = 1, TXDist = 200, GlobalFlagOffset = 160,TYDist=0;
static int AutoRunning = 0;
static int MoveReturn=0;
static int T1 = 0, T3 = 0;
static float avgSpeed = 0;
static float avgError = 0;
static float GlobalGyro = 0;
static float FinalObject=20;
static float GLOBALP=0.7,GLOBALI=0.000001,GLOBALD=4.1;
static float objheight;
static int selection=0;
int MATCHTIMER=0;
static bool preautoL;
//float GLOBALP=1.4,GLOBALI=0.0000001,GLOBALD=8.1;
};
#endif

 

#ifndef ROBOTSETUP
#define ROBOTSETUP
vex::brain Brain;
vex::controller   Controller = vex::controller(vex::controllerType::primary);
vex::gyro   Gyro = vex::gyro(Brain.ThreeWirePort.H);

vex::motor LF = vex::motor(vex::PORT18,vex::gearSetting::ratio18_1,false);
vex::motor LM = vex::motor(vex::PORT19,vex::gearSetting::ratio18_1,true);
vex::motor LB = vex::motor(vex::PORT20,vex::gearSetting::ratio18_1,false);
vex::motor RF = vex::motor(vex::PORT11,vex::gearSetting::ratio18_1,true);
vex::motor RM = vex::motor(vex::PORT12,vex::gearSetting::ratio18_1,false);
vex::motor RB = vex::motor(vex::PORT13,vex::gearSetting::ratio18_1,true);
vex::motor H1 = vex::motor(vex::PORT16,vex::gearSetting::ratio18_1,true);
vex::motor H2 = vex::motor(vex::PORT17,vex::gearSetting::ratio18_1,false);
vex::motor InL = vex::motor(vex::PORT15,vex::gearSetting::ratio18_1,false);
vex::motor InR = vex::motor(vex::PORT14,vex::gearSetting::ratio18_1,true);



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

#endif

#ifndef FUNCTIONS
#define FUNCTIONS
int GyroTrack();
void run(vex::motor,double);
void runRPM(vex::motor, double);
float enc(vex::motor);
int PrintScreen();
int TurnToCube();
int TIMER2();
void StopDrive(vex::brakeType);
void rightDrive(int);
void leftDrive(int); 
int Driver();
void pidTurn(float , float, float , float, int);
int Move(float, float,bool,vex::brakeType);
void ToWall(double);
//void SetDriveTorque(double);
int ENDAUTOTIMER();

#endif 