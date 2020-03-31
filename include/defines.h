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

#ifndef DEBUG  
#define DEBUG
#endif

#ifndef ENUMS
#define ENUMS
typedef enum _Toggle {on=1,off=0} ToggleMode;
ToggleMode intake= off;
ToggleMode Clamp = off;
ToggleMode RunRamp=off;
typedef enum _directional {fwrd=1,bwrd=-1} directional;
directional ramp = bwrd;
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

bool rampprev = false;
bool intakeprev = false;
bool clampprev = false;
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
vex::gyro   Gyro = vex::gyro(Brain.ThreeWirePort.A);
vex::limit  RampLimitBottom = vex::limit(Brain.ThreeWirePort.B);
vex::limit  RampLimit = vex::limit(Brain.ThreeWirePort.D);
vex::line  CubeSense = vex::line(Brain.ThreeWirePort.C);
vex::line  CubeSense2 = vex::line(Brain.ThreeWirePort.E);
vex::motor RampWheelL = vex::motor(vex::PORT1,vex::gearSetting::ratio18_1,true);//left Ramp motor
vex::motor RampWheelR = vex::motor(vex::PORT11,vex::gearSetting::ratio18_1,false);//right Ramp motor
vex::motor LF = vex::motor(vex::PORT5,vex::gearSetting::ratio18_1,false);//front left drivetrain motor
vex::motor LM = vex::motor(vex::PORT6,vex::gearSetting::ratio18_1,false);//middle left drivetrain motor
vex::motor LB = vex::motor(vex::PORT7,vex::gearSetting::ratio18_1,false);//back left drivetrain motor
vex::motor RF = vex::motor(vex::PORT8,vex::gearSetting::ratio18_1,true);//front right drivetrain motor
vex::motor RM = vex::motor(vex::PORT10,vex::gearSetting::ratio18_1,true);//middle right drivetrain motor
vex::motor RB = vex::motor(vex::PORT9,vex::gearSetting::ratio18_1,true);//back right drivetrain motor
vex::motor RightRoller = vex::motor(vex::PORT4,vex::gearSetting::ratio18_1,true);//front right intake motor
vex::motor LeftRoller = vex::motor(vex::PORT15,vex::gearSetting::ratio18_1,false);//front left intake motor
vex::motor ArmL = vex::motor(vex::PORT13,vex::gearSetting::ratio36_1,false);//left arm motor
vex::motor ArmR = vex::motor(vex::PORT12,vex::gearSetting::ratio36_1,true);//right arm motor
vex::motor RampL = vex::motor(vex::PORT2,vex::gearSetting::ratio36_1,true);//left lift motor
vex::motor RampR = vex::motor(vex::PORT3,vex::gearSetting::ratio36_1,false);//right lift motor
 
vex::digital_out RELEASE = vex::digital_out(Brain.ThreeWirePort.H);


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
void StopLift(vex::brakeType);
void CascadeControl();
void AutoStack();
void rightDrive(int);
void leftDrive(int); 
int Driver();
void pidTurn(float , float, float , float, int);
int Move(float, float,bool,vex::brakeType);
void ToWall(double);
void SwapColor();
//void SetDriveTorque(double);
int ENDAUTOTIMER();
int RampControl();

#endif 





/*class motor()
{
  public:

  private:





}
*/