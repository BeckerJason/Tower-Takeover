#include "defines.h"
//Lift.set(false);
//lift=off; 
using namespace G;
using namespace std; 
using namespace vex;
preautoL=false;
     #ifndef DEBUG //if DEBUG is not defined earlier this section will not compile
    #include "AllianceSelect.h"
    #endif 

	vex::task fourth (PrintScreen); 
	//vex::task fifth (TurnToCube); 
	vex::task mid2 (TIMER2);
	vex::task okay (GyroTrack);
