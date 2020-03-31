#include "defines.h"
using namespace G;
using namespace std; 
using namespace vex;
preautoL=false;
  #ifndef DEBUG //if DEBUG is not defined earlier this section will compile
  #include "AllianceSelect.h"
  #endif 

	vex::task fourth (PrintScreen); 
	//vex::task fifth (TurnToCube); 
	vex::task mid2 (TIMER2);
	vex::task okay (GyroTrack);
