#include "defines.h"
using namespace G;
using namespace std;
using namespace vex;

#ifdef DEBUG // if DEBUG is not defined earlier this section will not compile
/*while(DEBUG){
    if (bUp == 1) { GLOBALP += .01; wait(100); }
            if (bDown == 1) { GLOBALP -= .01; wait(100); }
    if (bX== 1) { GLOBALI += .0000001; wait(100); }
            if (bB == 1) {GLOBALI -= .0000001; wait(100); }
    if (bR1== 1) { GLOBALD += .05; wait(100); }
            if (bR2 == 1) {GLOBALD -= .05; wait(100); }
            wait(10);
}  */
#endif
AutoRunning = 0;
// SnapToCube = off;
GlobalFlagOffset = 160;
MATCHTIMER = 0;
while (1) {
  if (AutoRunning == 0 && (abs(ch3) > 2 || abs(ch4) > 2|| abs(ch1) > 2)) {
    
      run(LF, (ch3 + ch1)); //(Axis3+Axis4)/2
      run(LB, (ch3 + ch1)); //(Axis3+Axis4)/2
      run(LM, (ch3 + ch1));
      run(RF, (ch3 - ch1)); //(Axis3-Axis4)/2
      run(RM, (ch3 - ch1));
      run(RB, (ch3 - ch1)); //(Axis3-Axis4)/2
      run(H1,ch4);
      run(H2,ch4);
  } else if (AutoRunning == 0) {
    StopDrive(brake);
  } else {
  }
  if (intake == on) {if (bR1 && !G::intakeprev) intake = off;run(InR,100);run(InL,100);} 
  else if (intake == off) { if (bR1 && !G::intakeprev)intake = on;run(InR,0);run(InL,0);}
  G::intakeprev = bR1;


  /*if (fly == on) {if (bR1 && !G::flyprev) fly = off;} 
  else if (fly == off) { if (bR1 && !G::flyprev)fly = on;}
  G::flyprev = bR1;
*/
  wait(20);
}