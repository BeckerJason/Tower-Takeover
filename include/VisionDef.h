/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature OCUBE = vex::vision::signature (1, 4865, 7933, 6400, -2449, -1617, -2034, 1.6, 0);
vex::vision::signature PCUBE = vex::vision::signature (2, 315, 1479, 897, 8499, 11565, 10032, 3, 0);
vex::vision::signature GCUBE = vex::vision::signature (3, -8391, -4559, -6475, -2817, -851, -1834, 1.4, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision = vex::vision (vex::PORT4, 60, OCUBE, PCUBE, GCUBE, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/