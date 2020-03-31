/*vex-vision-config:begin*/
#include "v5.h"
#include "v5_vcs.h"
vex::vision::signature REDCOLOR = vex::vision::signature (1, 7545, 8245, 7896, -1, 835, 416, 6.9, 0);
vex::vision::signature BLUECOLOR = vex::vision::signature (2, -2621, -1819, -2220, 8121, 9161, 8642, 11, 0);
vex::vision::signature GREENFLAG = vex::vision::signature (3, -3447, -2551, -2999, -5969, -5473, -5721, 11, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision = vex::vision (vex::PORT20, 39, REDCOLOR, BLUECOLOR, GREENFLAG, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/