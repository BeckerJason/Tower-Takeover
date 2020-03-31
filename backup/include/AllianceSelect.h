//#include "globals.h"
#include "defines.h" 
using namespace G; 
#ifndef DEBUG //if DEBUG is defined earlier this section will not compile   //screen 480x272

 
        
  
	Brain.Screen.clearScreen(vex::color::black);//show red blue and confirm
	selection = 0;
	while (selection < 4)
	{ 
		Brain.Screen.setPenWidth(1);  
		Brain.Screen.setPenColor(vex::color::black);
		if (bRight == 1 || (Brain.Screen.xPosition() > 300 && Brain.Screen.xPosition() < 420 && Brain.Screen.yPosition() > 20 && Brain.Screen.yPosition() < 60)) { selection = 1; Controller.Screen.print("Blue"); }//if Touch within blue box:: selection=1
		else if (bLeft == 1 || (Brain.Screen.xPosition() > 60 && Brain.Screen.xPosition() < 180 && Brain.Screen.yPosition() > 20 && Brain.Screen.yPosition() < 60)) { selection = 2;  Controller.Screen.clearLine(1); Controller.Screen.print("Red"); } //else if Touch within red box:: selection=2
		else if (selection == 3 && Brain.Screen.pressing() == 0) { selection = 4; } //else if Touch within select box&& Selection >0 :: selection=3
		else if ((bA && selection > 0) || (Brain.Screen.xPosition() > 180 && Brain.Screen.xPosition() < 300 && Brain.Screen.yPosition() > 100 && Brain.Screen.yPosition() < 140 && selection > 0)) { selection = 3; } //else if Touch within select box&& Selection >0 :: selection=3

		if (selection == 1) { Color=Blue; Brain.Screen.drawRectangle(297, 17, 126, 46, vex::color::green); Brain.Screen.drawRectangle(57, 17, 126, 46, vex::color::black); }  //if selection==1 draw green box around Blue and Black Box around red
		else if (selection == 2) { Color = Red; Brain.Screen.drawRectangle(297, 17, 126, 46, vex::color::black); Brain.Screen.drawRectangle(57, 17, 126, 46, vex::color::green); }//else if selection==2 draw green box around red and Black Box around blue
		else if (selection == 3) { Brain.Screen.drawRectangle(177, 97, 126, 46, vex::color::green); }//else if selection==3 draw green box around select


		Brain.Screen.drawRectangle(60, 20, 120, 40, vex::color::red);
		Brain.Screen.drawRectangle(300, 20, 120, 40, vex::color::blue);
		Brain.Screen.drawRectangle(180, 100, 120, 40, vex::color::white);
		Brain.Screen.setPenWidth(10);
		Brain.Screen.setPenColor(vex::color::white);
		Brain.Screen.printAt(105, 45, false, "RED");
		Brain.Screen.printAt(340, 45, false, "BLUE");
		Brain.Screen.setPenColor(vex::color::black);
		Brain.Screen.printAt(210, 125, false, "SELECT");
		wait(100);
	}

  Brain.Screen.setPenColor(color::black);
 //screen 480x272
  if (Color==1)	Brain.Screen.clearScreen(vex::color::blue);//show blue back
  else Brain.Screen.clearScreen(vex::color::red);//show red back
  Brain.Screen.drawRectangle(20, 1, 430, 300, vex::color::black);
  selection = 1;
  int select=0;
  while(1)
  { 
for(int i=1;i<=4;i++)
{ 
  if(i==selection){Brain.Screen.setPenColor(color::green);}
else {Brain.Screen.setPenColor(color::black);}
Brain.Screen.drawCircle(50, 40*i, 15,vex::color::white );

}
Brain.Screen.setPenColor(color::white);

//Brain.Screen.setFillColor(color::black);
if (Color==1)
{
Brain.Screen.printAt(70, 43, false, "Blue1");
Brain.Screen.printAt(70, 83, false, "Blue2");
Brain.Screen.printAt(70, 123, false, "Blue3");
Brain.Screen.printAt(70, 163, false, "Blue4");
}
else 
{
Brain.Screen.printAt(70, 43, false, "Red1");
Brain.Screen.printAt(70, 83, false, "Red2");
Brain.Screen.printAt(70, 123, false, "Red3");
Brain.Screen.printAt(70, 163, false, "Red4");
}
if(Brain.Screen.yPosition() < 170)
{
if (Brain.Screen.yPosition() > 150 ){selection=4;}
else if (Brain.Screen.yPosition() > 110){selection=3;}
else if (Brain.Screen.yPosition() > 70){selection=2;}
else if (Brain.Screen.yPosition() > 30){selection=1;}
}
if (selection>1&&selection<4)
{
  selection-=bUp;
  selection+=bDown;
while(bUp||bDown){wait(10);}
}

if (select == 1) { Brain.Screen.drawRectangle(177, 177, 126, 46, vex::color::green); }//else if selection==3 draw green box around select
Brain.Screen.setPenColor(color::black);
if ((bA && selection > 0) || (Brain.Screen.xPosition() > 180 && Brain.Screen.xPosition() < 300 && Brain.Screen.yPosition() > 180 && Brain.Screen.yPosition() < 210 && selection > 0)) { select=1; } //else if Touch within select box&& Selection >0 :: selection=3
Brain.Screen.drawRectangle(180, 180, 120, 40, vex::color::white);
Brain.Screen.printAt(210,210, false, "SELECT");
if (select == 1 && Brain.Screen.pressing() == 0) { break; } //else if Touch within select box&& Selection >0 :: selection=3
wait(75);
  }
wait(500);

#endif