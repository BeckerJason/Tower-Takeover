// #include "defines.h"
// using namespace G;
// using namespace std;
// using namespace vex;
// //Positional Awareness Code
// void MoveToPosition(float Xf, float Yf, int speed,int MoveSpeed, int time, int adir_overide)
// {
// 	float gyros=GlobalGyro*0.973/10;
// 	int fullRotation = (360); //12.56 inches in distance with 4 inch wheel
// 	float oneFoot = (.955*fullRotation); //oneFoot = 12 inches or 343.95
// 	float tile = 2 * oneFoot;
// 	float Angle;
// 	int adir;
// 	float HypDist = sqrt(pow((Xf - initial.xval), 2) + pow((Yf - initial.yval), 2));
// 	if (HypDist>0)
// 	{
// 		Angle = acos((Xf - initial.xval) / HypDist)*(180/M_PI);
// 	}
// 	else{Angle =0;}
// 	float turnAngle = gyros - Angle;

// 	//writeDebugStreamLine("Distance %f",HypDist);
// 	//writeDebugStreamLine("Angle %f",Angle);
// 	//cout << "Turn Angle" << turnAngle << endl;

// 	if (Yf - initial.yval >= 0)
// 	{
// 		if(turnAngle > 0)
// 		{adir=1;
// 			//TURN RIGHT
// 			//writeDebugStreamLine("Right");
// 			while (GlobalGyro*0.973/10< Angle&&GlobalGyro>0)
// 			{
// 				 run(LM , adir* speed) ;
// 				 run(LF , adir* speed) ;
// 				 run(RM , -adir* speed) ;
// 				 run(RF , -adir* speed) ;
// 			}
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir* speed) ;
// 				 run(LF , adir* speed) ;
// 				 run(RM , -adir* speed) ;
// 				 run(RF , -adir* speed) ;
// 			}

// 			//WAIT TO OVERTURN AND THEN COMPENSATE LEFT
// 			wait(time);
// 			adir=-1;
// 			//writeDebugStreamLine("Left");
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}
// 			while (GlobalGyro*0.973/10< Angle)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}


// 			//WAIT TO OVERTURN AND THEN COMPENSATE RIGHT
// 			wait(time);

// 			adir=1;

// 			//writeDebugStreamLine("Right");
// 			while (GlobalGyro*0.973/10< Angle&&GlobalGyro>0)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}

// 			//BRAKE
// 			 run(LM , -adir*10) ;
// 			 run(LF , -adir*10) ;
// 			 run(RM , adir*10) ;
// 			 run(RF , adir*10) ;
// 			wait(100);
// 			//STOP
// 			 run(LM , 0);
// 			 run(LF , 0);
// 			 run(RM , 0);
// 			 run(RF ,0);
// 			//		right
// 			//		 run(LM , adir*100 ;
// 			// run(LF , adir*100 ;
// 			// run(RM , -adir*100 ;
// 			// run(RF , -adir*100 ;
// 			//Turn(1,Angle);//turn right
// 		}
// 		else if (turnAngle < 0 )
// 		{//TURN LEFT
// 			adir=-1;
// 			//writeDebugStreamLine("Left");
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir*speed) ;
// 				 run(LF , adir*speed) ;
// 				 run(RM , -adir*speed) ;
// 				 run(RF , -adir*speed) ;
// 			}
// 			while (GlobalGyro*0.973/10< Angle)
// 			{
// 				 run(LM , adir*speed) ;
// 				 run(LF , adir*speed) ;
// 				 run(RM , -adir*speed) ;
// 				 run(RF , -adir*speed) ;
// 			}
// 			//WAIT TO OVERTURN AND THEN COMPENSATE RIGHT
// 			wait(time);

// 			adir=1;

// 			//writeDebugStreamLine("Right");
// 			while (GlobalGyro*0.973/10< Angle&&GlobalGyro>0)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}
// 			//WAIT THEN COMPENSATE LEFT
// 			wait(time);
// 			adir=-1;
// 			//writeDebugStreamLine("Left");
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}
// 			while (GlobalGyro*0.973/10< Angle)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}


// 			 run(LM , -adir*10) ;
// 			 run(LF , -adir*10) ;
// 			 run(RM , adir*10) ;
// 			 run(RF , adir*10) ;
// 			wait(200);
// 			 StopDrive(brake);
// 			//Turn(-1,Angle);// turn left
// 		}
// 		else
// 		{
// 			//writeDebugStreamLine("Nope");
// 			Turn(0,Angle);
// 		}

// 	}
// 	else if(Yf - initial.yval < 0){
// 		turnAngle = 360 - turnAngle;
// 		//writeDebugStreamLine("Turn Angle %f",turnAngle);

// 		if(turnAngle > 0)
// 		{adir=1;
// 			//TURN RIGHT
// 			//writeDebugStreamLine("Right");
// 			while (GlobalGyro*0.973/10< Angle&&GlobalGyro>0)
// 			{
// 				 run(LM , adir*speed ;
// 				 run(LF , adir*speed ;
// 				 run(RM , -adir*speed ;
// 				 run(RF , -adir*speed ;
// 			}
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir*speed ;
// 				 run(LF , adir*speed ;
// 				 run(RM , -adir*speed ;
// 				 run(RF , -adir*speed ;
// 			}

// 			//WAIT TO OVERTURN AND THEN COMPENSATE LEFT
// 			wait(time);
// 			adir=-1;
// 			//writeDebugStreamLine("Left");
// 			while (fmod((GlobalGyro*0.973/10+360) ,360) >Angle)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}
// 			while (GlobalGyro*0.973/10< Angle)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}


// 			//WAIT TO OVERTURN AND THEN COMPENSATE RIGHT
// 			wait(time);

// 			adir=1;

// 			writeDebugStreamLine("Right");
// 			while (GlobalGyro*0.973/10< Angle&&GlobalGyro>0)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}
// 			while ((GlobalGyro*0.973/10+360) %360 >Angle)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}

// 			//BRAKE
// 			 run(LM , -adir*10 ;
// 			 run(LF , -adir*10 ;
// 			 run(RM , adir*10 ;
// 			 run(RF , adir*10 ;
// 			wait(100);
// 			//STOP
// StopDrive(brake);
// 		}
// 		else if (turnAngle < 0 )
// 		{	//TURN LEFT
// 			adir=-1;
// 			//writeDebugStreamLine("Left");
// 			while ((GlobalGyro*0.973/10+360) %360 >Angle)
// 			{
// 				 run(LM , adir* speed) ;
// 				 run(LF , adir* speed) ;
// 				 run(RM , -adir* speed) ;
// 				 run(RF , -adir* speed) ;
// 			}
// 			while (GlobalGyro*0.973/10< Angle)
// 			{
// 				 run(LM , adir* speed) ;
// 				 run(LF , adir* speed) ;
// 				 run(RM , -adir* speed) ;
// 				 run(RF , -adir* speed) ;
// 			}
// 			//WAIT TO OVERTURN AND THEN COMPENSATE RIGHT
// 			wait(time);

// 			adir=1;

// 			writeDebugStreamLine("Right");
// 			while (GlobalGyro*0.973/10< Angle&&GlobalGyro>0)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}
// 			while ((GlobalGyro*0.973/10+360) %360 >Angle)
// 			{
// 				 run(LM , adir* speed /2);
// 				 run(LF , adir* speed /2);
// 				 run(RM , -adir* speed /2);
// 				 run(RF , -adir* speed /2);
// 			}
// 			//WAIT THEN COMPENSATE LEFT
// 			wait(time);
// 			adir=-1;
// 			writeDebugStreamLine("Left");
// 			while ((GlobalGyro*0.973/10+360) %360 >Angle)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}
// 			while (GlobalGyro*0.973/10< Angle)
// 			{
// 				 run(LM , adir* speed /3);
// 				 run(LF , adir* speed /3);
// 				 run(RM , -adir* speed /3);
// 				 run(RF , -adir* speed /3);
// 			}


// 			 run(LM , -adir*10 ;
// 			 run(LF , -adir*10 ;
// 			 run(RM , adir*10 ;
// 			 run(RF , adir*10 ;
// 			wait(200);
// StopDrive(brake);
// 			//Turn(-1,Angle);// turn left
// 		}
// 		else
// 		{			//writeDebugStreamLine("Nope");
// 			Turn(0,Angle);

// 		}

// 	}

// 	/*	move(distance);
// 	correctionX = inX;
// 	correctionY = inY;
// 	gyro = angle;
// 	cout << "Turn Angle" << turnAngle << endl;

// 	cout << "correctionX "<<correctionX << endl;
// 	cout << "correctionY "<<correctionY << endl;
// 	cout << "Robot Angle" << gyro << endl;

// 	*/

// 	//writeDebugStreamLine("X%f   Y%f   H%f   TA%f   SV %f    ",(Xf - initial.xval),(Yf - initial.yval), HypDist, turnAngle,gyros);



// 	//MOVE
// 	speed = MoveSpeed;
// 	int dir;
// 	int count = tile*HypDist;			//Determines the encoder count of the travel distance
// 	if (adir_overide > 0) { dir = -1; }	//sets direction of robot movement based on overide
// 	else { dir = 1; }
// 	SensorValue[Renc , 0;
// 	SensorValue[Lenc , 0;
// 	if (dir == 1)
// 	{
// 		while (abs(SensorValue[Lenc])<count)
// 		{
// 			if (abs(SensorValue[Lenc])>abs(SensorValue[Renc]) + 5)
// 			{
// 				 run(LM ,  speed*.8);
// 				 run(LF ,  speed*.8);
// 				 run(RM , speed);
// 				 run(RF ,  speed) ;
// 			}
// 			else if (abs(SensorValue[Lenc])<abs(SensorValue[Renc]) - 5)
// 			{
// 				 run(LM ,  speed) ;
// 				 run(LF ,  speed) ;
// 				 run(RM ,  speed*.8);
// 				 run(RF ,  speed*.8);
// 			}
// 			else
// 			{
// 				 run(LM ,  speed) ;
// 				 run(LF ,  speed) ;
// 				 run(RM ,  speed) ;
// 				 run(RF ,  speed) ;
// 			}
// 		}
// 		 run(LM , -speed  / 4);
// 		 run(LF , -speed  / 4);
// 		 run(RM , -speed  / 4);
// 		 run(RF , -speed  / 4);
// 		wait(100);
// 	}

// 	else if (dir == -1)
// 	{
// 		while (abs(SensorValue[Lenc])<count)
// 		{
// 			if (abs(SensorValue[Lenc])>abs(SensorValue[Renc]) - 5)
// 			{
// 				 run(LM , - speed*.8);
// 				 run(LF , - speed*.8);
// 				 run(RM , - speed) ;
// 				 run(RF , - speed) ;
// 			}


// 			else if (abs(SensorValue[Lenc])<abs(SensorValue[Renc]) + 5)
// 			{
// 				 run(LM , - speed) ;
// 				 run(LF , - speed) ;
// 				 run(RM , - speed*.8);
// 				 run(RF , - speed*.8);
// 			}
// 			else
// 			{
// 				 run(LM , - speed) ;
// 				 run(LF , - speed) ;
// 				 run(RM , - speed) ;
// 				 run(RF , - speed) ;
// 			}

// 		}
// 		 run(LM , speed  / 4);
// 		 run(LF , speed  / 4);
// 		 run(RM , speed  / 4);
// 		 run(RF , speed  / 4);
// 		wait(100);
// 	}
// 	else {}
// StopDrive(brake);


// 	initial.xval = Xf;//set new X and Y values
// 	initial.yval = Yf;

// }
