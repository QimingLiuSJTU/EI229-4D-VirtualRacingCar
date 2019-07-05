/***************************************************************************

file : driver_cruise.cpp
description : user module for CyberFollow

***************************************************************************/

/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_follow.h"
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_follow";	// name of the module (short).
	modInfo[0].desc = "user module for CyberFollower";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

/*
WARNING!

DO NOT MODIFY CODES ABOVE!
*/

/*
define your variables here.
following are just examples
*/
/* variables */

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;
const int topGear = 6;
float thistimedistance;
float lasttimedistance;
float lasttimeLeaderspeed;
float Leaderspeed;
float Leaderacc;
float expectedspeed;
float curSpeedErr;
float speedErrSum;
float speedErrDiff;
float tmp;
float Tmp;
bool isdangerous;
float D_err;
float D_errDiff = 0;
float D_errSum = 0;
float lastx1;
int num;
float averageacc[5];
float acc;
circle c;

double kp_s = 0.03;//0.03
double ki_s = 0;
double kd_s = 0;
double kp_d = 1;
double ki_d = 0;
double kd_d = 0;

const int interval = 3;//3
static double lastX;
static double lastY;
//double last_speed[interval];
static double last_leader_speed;
//double last_relative_speed;
double transX;
double transY;
static double last_yaw;
double del_yaw;
double distance;
double k;
int cnt = 0;
int count = 0;
double lead_y[3];
double lead_v[5];
int startPoint;
int delta;
int wcount=0;
float speedsum;
float averagespeed;
bool B1=false;
static double radius[21];
double t;

/* variables */

/* functions */
void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);
bool djudge(float x2, float lastx);
/* functions */

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */
	int j = 0;
	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i< 200; ++i)  _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	while (j < 21) {
		c = getR(_midline[10 * j][0], _midline[10 * j][1], _midline[10 * (j + 1)][0], _midline[10 * (j + 1)][1], _midline[10 * (j + 2)][0], _midline[10 * (j + 2)][1]);
		radius[j] = c.r;
		j++;
	}

	/* you can modify the print code here to show what you want */

}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */



	startPoint = _speed * 0.2; //0.08
	delta = constrain(10, 25, _speed / (fabs(_yaw) + 1) / 7);// 10 25 7
	c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);

	//get the leader's speed and acc
	int i;
	for (i = 0; i<2; i++) { lead_y[i] = lead_y[i + 1]; lead_y[2] = _Leader_Y; }
	Leaderspeed = _speed + (lead_y[1] - lead_y[0]) * 180;
	for (i = 0; i<4; i++) { lead_v[i] = lead_v[i + 1]; lead_v[4] = Leaderspeed; }
	Leaderacc = (lead_v[3] - lead_v[2] + lead_v[1] - lead_v[0]) / (0.020*3.6 * 2);
	thistimedistance = sqrt(_Leader_X*_Leader_X + _Leader_Y * _Leader_Y);

	//speed control
	if (B1)
	{
		if (_Leader_Y < 15)
			expectedspeed = Leaderspeed + 0.02 * Leaderacc + (_Leader_Y - _speed / 100 - 10) / 10;///10
		else
			expectedspeed = Leaderspeed + 0.02 * Leaderacc + _Leader_Y / 5;
	}
	else
	{
		if (_Leader_Y < 12 + _speed / 80)
			expectedspeed = Leaderspeed + 0.02 * Leaderacc + (_Leader_Y - _speed / 100 - 10) / 5;///10
		else
			expectedspeed = Leaderspeed + 0.02 * Leaderacc + _Leader_Y / 5;
	}

	//{
	if (_Leader_Y < 10.5)
		expectedspeed = 0;
	else
		if (thistimedistance >(_speed / 100 + 10) && Leaderacc > 10)
			expectedspeed *= 1.1;
		else
			if (thistimedistance > (_speed / 100 + 11) && Leaderacc > 0)
				expectedspeed *= 1.1;

	kp_s = 0.05;
	ki_s = 0.00001;
	curSpeedErr = expectedspeed - _speed;
	speedErrSum = 0.1 * speedErrSum + curSpeedErr;
	speedErrDiff = curSpeedErr - tmp;
	tmp = curSpeedErr;
	if (Leaderacc < -20 && (_Leader_Y - _speed / 100 - 10) < 0)
	{
		*cmdBrake = 1.0;
		*cmdAcc = 0;
	}
	else
	{
		if (count>=300 && !B1)//其他道路
		{
			
			if (curSpeedErr > 0 && (_Leader_Y - _speed / 90 - 10) > 0 && Leaderacc > -10)//加速控制
			{
				if (_speed <= 80 && Leaderacc>-10) //针对23，31，较稳加速度大些
				{
					*cmdAcc = constrain(0, 1.0, kp_s * curSpeedErr * (840 * (_Leader_Y - 10) / (_speed + 10) + 1) + ki_s * speedErrSum + kd_s * speedErrDiff + Leaderacc / 95 * _Leader_Y);//
					*cmdBrake = 0;

				}
				else if (_speed <= 130 && Leaderacc >= 0)//针对15
				{
					*cmdAcc = constrain(0, 1.0, kp_s * curSpeedErr * (420 * (_Leader_Y - 10) / (_speed + 10) + 1) + ki_s * speedErrSum + kd_s * speedErrDiff + Leaderacc / 95 * _Leader_Y);//
					*cmdBrake = 0;

				}
				else//其他
				{
					*cmdAcc = constrain(0, 1.0, kp_s * curSpeedErr * (100 * (_Leader_Y - 10) / (_speed + 10) + 1) + ki_s * speedErrSum + kd_s * speedErrDiff + Leaderacc / 95 * _Leader_Y);//
					*cmdBrake = 0;

				}
			}
			else//减速控制
			{
				if (_speed <= 80)//针对23，31，低速较稳较稳加速度大些
				{
					if (c.r <= 150)//拐弯时降低减速
					{
						*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 3 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / (Leaderspeed) / 10 / (_Leader_Y - 10));//1.5
						*cmdAcc = 0;

					}
					else//其他
					{
						*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 18 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / 36 / (_Leader_Y - 10));//1.5
						*cmdAcc = 0;
					}
			    }
				else if (_speed <= 130) //针对15，高速较稳
				{
					if (c.r <= 150)
					{
						*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 3 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / (Leaderspeed) / 10 / (_Leader_Y - 10));//1.5
						*cmdAcc = 0;

					}
					else
					{
						*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 18 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / 24 / (_Leader_Y - 10));//1.5
						*cmdAcc = 0;

					}
				}
				else 
				{
					if (c.r <= 150)//拐弯时降低减速
					{
						*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 3 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / (Leaderspeed) / 10 / (_Leader_Y - 10));//1.5
						*cmdAcc = 0;

					}
					else//其他
					{
						*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 12 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / 8 / (_Leader_Y - 10));//1.5
						*cmdAcc = 0;
					}
				}
				

			}
		}
		else//B1
		{
			if (curSpeedErr > 0 && (_Leader_Y - _speed / 100 - 10) > 0 && Leaderacc > -10)
			{
				*cmdAcc = constrain(0, 1.0, kp_s * curSpeedErr * (_Leader_Y / (_speed + 10) + 1) + ki_s * speedErrSum + kd_s * speedErrDiff + Leaderacc / 100 * _Leader_Y);//
				*cmdBrake = 0;
			}
			else if(c.r<=150)
			{
				*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 3 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / 12 / (_Leader_Y - 10));//1.5
				*cmdAcc = 0;
			}
			else
			{
				*cmdBrake = constrain(0, 1.0, -kp_s * curSpeedErr * (_speed / (_Leader_Y) / 3 + 1) - ki_s * speedErrSum - kd_s * speedErrDiff - Leaderacc / 12 / (_Leader_Y - 10));//1.5
				*cmdAcc = 0;

			}
		}

		
	}

	if (++count < 250 && Leaderacc > 10)
		*cmdAcc = 1.0;
	//   derbelow
	kp_d = 1; //0.96
	ki_d = 0;
	kd_d = 0;

	t = 700;
	for (int j = 0; j < startPoint + 1; j++) { if (t >  fabs(radius[j]))  t = fabs(radius[j]); }

	float leaderror = atan2(_Leader_X, _Leader_Y);
	float a5 = atan2(_midline[5][0], _midline[5][1]);
	float x1 = _midline[1][0];

	D_err = -15 * leaderror;

/*	if ((_midline[1][0]) < - (_width/2 - 3) && c.r<=150 && _speed>=150)
		D_err -= 2 * a5;*/

	if (++cnt == interval)
	{
		if (count > interval)
		{
			transY = lastY - distance;
			del_yaw = atan2((_Leader_X - lastX), (_Leader_Y - transY));
		}
		lastX = _Leader_X;
		lastY = _Leader_Y;
		distance = 0;
		cnt = 0;
	}
	else
	{
		distance += _speed * 0.02;
	}

	if (B1)
	{
		D_err = -15 * leaderror;//15
		if (t > 130)//130
			D_err -= 0.7 * del_yaw;//0.7
		kp_d = 0.8;//0.8
	}

	D_errDiff = D_err - Tmp;
	D_errSum = D_errSum + D_err;
	Tmp = D_err;

	*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);

	//protect
	if (fabs(_midline[2][0]) >= _width / 3 && djudge(x1, lastx1))
	{
		*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err * 0.6 + ki_d * D_errSum + kd_d * D_errDiff);
	}
	else if (fabs(_midline[2][0]) >= _width / 2.4 && djudge(x1, lastx1))
	{
		*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err * 0.3 + ki_d * D_errSum + kd_d * D_errDiff);
	}

	if ((_midline[2][0] < (1 - _width) && *cmdSteer > 0) || (_midline[2][0] > (_width - 1) && *cmdSteer < 0))
		*cmdSteer /= 10;

	if (c.r > 130)
	{   
		if (*cmdBrake > 0.95 || *cmdAcc == 1)
		{
			if (count >= 1460 && count <= 1650 && Leaderacc<=-15)//针对15号头车
				*cmdSteer /= 2.5;
			else
			    *cmdSteer /= 2;
		}
		
	}
	else
	{
		if (*cmdBrake > 0.95 || *cmdAcc == 1)
			*cmdSteer /= 1.5;
	}

	

	if (*cmdBrake == 1 && fabs(*cmdSteer) > 0.45)
		*cmdSteer *= -1.001;

	
		 
	lastx1 = x1;

	if (count <= 300)
	{
    speedsum += Leaderspeed;
	averagespeed = speedsum / count;
	}
	if (averagespeed >= 75)
	{
		B1 = true;
	}
	else
	{
		B1 = false;
	}


	
	

	//printf("Leaderspeed %f \n", Leaderspeed);
	//printf("Leaderacc %f \n", Leaderacc);
	//printf("thistimedistance %f \n", thistimedistance);
	//printf("cmdAcc %f \n", *cmdAcc);
	//printf("cmdBrake %f \n", *cmdBrake);
	//printf("leaderror %f \n", leaderror);
	//printf("D_err %f \n", D_err);
	printf("cmdSteer %f \n", *cmdSteer);
	//printf("leadererror %f \n", leaderror);
	//printf("speed %.3f Leader XY(%.3f, %.3f)\n", _speed, _Leader_X, _Leader_Y);
	//printf("expectedspeed %f \n", expectedspeed);
	printf("Leaderspeed %f \n\n", Leaderspeed);
	printf("Leaderacc %f \n\n", Leaderacc);
	//printf("thistimedistance %f \n", thistimedistance);
	printf("cmdAcc %f \n\n", *cmdAcc);
	printf("cmdBrake %f \n\n", *cmdBrake);
	//printf("leaderror %f \n", leaderror);
	//printf("D_err %f \n", D_err);
	printf("cmdSteer %f \n\n", *cmdSteer);
	printf("leadererror %f \n\n", leaderror);
	printf("speed %.3f Leader XY(%.3f, %.3f)\n\n", _speed, _Leader_X, _Leader_Y);
	printf("expectedspeed %f \n\n", expectedspeed);
	printf("c.r %f \n\n", c.r);
	printf("_midline[1][0] %f \n\n", _midline[1][0]);
	printf("count %i \n\n", count);
	printf("averagespeed %f \n\n", averagespeed);

	if (B1)
	{
		if (_speed < 20)
			*cmdGear = 1;
		else
		{
			if (_rpm > 700 && (_speed > 58 * *cmdGear - *cmdGear**cmdGear) && *cmdGear <= 6)    *cmdGear = *cmdGear + 1;
			if ((_speed < 50 * (*cmdGear - 1)) && *cmdGear >= 2)   *cmdGear = *cmdGear - 1;
		}

	}
	else
	{
		updateGear(cmdGear);
	}
}

/**************************************************************************************************************************/
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/**************************************************************************************************************************/

bool djudge(float x1, float lastx)
{
	bool flag;
	if (x1 < 0)
	{
		if ((x1 - lastx1 < 0))
			flag = 1;
		else
			flag = 0;
	}
	if (x1 > 0)
	{
		if ((x1 - lastx1) > 0)
			flag = 1;
		else
			flag = 0;
	}
	return flag;
}


void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 62 && topGear >1)//65
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)//45
		{
			*cmdGear = 1;
		}
		else if (_speed >= 102 && topGear >2)//105
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)//90
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear >3)//145
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 135)//131
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear >4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 183)//173
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear >5)//234
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 227)//219
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b*f - e * c) / (b*d - e * a);
	y = (d*c - a * f) / (b*d - e * a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x>0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}
