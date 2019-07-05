/***************************************************************************

file                 : user3.cpp
author            : Xuangui Huang
email              : stslxg@gmail.com
description    :  user module for CyberParking

***************************************************************************/

/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
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
	printf("OK!\n");
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
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;


circle getR(float x1, float y1, float x2, float y2, float x3, float y3);
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
double thistimedistance;
float transx;
float transy;
float temp;
float expectedspeed;
float rel_X;
float rel_Y;
float relcar_X;
float relcar_Y;
float transcarx;
float transcary;
float absolute_X;
float absolute_Y;
float curSpeedErr;
float speedErrSum;
float speedErrDiff;
const int topGear = 6;
double kp_s = 0.05;	//kp for speed							     //
double ki_s = 0;	//ki for speed							     //
double kd_s = 0;	//kd for speed							     //
float D_err;
double tmp;
double t;
circle c;
int startPoint;
static double radius[21];
int count = 0;
float del_yaw;
float relpx;
float relpy;
float derr2;

double constrain(double lowerBoundary, double upperBoundary, double input);
void updateGear(int *cmdGear);

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */
	int j = 0;
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	while (j < 21) {
		c = getR(_midline[5 * j][0], _midline[5 * j][1], _midline[5 * (j + 1)][0], _midline[5 * (j + 1)][1], _midline[5 * (j + 2)][0], _midline[5 * (j + 2)][1]);
		radius[j] = c.r;
		j++;
	}

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	//printf("lotX %.6f \n lotY %.6f \n",_lotX,_lotY);
}

static int flag = 0;
static int flagout = 0;
static float k, b, dist;


static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	if (abs(_lotAngle)>(PI / 2 - 0.05) && abs(_lotAngle)< (PI / 2 + 0.05))  //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
		dist = abs(_carX - _lotX);
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = abs(k*_carX - _carY + b) / sqrt(k * k + 1);
	}
	thistimedistance = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));
	if (thistimedistance > 100)
		count += 1;
	rel_X = _carX * cos(_lotAngle) + _carY * sin(_lotAngle);
	rel_Y = _carY * cos(_lotAngle) - _carX * sin(_lotAngle);
	transx = _lotX * cos(_lotAngle) + _lotY * sin(_lotAngle);
	transy = _lotY * cos(_lotAngle) - _lotX * sin(_lotAngle);
	rel_X -= transx;
	rel_Y -= transy;
	temp = rel_X;
	rel_X = -rel_Y;
	rel_Y = temp;
	del_yaw = _caryaw - _lotAngle;
	if (_lotAngle < 0)
		del_yaw -= 2 * PI;
	if (_caryaw < 0)
		del_yaw += 2 * PI;

	relcar_X = absolute_X * cos(_caryaw) + absolute_Y * sin(_caryaw);
	relcar_Y = absolute_Y * cos(_caryaw) - absolute_X * sin(_caryaw);
	transcarx = _carX * cos(_caryaw) + _carY * sin(_caryaw);
	transcary = _carY * cos(_caryaw) - _carX * sin(_caryaw);
	relcar_X -= transcarx;
	relcar_Y -= transcary;
	temp = rel_X;
	relcar_X = -relcar_Y;
	relcar_Y = temp;
	startPoint = int(_speed*_speed*0.00015);
	t = 700;
	for (int j = 0; j < startPoint + 1; j++) { if (t >  fabs(radius[j]))  t = fabs(radius[j]); }

	relpx = (_lotX - 2 * cos(_lotAngle)) * cos(_caryaw) + (_lotY - 2 * sin(_lotAngle)) * sin(_caryaw);
	relpy = (_lotY - 2 * sin(_lotAngle)) * cos(_caryaw) - (_lotX - 2 * cos(_lotAngle)) * sin(_caryaw);
	transx = _carX * cos(_caryaw) + _carY * sin(_caryaw);
	transy = _carY * cos(_caryaw) - _carX * sin(_caryaw);
	relpx -= transx;
	relpy -= transy;
	temp = relpx;
	relpx = -relpy;
	relpy = temp;

	derr2 - 0.4 * atan2(relpx, relpy);

	
	//flag
	//状态0：巡线段
	//状态1：即将进入停车区靠左行驶段
	//状态2：进入停车区向右大拐弯段
	//状态3：倒车段进入车库
	//状态4：停车完成，开出车库
	
	if (flag == 0)//状态0：巡线段
	{
		
		if (_speed > 70) { *cmdBrake = 0.15; *cmdAcc = 0; }
		else { *cmdBrake = 0; *cmdAcc = 0.15; }
		*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
		if (thistimedistance < 100)
		{
			flag = 1;
		}
	}
	else
	{
		if (count == 6141)						//3号车位
		{
			switch (flag)
			{
			case 1:
			{
				*cmdSteer = constrain(-1, 1, (_yaw - atan2(_midline[8][0] - _width / 2, _midline[8][1])) / 3.14);

				if (_speed > rel_X * rel_X / 3) { *cmdBrake = 0.6; *cmdGear = 1; *cmdAcc = 0; }
				else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.15; }

				if (rel_X < 10)
				{
					flag = 2;
				}
				break;
			}
			case 2:
			{
				absolute_X = _lotX + 40 * cos(_lotAngle);
				absolute_Y = _lotY + 40 * sin(_lotAngle);
				D_err = -0.5 * atan2(relcar_X, relcar_Y);


				if (rel_X < 4 && fabs(_caryaw - _lotAngle) > 0.17)
				{
					D_err *= 2 * (3 - rel_X / 2);
				}
				if (fabs(_caryaw - _lotAngle) < 0.5 && fabs(rel_X) > 0.4)
				{
					*cmdSteer = constrain(-1, 1, D_err) + rel_X / 2;
				}
				else
				{
					*cmdSteer = constrain(-1, 1, D_err);
				}
				if (fabs(_caryaw - _lotAngle) > 0.27 || rel_X > 0.4)
				{
					if (_speed > 36 - rel_Y) { *cmdBrake = 0.25; *cmdGear = 1; *cmdAcc = 0; }//36
					else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.11; }
				}
				else
				{
					*cmdBrake = 0.85; *cmdGear = 1; *cmdAcc = 0; *cmdSteer = 0;
				}
				if (_speed < 1)
				{
					flag = 3;
				}

				*cmdSteer *= 0.9;
				break;
			}
			case 3:
			{
				absolute_X = _lotX - 3 * cos(_lotAngle);
				absolute_Y = _lotY - 3 * sin(_lotAngle);
				D_err = -0.4 * atan2(relcar_X, relcar_Y);
				if (fabs(rel_X) > 0.3)
				{
					D_err = -0.16 * atan2(relcar_X, relcar_Y) + rel_X / 2.5;
				}

				if ((_caryaw - _lotAngle) < 0.085) D_err /= 3;
				*cmdSteer = constrain(-1, 1, D_err);

				if (rel_Y > 5)
				{
					if (_speed < -rel_Y * 2.5 - 8)
					{
						*cmdBrake = 0.5;
						*cmdGear = -1;
						*cmdAcc = 0;

					}
					else
					{
						*cmdBrake = 0;
						*cmdGear = -1;
						*cmdAcc = 0.65;


					}
				}
				else if (rel_Y > 0.4)
				{
					if (_speed < -rel_Y * 2 - 10)
					{
						*cmdBrake = 0.35;
						*cmdGear = -1;
						*cmdAcc = 0;

					}
					else
					{
						*cmdBrake = 0;
						*cmdGear = -1;
						*cmdAcc = 0.15;
					}
				}
				else
				{
					*cmdBrake = 0.65; *cmdGear = -1; *cmdAcc = 0;//0.6
				}
				if (_speed > -0.15 && rel_Y < 0.05)
				{
					*bFinished = true;
					flag = 4;
				}
				break;
			}
			case 4:
			{
				if (flagout == 0)
				{
					*cmdAcc = 1; *cmdSteer = 1; *cmdBrake = 0;
					if (abs(_yaw) < 0.5)  flagout = 1;
				}
				else if (flagout == 1)
				{
					*cmdAcc = 0.5; *cmdSteer = -1; *cmdBrake = 0;
					if (fabs(_yawrate) < 0.1)  flagout = 2;
				}
				else if (flagout == 2)
				{
					*cmdAcc = 1; *cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
					*cmdBrake = 0;
				}
			}
			}
		}
		else
			if (count == 5736)							//4号车位
			{
				
				
				switch (flag)
				{
				case 1:
				{
					*cmdSteer = constrain(-1, 1, (_yaw - atan2(_midline[8][0] - _width / 2, _midline[8][1])) / 3.14);
					if (_speed > rel_X * rel_X / 4) { *cmdBrake = 0.6; *cmdGear = 1; *cmdAcc = 0; }
					else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.15; }
					if (rel_X < 10)
					{
						flag = 2;
					}
					break;
				}
				case 2:
				{
					absolute_X = _lotX + 40 * cos(_lotAngle);
					absolute_Y = _lotY + 40 * sin(_lotAngle);
					D_err = -0.5 * atan2(relcar_X, relcar_Y);
					if (rel_X < 4 && fabs(_caryaw - _lotAngle) > 0.17)
					{
						D_err *= 2 * (3 - rel_X / 2);
					}
					if (fabs(_caryaw - _lotAngle) < 0.5 && fabs(rel_X) > 0.4)
					{
						*cmdSteer = constrain(-1, 1, D_err) + rel_X / 2;
					}
					else
					{
						*cmdSteer = constrain(-1, 1, D_err);
					}
					if (fabs(_caryaw - _lotAngle) > 0.2 || rel_X > 0.4)
					{
						if (_speed > 36 - rel_Y) { *cmdBrake = 0.25; *cmdGear = 1; *cmdAcc = 0; }
						else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.11; }
					}
					else
					{
						*cmdBrake = 0.85; *cmdGear = 1; *cmdAcc = 0; *cmdSteer = 0;
					}
					if (_speed < 1)
					{
						flag = 3;
					}
					break;
				}
				case 3:
				{
					absolute_X = _lotX - 3 * cos(_lotAngle);
					absolute_Y = _lotY - 3 * sin(_lotAngle);
					D_err = -0.4 * atan2(relcar_X, relcar_Y);
					if (fabs(rel_X) > 0.3)
					{
						D_err = -0.16 * atan2(relcar_X, relcar_Y) + rel_X / 2.5;
					}

					if ((_caryaw - _lotAngle) < 0.085) D_err /= 3;
					*cmdSteer = constrain(-1, 1, D_err);

					if (rel_Y > 5)
					{
						if (_speed < -rel_Y * 2.5 - 8)
						{
							*cmdBrake = 0.52;
							*cmdGear = -1;
							*cmdAcc = 0;

						}
						else
						{
							*cmdBrake = 0;
							*cmdGear = -1;
							*cmdAcc = 0.65;
						}
					}
					else if (rel_Y > 0.4)
					{
						if (_speed < -rel_Y * 2 - 10)
						{
							*cmdBrake = 0.35;
							*cmdGear = -1;
							*cmdAcc = 0;
						}
						else
						{
							*cmdBrake = 0;
							*cmdGear = -1;
							*cmdAcc = 0.15;
						}
					}
					else
					{
						*cmdBrake = 0.6; *cmdGear = -1; *cmdAcc = 0;

					}
					if (_speed > -0.15 && rel_Y < 0.05)
					{
						*bFinished = true;
						flag = 4;
					}
					break;
				}
				case 4:
				{
					if (flagout == 0)
					{
						*cmdAcc = 1; *cmdSteer = 1; *cmdBrake = 0;
						if (abs(_yaw) < 0.5)  flagout = 1;
					}
					else if (flagout == 1)
					{
						*cmdAcc = 0.45; *cmdSteer = -1; *cmdBrake = 0;
						if (fabs(_yawrate) < 0.1)  flagout = 2;
					}
					else if (flagout == 2)
					{
						*cmdAcc = 1; *cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
						*cmdBrake = 0;
					}
				}
				}
			}
			else
				if (count == 5611)								//5号车位
				{
					switch (flag)
					{
					case 1:
					{
						*cmdSteer = constrain(-1, 1, (_yaw - atan2(_midline[8][0] - _width / 2, _midline[8][1])) / 3.14);

						if (_speed > rel_X * rel_X / 3.5) { *cmdBrake = 0.6; *cmdGear = 1; *cmdAcc = 0; }
						else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.15; }
						if (rel_X < 15)
						{
							flag = 2;
						}

						break;
					}
					case 2:
					{
						absolute_X = _lotX + 40 * cos(_lotAngle);
						absolute_Y = _lotY + 40 * sin(_lotAngle);
						D_err = -atan2(relcar_X, relcar_Y);



						if (rel_X < 4 && fabs(_caryaw - _lotAngle) > 0.17)
						{
							D_err *= 2 * (3 - rel_X / 2);
						}
						if (fabs(_caryaw - _lotAngle) < 0.5 && fabs(rel_X) > 0.4)
						{
							*cmdSteer = constrain(-1, 1, D_err) + rel_X / 2;
						}
						else
						{
							*cmdSteer = constrain(-1, 1, 0.757 * D_err + 0.75 * derr2);
						}
						if (fabs(_caryaw - _lotAngle) > 0.27 || rel_X > 0.4)
						{
							if (_speed > 39 - rel_Y) { *cmdBrake = 0.25; *cmdGear = 1; *cmdAcc = 0; }//36
							else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.11; }
						}
						else
						{
							*cmdBrake = 0.85; *cmdGear = 1; *cmdAcc = 0; *cmdSteer = 0;
						}
						if (_speed < 1)
						{
							flag = 3;
						}

						break;
					}
					case 3:
					{
						absolute_X = _lotX - 3 * cos(_lotAngle);
						absolute_Y = _lotY - 3 * sin(_lotAngle);
						D_err = -0.4 * atan2(relcar_X, relcar_Y);
						if (fabs(rel_X) > 0.3)
						{
							D_err = -0.16 * atan2(relcar_X, relcar_Y) + rel_X / 2.5;
						}

						if ((_caryaw - _lotAngle) < 0.085) D_err /= 3;
						*cmdSteer = constrain(-1, 1, D_err);

						if (rel_Y > 5)
						{
							if (_speed < -rel_Y * 2.5 - 8)
							{
								*cmdBrake = 0.52;

								*cmdGear = -1;
								*cmdAcc = 0;

							}
							else
							{
								*cmdBrake = 0;
								*cmdGear = -1;
								*cmdAcc = 0.65;


							}
						}
						else if (rel_Y > 0.4)
						{
							if (_speed < -rel_Y * 2 - 10)
							{
								*cmdBrake = 0.35;
								*cmdGear = -1;
								*cmdAcc = 0;

							}
							else
							{
								*cmdBrake = 0;
								*cmdGear = -1;
								*cmdAcc = 0.15;
							}
						}
						else
						{
							*cmdBrake = 0.6; *cmdGear = -1; *cmdAcc = 0;//0.6

						}
						if (_speed > -0.15 && rel_Y < 0.05)
						{
							*bFinished = true;
							flag = 4;
						}
						break;
					}
					case 4:
					{
						if (flagout == 0)
						{
							*cmdAcc = 1; *cmdSteer = 1; *cmdBrake = 0;
							if (abs(_yaw) < 0.5)  flagout = 1;
						}
						else if (flagout == 1)
						{
							*cmdAcc = 0.55; *cmdSteer = -1; *cmdBrake = 0;
							if (fabs(_yawrate) < 0.1)  flagout = 2;
						}
						else if (flagout == 2)
						{
							*cmdAcc = 1; *cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
							*cmdBrake = 0;
						}
					}
					}
				}
				else
					if (count >= 6508)  //1&2
				{
					switch (flag)
					{
					case 1:
					{
						*cmdSteer = constrain(-1, 1, (_yaw - atan2(_midline[8][0] - _width / 2, _midline[8][1])) / 3.14);

						if (_speed > rel_X * rel_X / 3.5) { *cmdBrake = 0.6; *cmdGear = 1; *cmdAcc = 0; }
						else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.15; }
						if (rel_X < 15)
						{
							flag = 2;
						}

						break;
					}
					case 2:
					{
						absolute_X = _lotX + 40 * cos(_lotAngle);
						absolute_Y = _lotY + 40 * sin(_lotAngle);
						D_err = -atan2(relcar_X, relcar_Y);



						if (rel_X < 4 && fabs(_caryaw - _lotAngle) > 0.17)
						{
							D_err *= 2 * (3 - rel_X / 2);
						}
						if (fabs(_caryaw - _lotAngle) < 0.5 && fabs(rel_X) > 0.4)
						{
							*cmdSteer = constrain(-1, 1, D_err) + rel_X / 2;
						}
						else
						{
							*cmdSteer = constrain(-1, 1, D_err);
						}
						if (fabs(_caryaw - _lotAngle) > 0.27 || rel_X > 0.4)
						{
							if (_speed > 36 - rel_Y) { *cmdBrake = 0.25; *cmdGear = 1; *cmdAcc = 0; }//36
							else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.11; }
						}
						else
						{
							*cmdBrake = 0.85; *cmdGear = 1; *cmdAcc = 0; *cmdSteer = 0;
						}
						if (_speed < 1)
						{
							flag = 3;
						}

						break;
					}
					case 3:
					{
						absolute_X = _lotX - 3 * cos(_lotAngle);
						absolute_Y = _lotY - 3 * sin(_lotAngle);
						D_err = -0.4 * atan2(relcar_X, relcar_Y);
						if (fabs(rel_X) > 0.15)
						{
							D_err = -0.16 * atan2(relcar_X, relcar_Y) + rel_X / 4;
						}
						
						if ((_caryaw - _lotAngle) < 0.085) D_err /= 3;
						*cmdSteer = constrain(-1, 1, D_err);
						
						if (rel_Y > 5)
						{
							if (_speed < -rel_Y * 2.5 - 8)
							{
								*cmdBrake = 0.52;
								*cmdGear = -1;
								*cmdAcc = 0;

							}
							else
							{
								*cmdBrake = 0;
								*cmdGear = -1;
								*cmdAcc = 0.65;

							}
						}
						else if (rel_Y > 0.4)
						{
							if (_speed < -rel_Y * 2 - 10)
							{
								*cmdBrake = 0.35;
								*cmdGear = -1;
								*cmdAcc = 0;

							}
							else
							{
								*cmdBrake = 0;
								*cmdGear = -1;
								*cmdAcc = 0.15;
							}
						}
						else
						{
							*cmdBrake = 0.6; *cmdGear = -1; *cmdAcc = 0;
						}
						if (_speed > -0.15 && rel_Y < 0.05)
						{
							*bFinished = true;
							flag = 4;
						}
						break;
					}
					case 4:
					{
						if (flagout == 0)
						{
							*cmdAcc = 1; *cmdSteer = 1; *cmdBrake = 0;
							if (abs(_yaw) < 0.5)  flagout = 1;
						}
						else if (flagout == 1)
						{
							*cmdAcc = 0.45; *cmdSteer = -1; *cmdBrake = 0;
							if (fabs(_yawrate) < 0.1)  flagout = 2;
						}
						else if (flagout == 2)
						{
							*cmdAcc = 1; *cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
							*cmdBrake = 0;
						}
					}
					}
				}
					else
					{
						switch (flag)
						{
						case 1:
						{
							*cmdSteer = constrain(-1, 1, (_yaw - atan2(_midline[8][0] - _width / 2, _midline[8][1])) / 3.14);
							if ((_speed > rel_X * rel_X / 3.5 && t > 100) || (_speed > rel_X * rel_X / 3 && t <= 100)) { *cmdBrake = 0.6; *cmdGear = 1; *cmdAcc = 0; }
							else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.15; }
							if (rel_X < 15)
							{
								flag = 2;
							}
							break;
						}
						case 2:
						{
							absolute_X = _lotX + 40 * cos(_lotAngle);
							absolute_Y = _lotY + 40 * sin(_lotAngle);
							D_err = -0.5 * atan2(relcar_X, relcar_Y);
							if (rel_X < 4 && fabs(_caryaw - _lotAngle) > 0.17)
							{
								D_err *= 2 * (3 - rel_X / 2);
							}
							if (fabs(_caryaw - _lotAngle) < 0.5 && fabs(rel_X) > 0.4)
							{
								*cmdSteer = constrain(-1, 1, D_err) + rel_X / 2;
							}
							else
							{
								*cmdSteer = constrain(-1, 1, D_err);
							}
							if (fabs(_caryaw - _lotAngle) > 0.27 || rel_X > 0.4)
							{
								if ((_speed > 39 - rel_Y && t > 100 && t <= 300) || (_speed > 36 - rel_Y && t <= 100) || (_speed > 36 - rel_Y && t > 300)) { *cmdBrake = 0.25; *cmdGear = 1; *cmdAcc = 0; }//36
								else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.11; }
							}
							else
							{
								*cmdBrake = 0.85; *cmdGear = 1; *cmdAcc = 0; *cmdSteer = 0;
							}
							if (_speed < 1)
							{
								flag = 3;
							}
							break;
						}
						case 3:
						{
							absolute_X = _lotX - 3 * cos(_lotAngle);
							absolute_Y = _lotY - 3 * sin(_lotAngle);
							D_err = -0.4 * atan2(relcar_X, relcar_Y);
							if (fabs(rel_X) > 0.3)
							{
								D_err = -0.16 * atan2(relcar_X, relcar_Y) + rel_X / 2.5;
							}
							if ((_caryaw - _lotAngle) < 0.085) D_err /= 3;
							*cmdSteer = constrain(-1, 1, D_err);
							if (rel_Y > 5)
							{
								if (_speed < -rel_Y * 2.5 - 8)
								{
									*cmdBrake = 0.52;
									*cmdGear = -1;
									*cmdAcc = 0;

								}
								else
								{
									*cmdBrake = 0;
									*cmdGear = -1;
									*cmdAcc = 0.65;

								}
							}
							else if (rel_Y > 0.4)
							{
								if (_speed < -rel_Y * 2 - 10)
								{
									*cmdBrake = 0.35;
									*cmdGear = -1;
									*cmdAcc = 0;

								}
								else
								{
									*cmdBrake = 0;
									*cmdGear = -1;
									*cmdAcc = 0.15;
								}
							}
							else
							{
								*cmdBrake = 0.6; *cmdGear = -1; *cmdAcc = 0;
							}
							if (_speed > -0.15 && rel_Y < 0.5)
							{
								*bFinished = true;
								flag = 4;
							}
							break;
						}
						case 4:
						{
							if (flagout == 0)
							{
								*cmdAcc = 1; *cmdSteer = 1; *cmdBrake = 0;
								if (abs(_yaw) < 0.5)  flagout = 1;
							}
							else if (flagout == 1)
							{
								*cmdAcc = 0.4; *cmdSteer = -1; *cmdBrake = 0;
								if (fabs(_yawrate) < 1.5)  flagout = 2;
							}
							else if (flagout == 2)
							{
								*cmdAcc = 1; *cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
								*cmdBrake = 0;
							}
						}
						}
					}
	}
	updateGear(cmdGear);
	//printf("count %d \n", count);
	//printf("r %f \n", t);
	//printf("cmdsteer %f \n", *cmdSteer);
	//printf("D_err %f \n", D_err);
	//printf("thistimedistance:%f\n", thistimedistance);
	//printf("Steer:%.2f\tflag:%d\n", *cmdSteer, flag);
	//printf("flagout:%d\n", flagout);
	//printf("cmdAcc:%f\t", *cmdAcc);
	//printf("expectedspeed %f \n", expectedspeed);
	//printf("rel_X %.2f\t", rel_X);
	//printf("rel_Y %.2f\n", rel_Y);
	//printf("flag: %d\n", flag);
	//printf("cmdAcc:%.2f\t", *cmdAcc);
	//printf("cmdbrake %.2f\t", *cmdBrake);
	//printf("del_yaw %f\t", del_yaw);
	//printf("cmdSteer:%f\t\n", *cmdSteer);
	//printf("expectedspeed %.2f \n", expectedspeed);
	//printf("thistimedistance %.2f \n", thistimedistance);
	//printf("fabs(atan2(relcar_X, relcar_Y) %.2f\n", fabs(atan2(relcar_X, relcar_Y)));
	//printf("speed %.2f \n", _speed);
	
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

void updateGear(int *cmdGear)
{
	if (flag != 3)
	{
		if (_gearbox == 1)
		{
			if (_speed >= 65 && topGear >1)
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
			if (_speed <= 45)
			{
				*cmdGear = 1;
			}
			else if (_speed >= 105 && topGear >2)
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
			if (_speed <= 90)
			{
				*cmdGear = 2;
			}
			else if (_speed >= 145 && topGear >3)
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
			if (_speed <= 131)
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
			if (_speed <= 173)
			{
				*cmdGear = 4;
			}
			else if (_speed >= 234 && topGear >5)
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
			if (_speed <= 219)
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
	else
	{
		*cmdGear = -1;
	}

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
	r = constrain(1.0, 5000000000.0, r);
	int sign = (x>0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}