#pragma once 

int walk_plan(int count, struct WalkParam* param);
int standup_plan(int count, struct WalkParam* param);

WalkParam config_motion_param();

const double PI = 3.141592657;
const double body_long = 355.73; //mm x方向
const double body_width = 105;   //mm  z方向
const double body_high = 300;   //    y方向
struct WalkParam
{
	double a;                                  //前向步长
	double b;                                  //步高
	double c;                                  //侧向步长
	double roll;                              //绕身体x轴转角
	double yaw;                                //绕身体y轴转角
	double pitch;                               //绕身体z轴转角
	int    per_step_count;                     //每一步时长
	int    n;                                  //步数
};
