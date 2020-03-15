#pragma once 

int walk_plan(int count, struct WalkParam* param);

const double PI = 3.141592657;
const double body_long = 355.73; //mm x方向
const double body_width = 105;   //mm  z方向
const double body_high = 300;   //    y方向
struct WalkParam
{
	double a;                                  //步长
	double b;                                  //步高
	int    per_step_count;                     //每一步时长
	int    n;                                  //步数
};
