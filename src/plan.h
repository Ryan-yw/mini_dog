#pragma once 

int walk_plan(int count, struct WalkParam* param);

const double PI = 3.141592657;
const double body_long = 355.73; //mm x����
const double body_width = 105;   //mm  z����
const double body_high = 300;   //    y����
struct WalkParam
{
	double a;                                  //����
	double b;                                  //����
	int    per_step_count;                     //ÿһ��ʱ��
	int    n;                                  //����
};
