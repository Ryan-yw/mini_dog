#pragma once 

int walk_plan(int count, struct WalkParam* param);

struct WalkParam
{
	double a;                                  //����
	double b;                                  //����
	int    per_step_count;                     //ÿһ��ʱ��
	int    n;                                  //����
};
