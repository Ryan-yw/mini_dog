#pragma once 

int walk_plan(int count, struct WalkParam* param);

struct WalkParam
{
	double a;                                  //步长
	double b;                                  //步高
	int    per_step_count;                     //每一步时长
	int    n;                                  //步数
};
