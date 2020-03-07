#include<math.h>
#include"plan.h"


const double PI = 3.141592657;
extern double current_xyz[12];
extern double current_body[3];
const double body[3] =
{
	0,0,300
};
//const double leg[12] =    //腿初始位置单位mm
//{
//	 150,0,-125,  //leg1 ->012
//	-150,0,-125,  //leg2 ->345
//	-150,0,125,   //leg3 ->678
//	 150,0,125    //leg4 ->91011
//};
const double leg[12] =    //腿初始位置单位mm
{
	0
};



void plan_leg(int e_1, int  e_2, double a, double b, double s,int n)
{
	if (e_1 % 2 == 0)  //偶数13迈腿，24停
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_xyz[0] = leg[0] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_xyz[1] = b * sin(PI - s);
			current_xyz[2] = leg[2];

			//规划leg2
			current_xyz[3] = leg[3] + a * e_2;
			current_xyz[4] = 0;
			current_xyz[5] = leg[5];
			//规划leg3
			current_xyz[6] = leg[6] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_xyz[7] = b * sin(PI - s);
			current_xyz[8] = leg[8];
			//规划leg4
			current_xyz[9] = leg[9] + a * e_2;
			current_xyz[10] = 0;
			current_xyz[11] = leg[11];
		}
		else
		{

			//规划leg1
			current_xyz[0] = leg[0] + a * e_2 + a * (1 + cos(PI - s)) / 2.0 - a / 2;
			current_xyz[1] = b * sin(PI - s);
			current_xyz[2] = leg[2];

			//规划leg2
			current_xyz[3] = leg[3] + a * e_2;
			current_xyz[4] = 0;
			current_xyz[5] = leg[5];
			//规划leg3
			current_xyz[6] = leg[6] + a * e_2 + a * (1 + cos(PI - s)) / 2.0 - a / 2;
			current_xyz[7] = b * sin(PI - s);
			current_xyz[8] = leg[8];
			//规划leg4
			current_xyz[9] = leg[9] + a * e_2;
			current_xyz[10] = 0;
			current_xyz[11] = leg[11];
		}
	}
	else if (e_1 % 2 == 1)  //奇数24迈腿13停
	{
		if (e_1 == (2 * n - 1))//减速段
		{
			//规划leg1
			current_xyz[0] = leg[0] + 2 * a * (e_2 + 1) - a;
			current_xyz[1] = 0;
			current_xyz[2] = leg[2];

			//规划leg2
			current_xyz[3] = leg[3] + 2 * a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_xyz[4] = b * sin(PI - s);
			current_xyz[5] = leg[5];
			//规划leg3
			current_xyz[6] = leg[6] + 2 * a * (e_2 + 1) - a;
			current_xyz[7] = 0;
			current_xyz[8] = leg[8];
			//规划leg4
			current_xyz[9] = leg[9] + 2 * a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_xyz[10] = b * sin(PI - s);
			current_xyz[11] = leg[11];
		}
		else
		{
			//规划leg1
			current_xyz[0] = leg[0] + a * (e_2 + 1) - a / 2;
			current_xyz[1] = 0;
			current_xyz[2] = leg[2];

			//规划leg2
			current_xyz[3] = leg[3] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_xyz[4] = b * sin(PI - s);
			current_xyz[5] = leg[5];
			//规划leg3
			current_xyz[6] = leg[6] + a * (e_2 + 1) - a / 2;
			current_xyz[7] = 0;
			current_xyz[8] = leg[8];
			//规划leg4
			current_xyz[9] = leg[9] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_xyz[10] = b * sin(PI - s);
			current_xyz[11] = leg[11];
		}
	}
}


int walk_plan(int count, struct WalkParam* param)
{
	double a = param->a;
	double b = param->b;
	int per_step_count = param->per_step_count;
	int n = param->n;


	//求加速度曲线
	int current_count = count % per_step_count;
	double s = 0;
	s = -(PI / 2) * cos(PI * (current_count + 1) / per_step_count) + PI / 2;//s从0到pi
	

	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1
	int e_2 = e_1 / 2;                //判断当前在哪一步，腿走两步e2加1
	
	//正式开始规划
	if (e_1 == 0)   //加速段
	{
		//规划腿
		plan_leg(e_1, e_2, a / 2, b, s, n);

		//规划身体
		current_body[0] = body[0] + a * count * count / (4.0 * per_step_count * per_step_count);
		current_body[1] = body[1];
		current_body[2] = body[2];
	}
	else if (e_1 == (2 * n - 1))//减速段
	{
		//规划腿
		plan_leg(e_1, e_2, a / 2, b, s, n);
		//规划身体
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[0] = body[0] -a * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + a * n - a / 2.0;//n * a 
		current_body[1] = body[1];
		current_body[2] = body[2];
	}
	else //匀速段
	{
		//规划腿
		plan_leg(e_1, e_2, a, b, s, n);
		//规划身体
		current_body[0] = body[0] + a / 4.0 + a * (count - per_step_count) / per_step_count / 2;//速度为100mm/s  每秒计数per_step_count
		current_body[1] = body[1];
		current_body[2] = body[2];
	}

	return 2 * n * per_step_count - count - 1;
}
