#include<math.h>
#include<cstring>

#include"plan.h"
#include"kinematics.h"

extern double time1;
/****************************参数配置******************************/


WalkParam config_motion_param()
{
	WalkParam x;
	
	x.a = -100;          //前后步长mm
	x.b = 80;            //步高mm
	x.c = -50;            //左右步长
	x.n= 2;             //步数
	x.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，


	return x;

}











/****************************参数配置******************************/



/****************************轨迹规划******************************/
extern double file_current_leg[12];
extern double file_current_body[16];
const double body0[16] =
{
	1, 0, 0, 0,
	0, 1, 0, 93,
	0, 0, 1, 0,
	0, 0, 0, 1
};
const double body[16] =
{
	1, 0, 0, 0,
	0, 1, 0, body_high,
	0, 0, 1, 0,
	0, 0, 0, 1
};
const double leg[12] =    //腿初始位置单位mm
{
	 body_long / 2, 0, -body_width / 2,  //leg1 ->012
	-body_long / 2, 0, -body_width / 2,  //leg2 ->345
	-body_long / 2, 0,  body_width / 2,   //leg3 ->678
	 body_long / 2, 0,  body_width / 2    //leg4 ->91011
};
//const double leg[12] =    //腿初始位置单位mm
//{
//	0
//};

void plan_leg(int e_1, int  e_2, double a, double b, double c, double s,int n,double *current_leg,double *R_y)
{
	if (e_1 % 2 == 0)  //偶数13迈腿，24停
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_leg[0] = leg[0] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[1] = leg[1] + b * sin(PI - s);
			current_leg[2] = leg[2] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;

			//规划leg2
			current_leg[3] = leg[3] + a * e_2;
			current_leg[4] = leg[4];
			current_leg[5] = leg[5] + c * e_2;
			//规划leg3
			current_leg[6] = leg[6] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[7] = leg[7] + b * sin(PI - s);
			current_leg[8] = leg[8] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;
			//规划leg4
			current_leg[9] = leg[9] + a * e_2;
			current_leg[10] = leg[10];
			current_leg[11] = leg[11] + c * e_2;
		}
		else
		{

			//规划leg1
			current_leg[0] = leg[0] + a * e_2 + a * (1 + cos(PI - s)) / 2.0 - a / 2;
			current_leg[1] = leg[1] + b * sin(PI - s);
			current_leg[2] = leg[2] + c * e_2 + c * (1 + cos(PI - s)) / 2.0 - c / 2;

			//规划leg2
			current_leg[3] = leg[3] + a * e_2;
			current_leg[4] = leg[4];
			current_leg[5] = leg[5] + c * e_2;
			//规划leg3
			current_leg[6] = leg[6] + a * e_2 + a * (1 + cos(PI - s)) / 2.0 - a / 2;
			current_leg[7] = leg[7] + b * sin(PI - s);
			current_leg[8] = leg[8] + c * e_2 + c * (1 + cos(PI - s)) / 2.0 - c / 2;
			//规划leg4
			current_leg[9] = leg[9] + a * e_2;
			current_leg[10] = leg[10];
			current_leg[11] = leg[11] + c * e_2;
		}


		double temp1[3] = { 0 };
		
		temp1[0] = R_y[0] * current_leg[0] + R_y[1] * current_leg[1] + R_y[2] * current_leg[2];
		temp1[1] = R_y[3] * current_leg[0] + R_y[4] * current_leg[1] + R_y[5] * current_leg[2];
		temp1[2] = R_y[6] * current_leg[0] + R_y[7] * current_leg[1] + R_y[8] * current_leg[2];
		memcpy(current_leg + 0 * 3, temp1, sizeof(double) * 3);
		
		double temp3[3] = { 0 };

		temp3[0] = R_y[0] * current_leg[6] + R_y[1] * current_leg[7] + R_y[2] * current_leg[8];
		temp3[1] = R_y[3] * current_leg[6] + R_y[4] * current_leg[7] + R_y[5] * current_leg[8];
		temp3[2] = R_y[6] * current_leg[6] + R_y[7] * current_leg[7] + R_y[8] * current_leg[8];
		memcpy(current_leg + 2 * 3, temp3, sizeof(double) * 3);
	}
	else if (e_1 % 2 == 1)  //奇数24迈腿13停
	{
		if (e_1 == (2 * n - 1))//减速段
		{
			//规划leg1
			current_leg[0] = leg[0] + 2 * a * (e_2 + 1) - a;
			current_leg[1] = leg[1];
			current_leg[2] = leg[2] + 2 * c * (e_2 + 1) - c;

			//规划leg2
			current_leg[3] = leg[3] + 2 * a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[4] = leg[4] + b * sin(PI - s);
			current_leg[5] = leg[5] + 2 * c * e_2 + c * (1 + cos(PI - s)) / 2.0;
			//规划leg3
			current_leg[6] = leg[6] + 2 * a * (e_2 + 1) - a;
			current_leg[7] = leg[7];
			current_leg[8] = leg[8] + 2 * c * (e_2 + 1) - c;
			//规划leg4
			current_leg[9] = leg[9] + 2 * a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[10] = leg[10] + b * sin(PI - s);
			current_leg[11] = leg[11] + 2 * c * e_2 + c * (1 + cos(PI - s)) / 2.0;
		}
		else
		{
			//规划leg1
			current_leg[0] = leg[0] + a * (e_2 + 1) - a / 2;
			current_leg[1] = leg[1];
			current_leg[2] = leg[2] + c * (e_2 + 1) - c / 2;

			//规划leg2
			current_leg[3] = leg[3] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[4] = leg[4] + b * sin(PI - s);
			current_leg[5] = leg[5] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;
			//规划leg3
			current_leg[6] = leg[6] + a * (e_2 + 1) - a / 2;
			current_leg[7] = leg[7];
			current_leg[8] = leg[8] + c * (e_2 + 1) - c / 2;
			//规划leg4
			current_leg[9] = leg[9] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[10] = leg[10] + b * sin(PI - s);
			current_leg[11] = leg[11] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;
		}

		double temp2[3] = { 0 };

		temp2[0] = R_y[0] * current_leg[3] + R_y[1] * current_leg[4] + R_y[2] * current_leg[5];
		temp2[1] = R_y[3] * current_leg[3] + R_y[4] * current_leg[4] + R_y[5] * current_leg[5];
		temp2[2] = R_y[6] * current_leg[3] + R_y[7] * current_leg[4] + R_y[8] * current_leg[5];
		memcpy(current_leg + 1 * 3, temp2, sizeof(double) * 3);

		double temp4[3] = { 0 };

		temp4[0] = R_y[0] * current_leg[9] + R_y[1] * current_leg[10] + R_y[2] * current_leg[11];
		temp4[1] = R_y[3] * current_leg[9] + R_y[4] * current_leg[10] + R_y[5] * current_leg[11];
		temp4[2] = R_y[6] * current_leg[9] + R_y[7] * current_leg[10] + R_y[8] * current_leg[11];
		memcpy(current_leg + 3 * 3, temp4, sizeof(double) * 3);
	}
}


double* rotation_matrix_in_body( double* rm, const double* body_init, double* r_m_out)//3*3 4*4->4*4
{
	r_m_out[0] = rm[0] * body_init[0] + rm[1] * body_init[4] + rm[2] * body_init[8];
	r_m_out[1] = rm[0] * body_init[1] + rm[1] * body_init[5] + rm[2] * body_init[9];
	r_m_out[2] = rm[0] * body_init[2] + rm[1] * body_init[6] + rm[2] * body_init[10];
	r_m_out[3] = body_init[3];

	r_m_out[4] = rm[3] * body_init[0] + rm[4] * body_init[4] + rm[5] * body_init[8];
	r_m_out[5] = rm[3] * body_init[1] + rm[4] * body_init[5] + rm[5] * body_init[9];
	r_m_out[6] = rm[3] * body_init[2] + rm[4] * body_init[6] + rm[5] * body_init[10];
	r_m_out[7] = body_init[7];

	r_m_out[8] = rm[6] * body_init[0] + rm[7] * body_init[4] + rm[8] * body_init[8];
	r_m_out[9] = rm[6] * body_init[1] + rm[7] * body_init[5] + rm[8] * body_init[9];
	r_m_out[10] = rm[6] * body_init[2] + rm[7] * body_init[6] + rm[8] * body_init[10];
	r_m_out[11] = body_init[11];

	r_m_out[12] = 0;
	r_m_out[13] = 0;
	r_m_out[14] = 0;
	r_m_out[15] = 1;

	return r_m_out;
}


int walk_plan(int count, WalkParam* param)
{
	double a = param->a;
	double b = param->b;
	double c = param->c;
	double pitch = param->pitch;
	double roll = param->roll;
	double yaw = param->yaw;
	int per_step_count = param->per_step_count;
	int n = param->n;


	static double current_leg[12] = { 0 };
	static double current_body[16] = { 1,0,0,0,
									   0,1,0,0,
									   0,0,1,0,
									   0,0,0,1};



	//求加速度曲线
	int current_count = count % per_step_count;
	double s = 0;
	s = -(PI / 2) * cos(PI * (current_count + 1) / per_step_count) + PI / 2;//s从0到pi
	pitch = s * pitch / 180;
	roll = s * roll / 180;
	yaw = s * yaw / 180;

	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1
	int e_2 = e_1 / 2;                //判断当前在哪一步，腿走两步e2加1

	double R_x[9] = {
				1, 0,          0,
				0, cos(roll), -sin(roll),
				0, sin(roll),  cos(roll),
	};
	double R_y[9] = {
						cos(yaw), 0, sin(yaw),
							0,    1,    0,
						-sin(yaw), 0, cos(yaw),
	};
	double R_z[9] = {
						cos(pitch), -sin(pitch),0,
						sin(pitch),  cos(pitch),0,
						0 ,          0,         1,
	};

	//正式开始规划
	if (e_1 == 0)   //加速段
	{
		//规划腿
		plan_leg(e_1, e_2, a / 2, b, c / 2, s, n, current_leg, R_y);

		//规划身体
		current_body[3] = a * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body[7];
		current_body[11] = c * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//减速段
	{
		//规划腿
		plan_leg(e_1, e_2, a / 2, b, c / 2, s, n, current_leg, R_y);
		//规划身体
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = -a * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + a * n - a / 2.0;//n * a 
		current_body[7] = body[7];
		current_body[11] = -c * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + c * n - c / 2.0;
	}
	else //匀速段
	{
		//规划腿
		plan_leg(e_1, e_2, a, b, c, s, n, current_leg, R_y);
		//规划身体
		current_body[3] = a / 4.0 + a * (count - per_step_count) / per_step_count / 2;//速度为100mm/s  每秒计数per_step_count
		current_body[7] = body[7];
		current_body[11] = c / 4.0 + c * (count - per_step_count) / per_step_count / 2;
	}



	{
		double tempx[16] = { 0 };
		rotation_matrix_in_body(R_x, current_body, tempx);
		double tempy[16] = { 0 };
		rotation_matrix_in_body(R_y, tempx, tempy);
		double tempz[16] = { 0 };
		rotation_matrix_in_body(R_z, tempy, tempz);
		memcpy(current_body, tempz, sizeof(double) * 16);
	}


	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg[j];
	}
	for (int j = 0; j < 16; j++)
	{
		file_current_body[j] = current_body[j];
	}

	current_body[3] = -current_body[3];
	current_body[7] = -current_body[7];
	current_body[11] = -current_body[11];
	inverse(current_leg,current_body);

	time1 = time1 + 0.1;
	return 2 * n * per_step_count - count - 1;
}


int walk_plan_standup(int count, WalkParam* param)
{
	double a = param->a;
	double b = param->b;
	double c = param->c;
	double pitch = param->pitch;
	double roll = param->roll;
	double yaw = param->yaw;
	int per_step_count = param->per_step_count;
	int n = param->n;


	static double current_leg[12] = { 0 };
	static double current_body[16] = { 1,0,0,0,
									   0,1,0,0,
									   0,0,1,0,
									   0,0,0,1 };



	//求加速度曲线
	int current_count = count % per_step_count;
	double s = 0;
	s = -(PI / 2) * cos(PI * (current_count + 1) / per_step_count) + PI / 2;//s从0到pi
	pitch = s * pitch / 180;
	roll = s * roll / 180;
	yaw = s * yaw / 180;

	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1
	int e_2 = e_1 / 2;                //判断当前在哪一步，腿走两步e2加1

	double R_x[9] = {
				1, 0,          0,
				0, cos(roll), -sin(roll),
				0, sin(roll),  cos(roll),
	};
	double R_y[9] = {
						cos(yaw), 0, sin(yaw),
							0,    1,    0,
						-sin(yaw), 0, cos(yaw),
	};
	double R_z[9] = {
						cos(pitch), -sin(pitch),0,
						sin(pitch),  cos(pitch),0,
						0 ,          0,         1,
	};

	//正式开始规划
	if (e_1 == 0)   //加速段
	{
		//规划腿
		plan_leg(e_1, e_2, a / 2, b, c / 2, s, n, current_leg, R_y);

		//规划身体
		current_body[3] = a * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body0[7] + (body_high - body0[7]) * s / PI;
		current_body[11] = c * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//减速段
	{
		//规划腿
		plan_leg(e_1, e_2, a / 2, b, c / 2, s, n, current_leg, R_y);
		//规划身体
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = -a * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + a * n - a / 2.0;//n * a 
		current_body[7] = body0[7] + (body_high - body0[7]) * s / PI;
		current_body[11] = -c * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + c * n - c / 2.0;
	}
	else //匀速段
	{
		//规划腿
		plan_leg(e_1, e_2, a, b, c, s, n, current_leg, R_y);
		//规划身体
		current_body[3] = a / 4.0 + a * (count - per_step_count) / per_step_count / 2;//速度为100mm/s  每秒计数per_step_count
		current_body[7] = body0[7] + (body_high - body0[7]) * s / PI;
		current_body[11] = c / 4.0 + c * (count - per_step_count) / per_step_count / 2;
	}



	{
		double tempx[16] = { 0 };
		rotation_matrix_in_body(R_x, current_body, tempx);
		double tempy[16] = { 0 };
		rotation_matrix_in_body(R_y, tempx, tempy);
		double tempz[16] = { 0 };
		rotation_matrix_in_body(R_z, tempy, tempz);
		memcpy(current_body, tempz, sizeof(double) * 16);
	}


	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg[j];
	}
	for (int j = 0; j < 16; j++)
	{
		file_current_body[j] = current_body[j];
	}

	current_body[3] = -current_body[3];
	current_body[7] = -current_body[7];
	current_body[11] = -current_body[11];
	inverse(current_leg, current_body);

	time1 = time1 + 0.1;
	return  n * per_step_count - count - 1;
}

/****************************轨迹规划******************************/
