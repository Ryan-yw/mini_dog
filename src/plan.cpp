#include<math.h>
#include"plan.h"
#include"kinematics.h"


/****************************��������******************************/

//double P_body_frame[16] =
//{
//
//};

WalkParam config_motion_param()
{
	WalkParam x;
	
	x.a = -100;          //ǰ�󲽳�mm
	x.b = 80;            //����mm
	x.c = -50;            //���Ҳ���
	x.n= 2;             //����
	x.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�


	return x;

}











/****************************��������******************************/



/****************************�켣�滮******************************/
extern double file_current_leg[12];
extern double file_current_body[3];

const double body[3] =
{
	0,body_high,0
};
//const double leg[12] =    //�ȳ�ʼλ�õ�λmm
//{
//	 body_long / 2, 0, -body_width / 2,  //leg1 ->012
//	-body_long / 2, 0, -body_width / 2,  //leg2 ->345
//	-body_long / 2, 0,  body_width / 2,   //leg3 ->678
//	 body_long / 2, 0,  body_width / 2    //leg4 ->91011
//};
const double leg[12] =    //�ȳ�ʼλ�õ�λmm
{
	0
};

void plan_leg(int e_1, int  e_2, double a, double b, double c, double s,int n,double *current_leg)
{

	if (e_1 % 2 == 0)  //ż��13���ȣ�24ͣ
	{
		if (e_1 == 0)   //���ٶ�
		{
			//�滮leg1
			current_leg[0] = leg[0] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[1] = leg[1] + b * sin(PI - s);
			current_leg[2] = leg[2] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;

			//�滮leg2
			current_leg[3] = leg[3] + a * e_2;
			current_leg[4] = leg[4];
			current_leg[5] = leg[5] + c * e_2;
			//�滮leg3
			current_leg[6] = leg[6] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[7] = leg[7] + b * sin(PI - s);
			current_leg[8] = leg[8] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;
			//�滮leg4
			current_leg[9] = leg[9] + a * e_2;
			current_leg[10] = leg[10];
			current_leg[11] = leg[11] + c * e_2;
		}
		else
		{

			//�滮leg1
			current_leg[0] = leg[0] + a * e_2 + a * (1 + cos(PI - s)) / 2.0 - a / 2;
			current_leg[1] = leg[1] + b * sin(PI - s);
			current_leg[2] = leg[2] + c * e_2 + c * (1 + cos(PI - s)) / 2.0 - c / 2;

			//�滮leg2
			current_leg[3] = leg[3] + a * e_2;
			current_leg[4] = leg[4];
			current_leg[5] = leg[5] + c * e_2;
			//�滮leg3
			current_leg[6] = leg[6] + a * e_2 + a * (1 + cos(PI - s)) / 2.0 - a / 2;
			current_leg[7] = leg[7] + b * sin(PI - s);
			current_leg[8] = leg[8] + c * e_2 + c * (1 + cos(PI - s)) / 2.0 - c / 2;
			//�滮leg4
			current_leg[9] = leg[9] + a * e_2;
			current_leg[10] = leg[10];
			current_leg[11] = leg[11] + c * e_2;
		}
	}
	else if (e_1 % 2 == 1)  //����24����13ͣ
	{
		if (e_1 == (2 * n - 1))//���ٶ�
		{
			//�滮leg1
			current_leg[0] = leg[0] + 2 * a * (e_2 + 1) - a;
			current_leg[1] = leg[1];
			current_leg[2] = leg[2] + 2 * c * (e_2 + 1) - c;

			//�滮leg2
			current_leg[3] = leg[3] + 2 * a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[4] = leg[4] + b * sin(PI - s);
			current_leg[5] = leg[5] + 2 * c * e_2 + c * (1 + cos(PI - s)) / 2.0;
			//�滮leg3
			current_leg[6] = leg[6] + 2 * a * (e_2 + 1) - a;
			current_leg[7] = leg[7];
			current_leg[8] = leg[8] + 2 * c * (e_2 + 1) - c;
			//�滮leg4
			current_leg[9] = leg[9] + 2 * a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[10] = leg[10] + b * sin(PI - s);
			current_leg[11] = leg[11] + 2 * c * e_2 + c * (1 + cos(PI - s)) / 2.0;
		}
		else
		{
			//�滮leg1
			current_leg[0] = leg[0] + a * (e_2 + 1) - a / 2;
			current_leg[1] = leg[1];
			current_leg[2] = leg[2] + c * (e_2 + 1) - c / 2;

			//�滮leg2
			current_leg[3] = leg[3] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[4] = leg[4] + b * sin(PI - s);
			current_leg[5] = leg[5] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;
			//�滮leg3
			current_leg[6] = leg[6] + a * (e_2 + 1) - a / 2;
			current_leg[7] = leg[7];
			current_leg[8] = leg[8] + c * (e_2 + 1) - c / 2;
			//�滮leg4
			current_leg[9] = leg[9] + a * e_2 + a * (1 + cos(PI - s)) / 2.0;
			current_leg[10] = leg[10] + b * sin(PI - s);
			current_leg[11] = leg[11] + c * e_2 + c * (1 + cos(PI - s)) / 2.0;
		}
	}
}


int walk_plan(int count, WalkParam* param)
{
	double a = param->a;
	double b = param->b;
	double c = param->c;
	int per_step_count = param->per_step_count;
	int n = param->n;

    static double current_leg[12], current_body[3];

	//����ٶ�����
	int current_count = count % per_step_count;
	double s = 0;
	s = -(PI / 2) * cos(PI * (current_count + 1) / per_step_count) + PI / 2;//s��0��pi
	

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��e1��1
	int e_2 = e_1 / 2;                //�жϵ�ǰ����һ������������e2��1


	
	//��ʽ��ʼ�滮
	if (e_1 == 0)   //���ٶ�
	{
		//�滮��
		plan_leg(e_1, e_2, a / 2, b, c / 2, s, n, current_leg);

		//�滮����
		current_body[0] = body[0] + a * count * count / (4.0 * per_step_count * per_step_count);
		current_body[1] = body[1];
		current_body[2] = body[2] + c * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//���ٶ�
	{
		//�滮��
		plan_leg(e_1, e_2, a / 2, b, c / 2, s, n, current_leg);
		//�滮����
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[0] = body[0] -a * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + a * n - a / 2.0;//n * a 
		current_body[1] = body[1];
		current_body[2] = body[2] - c * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + c * n - c / 2.0;
	}
	else //���ٶ�
	{
		//�滮��
		plan_leg(e_1, e_2, a, b, c, s, n, current_leg);
		//�滮����
		current_body[0] = body[0] + a / 4.0 + a * (count - per_step_count) / per_step_count / 2;//�ٶ�Ϊ100mm/s  ÿ�����per_step_count
		current_body[1] = body[1];
		current_body[2] = body[2] + c / 4.0 + c * (count - per_step_count) / per_step_count / 2;
	}

	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg[j];
	}
	for (int j = 0; j < 3; j++)
	{
		file_current_body[j] = current_body[j];
	}

	inverse(current_leg,current_body);

	return 2 * n * per_step_count - count - 1;
}

/****************************�켣�滮******************************/
