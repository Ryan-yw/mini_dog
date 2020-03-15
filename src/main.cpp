#include<iostream>
#include<cmath>
#include<fstream>
#include"plan.h"
#include"kinematics.h"

using namespace std;

double file_leg_xyz[12] = { 0 };
double file_current_leg[12] = { 0 };
double file_current_body[3] = { 0 };

double input_angle[12] = { 0 };
WalkParam param;


void main()
{

	//����ofstream��Ĺ��캯������һ���ļ���������������ļ�
	ofstream fout("plan_data_in_ground.txt");
	ofstream angle("input_angle.txt");
	ofstream xyz_leg("xyz_in_leg.txt");
	if (!angle)
	{
		std::cout << "angle�ļ����ܴ�" << std::endl;
	}
	else
	{
		std::cout << "angle�ļ��ܴ�" << std::endl;
	}
	if (!fout)
	{
		std::cout << "xyz�ļ����ܴ�" << std::endl;
	}
	else
	{
		std::cout << "xyz�ļ��ܴ�" << std::endl;
	}
	

	//��̬�滮
	int ret = 1;

	param.a = 100;           //����mm
	param.b = 80;            //����mm
	param.n = 5;             //����
	param.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�

	//fout << "x1" << "\t" << "y1" << "\t" << "z1" << "\t";//����ļ�̧ͷ
	//fout << "x2" << "\t" << "y2" << "\t" << "z2" << "\t";
	//fout << "x3" << "\t" << "y3" << "\t" << "z3" << "\t";
	//fout << "x4" << "\t" << "y4" << "\t" << "z4" << "\t";
	//fout << "body_x" << "\t" << endl;
	for (int i = 0; ret; ++i)
	{
		ret = walk_plan(i, &param);


		for (int j = 0; j < 12; j++)
		{
			fout << file_current_leg[j] << "\t";
		}
		fout << file_current_body[0] << std::endl;

		for (int j = 0; j < 12; j++)
		{
			xyz_leg << file_leg_xyz[j] << "\t" << "\t";
			
		}
		xyz_leg << std::endl;

		for (int j = 0; j < 12; j++)
		{
			angle << input_angle[j] << "\t";

		}
		angle << std::endl;
		
	}
	fout.close();
	angle.close();
	xyz_leg.close();
}