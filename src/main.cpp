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

	//利用ofstream类的构造函数创建一个文件输出流对象来打开文件
	ofstream fout("plan_data_in_ground.txt");
	ofstream angle("input_angle.txt");
	ofstream xyz_leg("xyz_in_leg.txt");
	if (!angle)
	{
		std::cout << "angle文件不能打开" << std::endl;
	}
	else
	{
		std::cout << "angle文件能打开" << std::endl;
	}
	if (!fout)
	{
		std::cout << "xyz文件不能打开" << std::endl;
	}
	else
	{
		std::cout << "xyz文件能打开" << std::endl;
	}
	

	//步态规划
	int ret = 1;

	param.a = 100;           //步长mm
	param.b = 80;            //步高mm
	param.n = 5;             //步数
	param.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，

	//fout << "x1" << "\t" << "y1" << "\t" << "z1" << "\t";//输出文件抬头
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