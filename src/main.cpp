#include<iostream>
#include<cmath>
#include<fstream>
#include<string>


#include"plan.h"
#include"kinematics.h"

using namespace std;

double file_leg_xyz[12] = { 0 };
double file_current_leg[12] = { 0 };
double file_current_body[3] = { 0 };

double input_angle[12] = { 0 };
WalkParam param;
ofstream fout("plan_data_in_ground.txt");
ofstream angle("input_angle.txt");
ofstream xyz_leg("xyz_in_leg.txt");

void execute(WalkParam & x)
{
	//fout << "x1" << "\t" << "y1" << "\t" << "z1" << "\t";//输出文件抬头
	//fout << "x2" << "\t" << "y2" << "\t" << "z2" << "\t";
	//fout << "x3" << "\t" << "y3" << "\t" << "z3" << "\t";
	//fout << "x4" << "\t" << "y4" << "\t" << "z4" << "\t";
	//fout << "body_x" << "\t" << endl;



	int ret = 1;
	for (int i = 0; ret; ++i)
	{
		param = config_motion_param();//行走参数配置
		ret = walk_plan(i, &x);  //轨迹规划

		{
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

	}
	fout.close();
	angle.close();
	xyz_leg.close();

}

void main()
{

	//利用ofstream类的构造函数创建一个文件输出流对象来打开文件

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

	execute(param);
}