#include<iostream>
#include<cmath>
#include<fstream>
#include"plan.h"

using namespace std;
double current_xyz[12] ={0};
double current_body[3];
WalkParam param;


void main()
{

	//利用ofstream类的构造函数创建一个文件输出流对象来打开文件
	ofstream fout("plan_test_data.txt");
	if (!fout)
	{
		cout << "文件不能打开" << endl;
	}
	else
	{
		cout << "文件能打开" << endl;
	}
	

	//步态规划
	int ret = 1;

	param.a = 120;           //步长mm
	param.b = 80;            //步高mm
	param.n = 10;             //步数
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
			fout << current_xyz[j] << "\t";
		}
		fout << current_body[0] << std::endl;
		
		
	}
	fout.close();
}