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

	//����ofstream��Ĺ��캯������һ���ļ���������������ļ�
	ofstream fout("plan_test_data.txt");
	if (!fout)
	{
		cout << "�ļ����ܴ�" << endl;
	}
	else
	{
		cout << "�ļ��ܴ�" << endl;
	}
	

	//��̬�滮
	int ret = 1;

	param.a = 120;           //����mm
	param.b = 80;            //����mm
	param.n = 10;             //����
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
			fout << current_xyz[j] << "\t";
		}
		fout << current_body[0] << std::endl;
		
		
	}
	fout.close();
}