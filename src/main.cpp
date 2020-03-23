#include<iostream>
#include<cmath>
#include<fstream>
#include<string>

#include <chrono>
#include <thread>
#include <algorithm>


#include <aris.hpp>
#include "json.hpp"


#include"plan.h"
#include"kinematics.h"

using namespace std;

double file_leg_xyz[12] = { 0 };
double file_current_leg[12] = { 0 };
double file_current_body[16] = { 0 };

double input_angle[12] = { 0 };
WalkParam param;
//ofstream fout("plan_data_in_ground.txt",ios::out);
//ofstream angle("input_angle.txt",ios::out);
//ofstream xyz_leg("xyz_in_leg.txt",ios::out);








///**************��ҳ�������******************/
int command = 0, state = 0;
int step = 0;
std::string str_command;
enum  //״̬����
{
	INIT = 0,		//��ʼ
	HOMED = 1,		//��ԭ��
	HOMEFINISH = 2, //��ԭ�����
	PREPAIRD = 3,	 //׼��
	RUNNING = 4,		//����
	ERRO = 5,		 //����
};

enum  //�����
{
	HOME = 0,      //��ԭ��
	PREPAIR = 1,   //׼��
	FORWARD = 2,   //ǰ��
	BACK = 3,      //����
	LEFT = 4,      //����
	RIGHT = 5,      //����
	TURNL = 6,      //����
	TURNR = 7		//����
};
int table_state[6][8] =       //״̬��
{
	//       home    prepair  forward    back      lest     right   turnl    turnr
	 {      HOMED,        -1,      -1,      -1,      -1,      -1,      -1,      -1},		//init0
	 { HOMEFINISH,        -1,      -1,      -1,      -1,      -1,      -1,      -1},		//homed1
	 { HOMEFINISH,  PREPAIRD,      -1,      -1,      -1,      -1,      -1,      -1},		//homefinish2
	 {         -1,  PREPAIRD, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING},		//prepaird3
	 {         -1,        -1, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING},		//running4
	 {       ERRO,      ERRO,    ERRO,    ERRO,    ERRO,    ERRO,    ERRO, ERRO}		   //error5
};



auto current_state(std::string& cmd) ->int
{
	std::map<std::string, int> int2str =
	{
		std::make_pair(std::string("home"),    HOME),
		std::make_pair(std::string("prepair"), PREPAIR),
		std::make_pair(std::string("forward"), FORWARD),
		std::make_pair(std::string("back"),    BACK),
		std::make_pair(std::string("left"),    LEFT),
		std::make_pair(std::string("right"),   RIGHT),
		std::make_pair(std::string("turnl"),   TURNL),
		std::make_pair(std::string("turnr"),   TURNR),
	};
	command = int2str.at(cmd);
	return table_state[state][command];
}


auto StringParser(std::string& cmd)->std::string
{
	std::string str1;

	int len = cmd.size();  //cmdΪ��ҳ�����ַ���,�����ַ�����)
	if (len < 8)
	{
		str1 = cmd;  //str_commandΪ��ȡ�����ַ���
		step = 0;
	}
	else if ((cmd[len - 1] >= 48 && cmd[len - 1] <= 57) && (cmd[len - 2] < 48 || cmd[len - 2] > 57))
	{
		str1.assign(cmd, 0, len - 6);  //str_commandΪ��ȡ�����ַ���
		step = cmd[len - 1];     //aΪ����
		step = step - 48;
	}
	else if ((cmd[len - 1] >= 48 && cmd[len - 1] <= 57) && (cmd[len - 2] >= 48 || cmd[len - 2] <= 57))
	{
		str1.assign(cmd, 0, len - 7);  //str_commandΪ��ȡ�����ַ���
		step = (cmd[len - 1] - 48) + (cmd[len - 2] - 48) * 10;     //aΪ����

	}
	return str1;
}

///**************��ҳ�������******************/


//void execute(WalkParam & x)
//{
//	//fout << "x1" << "\t" << "y1" << "\t" << "z1" << "\t";//����ļ�̧ͷ
//	//fout << "x2" << "\t" << "y2" << "\t" << "z2" << "\t";
//	//fout << "x3" << "\t" << "y3" << "\t" << "z3" << "\t";
//	//fout << "x4" << "\t" << "y4" << "\t" << "z4" << "\t";
//	//fout << "body_x" << "\t" << endl;
//
//
//
//
//
//}



auto execute(int command, int state)->std::tuple<int, std::string>
{

	int n = step;
	std::cout << n << std::endl;

	switch (command)
	{

		case HOME:
		{
			if (state == HOMED || state == HOMEFINISH)
			{


				std::cout << "home finished" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			return std::make_tuple<int, std::string>(0, "home finished");
			break;
		}
		case PREPAIR:
		{
			if (state == HOMEFINISH || state == PREPAIRD)
			{

				std::cout << "prepair finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "prepair finish");
			break;
		}
		case FORWARD:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				ofstream fout("plan_data_in_ground.txt", ios::out);
				ofstream angle("input_angle.txt", ios::out);
				ofstream xyz_leg("xyz_in_leg.txt", ios::out);
				param.a = 100;          //ǰ�󲽳�mm
				param.b = 80;            //����mm
				param.c = 0;            //���Ҳ���
				param.pitch = 0;        //��λΪ��
				param.roll = 0;        
				param.yaw = 0;
				param.n = n;             //����
				param.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//���߲�������
					ret = walk_plan(i, &param);  //�켣�滮

					{
						for (int j = 0; j < 12; j++)
						{
							fout << file_current_leg[j] << "\t";
						}
						fout << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

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
				{
					fout.close();
					angle.close();
					xyz_leg.close();
				}
				std::cout << "forward finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "forward finish");
			break;
		}
		case BACK:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				ofstream fout("plan_data_in_ground.txt", ios::out);
				ofstream angle("input_angle.txt", ios::out);
				ofstream xyz_leg("xyz_in_leg.txt", ios::out);
				param.a = -100;          //ǰ�󲽳�mm
				param.b = 80;            //����mm
				param.c = 0;            //���Ҳ���
				param.pitch = 0;       //��λΪ��
				param.roll = 0;
				param.yaw = 0;
				param.n = n;             //����
				param.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//���߲�������
					ret = walk_plan(i, &param);  //�켣�滮

					{
						for (int j = 0; j < 12; j++)
						{
							fout << file_current_leg[j] << "\t";
						}
						fout << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

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
				{
					fout.close();
					angle.close();
					xyz_leg.close();
				}
				std::cout << "back finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "back finish");
			break;
		}
		case LEFT:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				ofstream fout("plan_data_in_ground.txt", ios::out);
				ofstream angle("input_angle.txt", ios::out);
				ofstream xyz_leg("xyz_in_leg.txt", ios::out);
				param.a = 0;          //ǰ�󲽳�mm
				param.b = 80;            //����mm
				param.c = -100;            //���Ҳ���      
				param.pitch = 0;          //��λΪ��
				param.roll = 0;
				param.yaw = 0;
				param.n = n;             //����
				param.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//���߲�������
					ret = walk_plan(i, &param);  //�켣�滮

					{
						for (int j = 0; j < 12; j++)
						{
							fout << file_current_leg[j] << "\t";
						}
						fout << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

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
				{
					fout.close();
					angle.close();
					xyz_leg.close();
				}
				std::cout << "left finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "left finish");
			break;
		}
		case RIGHT:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				ofstream fout("plan_data_in_ground.txt", ios::out);
				ofstream angle("input_angle.txt", ios::out);
				ofstream xyz_leg("xyz_in_leg.txt", ios::out);
				param.a = 0;          //ǰ�󲽳�mm
				param.b = 80;            //����mm
				param.c = 100;            //���Ҳ���
				param.pitch = 0;          //��λΪ��
				param.roll = 0;
				param.yaw = 0;
				param.n = n;             //����
				param.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//���߲�������
					ret = walk_plan(i, &param);  //�켣�滮

					{
						for (int j = 0; j < 12; j++)
						{
							fout << file_current_leg[j] << "\t";
						}
						fout << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

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
				{
					fout.close();
					angle.close();
					xyz_leg.close();
				}
				std::cout << "right finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "right finish");
			break;
		}
		case TURNL:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				ofstream fout("plan_data_in_ground.txt", ios::out);
				ofstream angle("input_angle.txt", ios::out);
				ofstream xyz_leg("xyz_in_leg.txt", ios::out);
				param.a = 0;          //ǰ�󲽳�mm
				param.b = 80;            //����mm
				param.c = 0;            //���Ҳ���
				param.pitch = 0;          //��λΪ��
				param.roll = 0;
				param.yaw = 60;
				param.n = n;             //����
				param.per_step_count = 50;   //0.5s     1s��100�Σ�10������1�Σ�

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//���߲�������
					ret = walk_plan(i, &param);  //�켣�滮

					{
						for (int j = 0; j < 12; j++)
						{
							fout << file_current_leg[j] << "\t";
						}
						fout << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

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
				{
					fout.close();
					angle.close();
					xyz_leg.close();
				}
				
				std::cout << "turn left finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "turn left finish");
			break;
		}
		case TURNR:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
		
				std::cout << "turn right finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "turn right finish");
			break;
		}
	}


}

void main()
{
	/*����ofstream��Ĺ��캯������һ���ļ���������������ļ�*/

	//if (!angle)
	//{
	//	std::cout << "angle�ļ����ܴ�" << std::endl;
	//}
	//else
	//{
	//	std::cout << "angle�ļ��ܴ�" << std::endl;
	//}
	//if (!fout)
	//{
	//	std::cout << "xyz�ļ����ܴ�" << std::endl;
	//}
	//else
	//{
	//	std::cout << "xyz�ļ��ܴ�" << std::endl;
	//}

	
	state = table_state[state][command];  //״̬��ʼ��



	//����
	aris::core::Socket socket("sock", "", "5866", aris::core::Socket::WEB);

	socket.setOnReceivedMsg([](aris::core::Socket* socket, aris::core::Msg& msg)->int
		{
			auto send_ret = [socket](aris::core::Msg& ret_msg)->void
			{
				try
				{
					socket->sendMsg(ret_msg);
				}
				catch (std::exception & e)
				{
					std::cout << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			};
			auto send_code_and_msg = [send_ret, msg](int code, const std::string& ret_msg_str)
			{
				nlohmann::json js;
				js["return_code"] = code;
				js["return_message"] = ret_msg_str;

				aris::core::Msg ret_msg = msg;
				ret_msg.copy(js.dump(2));
				send_ret(ret_msg);
			};

			auto msg_data = std::string_view(msg.data(), msg.size());
			if (msg_data == "get") return 0;

			std::string  str = msg.toString();





			//�յ���ҳ�����
			str_command = StringParser(str);   //�ַ����������õ�����Ͳ���

			try
			{
				if (current_state(str_command) == -1)    //���������ж���һ��״̬����״̬���������״̬���������Ϊ��һ��״̬������������Ϣ
				{
					state = state;
					send_code_and_msg(-1, "fail");
				}
				else
				{
					state = current_state(str_command);
				}
				auto [code, ret_str] = execute(command, state);    //ִ��״̬���º������
				send_code_and_msg(code, ret_str);
			}
			catch (std::exception & e)
			{
				send_code_and_msg(-1, e.what());
			}

			return 0;
		});
	socket.setOnReceivedConnection([](aris::core::Socket* sock, const char* ip, int port)->int
		{
			std::cout << "socket receive connection" << std::endl;
			LOG_INFO << "socket receive connection:\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
			return 0;
		});
	socket.setOnLoseConnection([](aris::core::Socket* socket)->int
		{
			std::cout << "socket lose connection" << std::endl;
			LOG_INFO << "socket lose connection" << std::endl;
			for (;;)
			{
				try
				{
					socket->startServer(socket->port());
					break;
				}
				catch (std::runtime_error & e)
				{
					std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			}
			std::cout << "socket restart successful" << std::endl;
			LOG_INFO << "socket restart successful" << std::endl;

			return 0;
		});

	socket.startServer();


	for (;;)std::this_thread::sleep_for(std::chrono::milliseconds(1));


}








