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








///**************网页命令解析******************/
int command = 0, state = 0;
int step = 0;
std::string str_command;
enum  //状态常量
{
	INIT = 0,		//初始
	HOMED = 1,		//回原点
	HOMEFINISH = 2, //回原点结束
	PREPAIRD = 3,	 //准备
	RUNNING = 4,		//运行
	ERRO = 5,		 //错误
};

enum  //命令常量
{
	HOME = 0,      //回原点
	PREPAIR = 1,   //准备
	FORWARD = 2,   //前进
	BACK = 3,      //后退
	LEFT = 4,      //左移
	RIGHT = 5,      //右移
	TURNL = 6,      //左旋
	TURNR = 7		//右旋
};
int table_state[6][8] =       //状态表
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

	int len = cmd.size();  //cmd为网页发送字符串,计算字符长度)
	if (len < 8)
	{
		str1 = cmd;  //str_command为抽取命令字符串
		step = 0;
	}
	else if ((cmd[len - 1] >= 48 && cmd[len - 1] <= 57) && (cmd[len - 2] < 48 || cmd[len - 2] > 57))
	{
		str1.assign(cmd, 0, len - 6);  //str_command为抽取命令字符串
		step = cmd[len - 1];     //a为步数
		step = step - 48;
	}
	else if ((cmd[len - 1] >= 48 && cmd[len - 1] <= 57) && (cmd[len - 2] >= 48 || cmd[len - 2] <= 57))
	{
		str1.assign(cmd, 0, len - 7);  //str_command为抽取命令字符串
		step = (cmd[len - 1] - 48) + (cmd[len - 2] - 48) * 10;     //a为步数

	}
	return str1;
}

///**************网页命令解析******************/


//void execute(WalkParam & x)
//{
//	//fout << "x1" << "\t" << "y1" << "\t" << "z1" << "\t";//输出文件抬头
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
				param.a = 100;          //前后步长mm
				param.b = 80;            //步高mm
				param.c = 0;            //左右步长
				param.pitch = 0;        //单位为度
				param.roll = 0;        
				param.yaw = 0;
				param.n = n;             //步数
				param.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//行走参数配置
					ret = walk_plan(i, &param);  //轨迹规划

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
				param.a = -100;          //前后步长mm
				param.b = 80;            //步高mm
				param.c = 0;            //左右步长
				param.pitch = 0;       //单位为度
				param.roll = 0;
				param.yaw = 0;
				param.n = n;             //步数
				param.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//行走参数配置
					ret = walk_plan(i, &param);  //轨迹规划

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
				param.a = 0;          //前后步长mm
				param.b = 80;            //步高mm
				param.c = -100;            //左右步长      
				param.pitch = 0;          //单位为度
				param.roll = 0;
				param.yaw = 0;
				param.n = n;             //步数
				param.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//行走参数配置
					ret = walk_plan(i, &param);  //轨迹规划

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
				param.a = 0;          //前后步长mm
				param.b = 80;            //步高mm
				param.c = 100;            //左右步长
				param.pitch = 0;          //单位为度
				param.roll = 0;
				param.yaw = 0;
				param.n = n;             //步数
				param.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//行走参数配置
					ret = walk_plan(i, &param);  //轨迹规划

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
				param.a = 0;          //前后步长mm
				param.b = 80;            //步高mm
				param.c = 0;            //左右步长
				param.pitch = 0;          //单位为度
				param.roll = 0;
				param.yaw = 60;
				param.n = n;             //步数
				param.per_step_count = 50;   //0.5s     1s算100次，10毫秒算1次，

				int ret = 1;
				for (int i = 0; ret; ++i)
				{

					//param = config_motion_param();//行走参数配置
					ret = walk_plan(i, &param);  //轨迹规划

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
	/*利用ofstream类的构造函数创建一个文件输出流对象来打开文件*/

	//if (!angle)
	//{
	//	std::cout << "angle文件不能打开" << std::endl;
	//}
	//else
	//{
	//	std::cout << "angle文件能打开" << std::endl;
	//}
	//if (!fout)
	//{
	//	std::cout << "xyz文件不能打开" << std::endl;
	//}
	//else
	//{
	//	std::cout << "xyz文件能打开" << std::endl;
	//}

	
	state = table_state[state][command];  //状态初始化



	//网络
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





			//收到网页命令后
			str_command = StringParser(str);   //字符串解析，得到命令和步数

			try
			{
				if (current_state(str_command) == -1)    //根据命令判断下一步状态，若状态正常则更新状态，否则继续为上一步状态并反馈错误信息
				{
					state = state;
					send_code_and_msg(-1, "fail");
				}
				else
				{
					state = current_state(str_command);
				}
				auto [code, ret_str] = execute(command, state);    //执行状态更新后的命令
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








