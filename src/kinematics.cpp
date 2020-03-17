#include<cmath>
#include<iostream>
#include<fstream>
#include"plan.h"
#include"kinematics.h"

extern double file_leg_xyz[12];
extern double input_angle[12];
double L1 = 86.07;
double L2 = 306;
double L3 = 341.04;

//身体在腿坐标系下的变换矩阵
double PL1[16] =
{
    -1, 0,  0,  body_long/2 ,
     0, 1,  0,  0           ,
     0, 0, -1, -body_width/2,
     0, 0,  0,  1           
};

double PL2[16] =
{
    -1, 0,  0, -body_long/2 ,
     0, 1,  0,  0           ,
     0, 0, -1, -body_width/2,
     0, 0,  0,  1           
};

double PL3[16] =
{
    1, 0, 0,  body_long/2 ,
    0, 1, 0,  0           ,
    0, 0, 1, -body_width/2,
    0, 0, 0,  1           
};

double PL4[16] =
{
    1, 0, 0, -body_long/2 ,
    0, 1, 0,  0           ,
    0, 0, 1, -body_width/2,
    0, 0, 0,  1           
};





double* t_matrix_ground_to_leg(const double* p_bl, double* p_gb, double* pm_out)
{
	pm_out[0] = p_bl[0] * p_gb[0] + p_bl[1] * p_gb[4] + p_bl[2] * p_gb[8];
	pm_out[1] = p_bl[0] * p_gb[1] + p_bl[1] * p_gb[5] + p_bl[2] * p_gb[9];
	pm_out[2] = p_bl[0] * p_gb[2] + p_bl[1] * p_gb[6] + p_bl[2] * p_gb[10];
	pm_out[3] = p_bl[0] * p_gb[3] + p_bl[1] * p_gb[7] + p_bl[2] * p_gb[11] + p_bl[3];

	pm_out[4] = p_bl[4] * p_gb[0] + p_bl[5] * p_gb[4] + p_bl[6] * p_gb[8];
	pm_out[5] = p_bl[4] * p_gb[1] + p_bl[5] * p_gb[5] + p_bl[6] * p_gb[9];
	pm_out[6] = p_bl[4] * p_gb[2] + p_bl[5] * p_gb[6] + p_bl[6] * p_gb[10];
	pm_out[7] = p_bl[4] * p_gb[3] + p_bl[5] * p_gb[7] + p_bl[6] * p_gb[11] + p_bl[7];

	pm_out[8] = p_bl[8] * p_gb[0] + p_bl[9] * p_gb[4] + p_bl[10] * p_gb[8];
	pm_out[9] = p_bl[8] * p_gb[1] + p_bl[9] * p_gb[5] + p_bl[10] * p_gb[9];
	pm_out[10] = p_bl[8] * p_gb[2] + p_bl[9] * p_gb[6] + p_bl[10] * p_gb[10];
	pm_out[11] = p_bl[8] * p_gb[3] + p_bl[9] * p_gb[7] + p_bl[10] * p_gb[11] + p_bl[11];

	pm_out[12] = 0;
	pm_out[13] = 0;
	pm_out[14] = 0;
	pm_out[15] = 1;

	return pm_out;
}

double* compute_xyz_for_leg( double* t_matrix, double* xyz_ee, double* to_pp)
{
    to_pp[0] = t_matrix[0] * xyz_ee[0] + t_matrix[1] * xyz_ee[1] + t_matrix[2] * xyz_ee[2] + t_matrix[3];
	to_pp[1] = t_matrix[4] * xyz_ee[0] + t_matrix[5] * xyz_ee[1] + t_matrix[6] * xyz_ee[2] + t_matrix[7];
	to_pp[2] = t_matrix[8] * xyz_ee[0] + t_matrix[9] * xyz_ee[1] + t_matrix[10] * xyz_ee[2] + t_matrix[11];

	return to_pp;
}



void leg_left_12(double* ee_xyz_wrt_leg, double* mot_pos_3)
{
    //计算xita3
    double x = ee_xyz_wrt_leg[0];
    double y = ee_xyz_wrt_leg[1];
    double z = ee_xyz_wrt_leg[2];
    
    double A = 0;
    double B = 0;

    //计算xita1
    A = acos(abs(z) / hypot(y, z)) * 180 / PI;
    B = acos(L1 / hypot(y, z)) * 180 / PI;

    if (y <= 0 && z > 0)
    {
        mot_pos_3[0] = A - B;
    }
    else if (y <= 0 && z < 0)
    {
        mot_pos_3[0] = 180 - A - B;
    }
    else if (y > 0 && z >= 0)
    {
        mot_pos_3[0] = A + B;
    }
    else if (y > 0 && z <= 0)
    {
        mot_pos_3[0] = -180 + A - B;
    }

    //计算xita3

    mot_pos_3[2] = -180 + acos((L2 * L2 + L3 * L3 - (y * y + z * z - L1 * L1 + x * x)) / (2 * L2 * L3)) * 180 / PI;

    //计算xita2

    //判断大小腿坐标系

    A = PI / 2;
    B = sqrt(y * y + z * z - L1 * L1);
    y = -B;

    //计算xita2

    A = acos(abs(x) / hypot(x, y)) * 180 / PI;
    B = acos((L2 * L2 + x * x + y * y - L3 * L3) / (2 * L2 * hypot(x, y))) * 180 / PI;

    if (x <= 0 && y < 0)
    {
        mot_pos_3[1] = A - B;
    }
    else if (x > 0 && y <= 0)
    {
        mot_pos_3[1] = 180 - A - B;
    }
}
void leg_right_34(double* ee_xyz_wrt_leg, double* mot_pos_3)
{
    //计算xita3

    double x = ee_xyz_wrt_leg[0];
    double y = ee_xyz_wrt_leg[1];
    double z = ee_xyz_wrt_leg[2];
    double A = 0;
    double B = 0;



    //计算xita1
    A = acos(abs(z) / hypot(y, z)) * 180 / PI;
    B = acos(L1 / hypot(y, z)) * 180 / PI;

    if (y <= 0 && z > 0)
    {
        mot_pos_3[0] = A - B;
    }
    else if (y <= 0 && z < 0)
    {
        mot_pos_3[0] = 180 - A - B;
    }
    else if (y > 0 && z >= 0)
    { 
        mot_pos_3[0] = A + B;
    }
    else if (y > 0 && z <= 0)
    { 
        mot_pos_3[0] = -180 + A - B;
    }

    //计算xita3

    mot_pos_3[2] = 180 - acos((L2 * L2 + L3 * L3 - (y * y + z * z - L1 * L1 + x * x)) / (2 * L2 * L3)) * 180 / PI;

    //计算xita2

    //判断大小腿坐标系

    A = PI / 2;
    B = sqrt(y * y + z * z - L1 * L1);
    y = -B;

    //计算xita2

    A = acos(abs(x) / hypot(x, y)) * 180 / PI;
    B = acos((L2 * L2 + x * x + y * y - L3 * L3) / (2 * L2 * hypot(x, y))) * 180 / PI;

    if (x <= 0 && y < 0)
    {
        mot_pos_3[1] = A - B;
    }
    else if (x > 0 && y <= 0)
    {
        mot_pos_3[1] = 180 - A - B;
    }
}
int inverse(double *leg_in_ground,double *body_in_ground)
{
    double P_bg[16] =     //地面坐标系相对身体的变换矩阵
    {
        1, 0, 0,-body_in_ground[0],
        0, 1, 0,-body_in_ground[1],
        0, 0, 1,body_in_ground[2],
        0, 0, 0, 1
    };

    double real_pm1[16] = { 0 }, real_pm2[16] = { 0 }, real_pm3[16] = { 0 }, real_pm4[16] = {0};
	t_matrix_ground_to_leg(PL1, P_bg, real_pm1);
	t_matrix_ground_to_leg(PL2, P_bg, real_pm2);
	t_matrix_ground_to_leg(PL3, P_bg, real_pm3);
	t_matrix_ground_to_leg(PL4, P_bg, real_pm4);

    //for (int j = 0; j < 16; j++)
    //{
    //    std::cout << real_pm1[j] << "\t";
    //    if ((j + 1) % 4 == 0)
    //    {
    //        std::cout << std::endl;

    //    }
    //}
    double xyz_in_leg[12] = {0};
	compute_xyz_for_leg(real_pm1, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
	compute_xyz_for_leg(real_pm2, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);
	compute_xyz_for_leg(real_pm3, leg_in_ground + 2 * 3, xyz_in_leg + 2 * 3);
	compute_xyz_for_leg(real_pm4, leg_in_ground + 3 * 3, xyz_in_leg + 3 * 3);

    for (int j = 0; j < 12; j++)
    {
        file_leg_xyz[j] = xyz_in_leg[j];
    }
    //std::cout << std::endl;
    leg_left_12(xyz_in_leg + 0 * 3, input_angle + 0 * 3);//1
    leg_left_12(xyz_in_leg + 1 * 3, input_angle + 1 * 3);//2
    leg_right_34(xyz_in_leg + 2 * 3, input_angle + 2 * 3);//3
    leg_right_34(xyz_in_leg + 3 * 3, input_angle + 3 * 3);//4

	return 0;
}



