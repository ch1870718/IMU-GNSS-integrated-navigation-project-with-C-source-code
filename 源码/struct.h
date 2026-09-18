#pragma once
#include"matrix.h"
//算法均前右下/北东地
const double dt = 0.005;
const double pi = 3.1415926535897932384626433832795;
const double deg2rad = pi / 180.0;

// GRS80 椭球标准参数
const double RE_a = 6378137.0;          // 长半轴（米），与WGS84一致
const double RE_b = 6356752.314140;     // 短半轴（米）
const double RE_f = 1.0 / 298.257222101;// 扁率
const double RE_e = 0.0818191910428158; // 第一偏心率
const double RE_ga = 9.7803267715;      // 赤道正常重力（m/s²）
const double RE_gb = 9.8321863685;      // 极点正常重力（m/s²）
const double RE_GM = 3986005.0e8;       // 地球引力常数（m³/s²）
const double RE_we = 7.2921150e-5;      // 地球自转角速度（rad/s）

//当地地理参数
const double Local_Gp = 9.7936174;
const double Local_G = 9.8204;
const double Local_FY = 30.531651244;
const static Matrix Gp_true = { {0},{0},{Local_Gp} };
const static Matrix Wie_true = { {RE_we * cos(Local_FY / 180 * pi)},{0},{-RE_we * sin(Local_FY / 180 * pi) } };
struct Filename
{
	string imu_static = "imu_static.txt";
	string imu = "imu.txt";
	string gnss = "gnss_20260602_100202_517487.pos";
	string output = "output.txt";
};


struct Lever_arm
{
	Matrix length ={{0.14},{-0.0005},{0.005}};
};

int kkkkk = 1;
struct Noise_imu
{
	Matrix gy_bias = Matrix(3, 1);
	Matrix ac_bias = Matrix(3, 1);
	Matrix gy_walk{ {3.0e-4*kkkkk}, {3.0e-4 * kkkkk}, {3.0e-4 * kkkkk} };
	Matrix ac_walk{ {5.0e-3 * kkkkk}, {5.0e-3 * kkkkk}, {5.0e-3 * kkkkk} };
	Matrix gy_bias_std{ {3.0e-4 * kkkkk}, {3.0e-4 * kkkkk}, {3.0e-4 * kkkkk} };
	Matrix ac_bias_std{ {5.0e-3 * kkkkk}, {5.0e-3 * kkkkk}, {5.0e-3 * kkkkk} };
	double corrtime = 1000.0;
};
struct Aliment
{
	bool rough = false;
};



struct Updataways
{
	bool nhc = false;
	bool gnsspos = true;
	bool gnssvel = true;
	double nhc_rate = 10.0;
};
struct State
{
	//历元
	double time = 0.0;

	//imu原始数据
	Matrix IMU_dv = Matrix(3, 1);
	Matrix IMU_dthe = Matrix(3, 1);

	//中间量
	double RM = 0.0;
	double RN = 0.0;
	Matrix Wie = Matrix(3, 1);
	Matrix Wen = Matrix(3, 1);
	Matrix Gp = Matrix(3, 1);

	//运动状态
	Matrix X = Matrix(3, 1);
	Matrix V = Matrix(3, 1);
	Matrix EU = Matrix(3, 1);
	Matrix Cnb = Matrix().identity(3);
	Matrix gy_bias = Matrix(3, 1);
	Matrix ac_bias = Matrix(3, 1);

	//可能存在的估计的gnss运动状态的std
	Matrix pos_std = Matrix(3, 1);     
	Matrix vel_std = Matrix(3, 1);     
	Matrix att_std = Matrix(3, 1);     
	Matrix gy_bias_std = Matrix(3, 1); 
	Matrix ac_bias_std = Matrix(3, 1);
};