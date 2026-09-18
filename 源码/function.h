#pragma once
#include"struct.h"
//前面的函数均服务于惯导状态更新

//外推法
void Extrap(double b2_x, double b1_x, double& b05_x)
{
	b05_x = 1.5 * b1_x - 0.5 * b2_x;
}
//时间平均法
void TimeAvg(double b1_x, double b0_x, double& b05_x)
{
	b05_x = 0.5 * b1_x + 0.5 * b0_x;
}

//粗对准:当地真实重力加速度、地球自转角速度+当前IMU测量值-》当前姿态矩阵
void RoughCnb(Matrix Gp_true, Matrix Wie_true, Matrix IMU_dv, Matrix IMU_dthe, Matrix& C)
{
	Matrix Gp = -1 / dt * IMU_dv, Wie = 1 / dt * IMU_dthe;

	Matrix Vg = Gp_true.unit(), Vgw = (Gp_true.SSM() * Wie_true).unit(), Vgwg = (Gp_true.SSM() * Wie_true.SSM() * Gp_true).unit();
	Matrix Wg = Gp.unit(), Wgw = (Gp.SSM() * Wie).unit(), Wgwg = (Gp.SSM() * Wie.SSM() * Gp).unit();

	Matrix V(3, 3), W(3, 3);
	for (int k = 0; k < 3; k++)
	{
		V(0, k) = Vg.transpose()(0, k); W(0, k) = Wg.transpose()(0, k);
		V(1, k) = Vgw.transpose()(0, k); W(1, k) = Wgw.transpose()(0, k);
		V(2, k) = Vgwg.transpose()(0, k); W(2, k) = Wgwg.transpose()(0, k);
	}
	C = V * W.inverse();
	C = C.normalizeCnb();
}

//速度位置-》状态中间量
void VXtoWG(const Matrix X, const Matrix V,double &RM ,double &RN, Matrix& Wie, Matrix& Wen, Matrix& Gp)
{
	double FY = X(0, 0) * pi / 180;

	RM = RE_a * (1 - RE_e * RE_e) / pow(1 - RE_e * RE_e * sin(FY) * sin(FY), 1.5);
	RN = RE_a / sqrt(1 - RE_e * RE_e * sin(FY) * sin(FY));

	Wie(0, 0) = RE_we * cos(FY);
	Wie(1, 0) = 0;
	Wie(2, 0) = -RE_we * sin(FY);

	Wen(0, 0) = V(1, 0) / (RN + X(2, 0));
	Wen(1, 0) = -V(0, 0) / (RM + X(2, 0));
	Wen(2, 0) = -V(1, 0) * tan(FY) / (RN + X(2, 0));

	//Gp(0, 0) = -8.08 * 1e-3 * X(2,0) * sin(2 * X(0,0) * pi / 180);
	Gp(0, 0) = 0;
	Gp(1, 0) = 0;
	//double GpFY = (RE_a * RE_ga * cos(FY) * cos(FY) + RE_b * RE_gb * sin(FY) * sin(FY)) / sqrt(pow(RE_a * cos(FY), 2) + pow(RE_b * sin(FY), 2));
	//double m = RE_we * RE_we * RE_a * RE_a * RE_b / RE_GM;
	//Gp(2, 0) = GpFY*(1-2/RE_a*(1+ RE_f +m-2*RE_f* sin(FY) * sin(FY))*X(2,0)+3/RE_a/RE_a*X(2,0)*X(2,0));
	double g0 = 9.7803267715 * (1 + 0.0052790414 * sin(FY) * sin(FY) + 0.0000232718 * pow(sin(FY), 4));
	Gp(2, 0) = g0 - (3.087691089 * 1e-6 - 4.397731 * 1e-9 * sin(FY) * sin(FY)) * X(2, 0) + 0.721 * 1e-12 * X(2, 0) * X(2, 0);
}

//姿态角(横滚、俯仰、航向)《--》姿态矩阵
void EUtoCnb(Matrix SL, Matrix& C)
{
	double roll = SL(0, 0) * pi / 180.0;   // 横滚（X轴）
	double pitch = SL(1, 0) * pi / 180.0;  // 俯仰（Y轴）
	double yaw = SL(2, 0) * pi / 180.0;    // 航向（Z轴）

	double cr = cos(roll), sr = sin(roll);
	double cp = cos(pitch), sp = sin(pitch);
	double cy = cos(yaw), sy = sin(yaw);

	C(0, 0) = cy * cp;
	C(0, 1) = cy * sp * sr - sy * cr;
	C(0, 2) = cy * sp * cr + sy * sr;
	C(1, 0) = sy * cp;
	C(1, 1) = sy * sp * sr + cy * cr;
	C(1, 2) = sy * sp * cr - cy * sr;
	C(2, 0) = -sp;
	C(2, 1) = cp * sr;
	C(2, 2) = cp * cr;
}
void CnbtoEU(Matrix C, Matrix& SL)
{
	SL(0, 0) = atan2(C(2, 1), C(2, 2)) * 180 / pi;
	SL(1, 0) = -atan2(C(2, 0), sqrt(C(2, 1) * C(2, 1) + C(2, 2) * C(2, 2))) * 180 / pi;
	SL(2, 0) = atan2(C(1, 0), C(0, 0)) * 180 / pi;
	if (SL(2, 0) > 180)SL(2, 0) -= 360;
}
//等效旋转矢量-》姿态矩阵
void SLtoCnb(Matrix SL, Matrix& C)
{
	double mode = SL.norm();
	double c1 = sin(mode) / mode, c2 = (1 - cos(mode)) / mode / mode;
	C = Matrix().identity(3) + c1 * SL.SSM() + c2 * SL.SSM() * SL.SSM();
}
//零速检测和修正：IMU-》V
void ZeroV(Matrix IMU_dv, Matrix IMU_dthe, Matrix& V)
{
	if (fabs(IMU_dv.norm() / dt - Local_Gp) < 0.2 && fabs(IMU_dthe.norm() / dt - RE_we) < 0.006)V = 0 * V;
}





