#pragma once
#include"function.h"

void Ins_StateUpdate(State b2, State b1, State& b0)
{
    State b05;//用于储存中间时刻元素，包括速度位置及其对应的角速度重力加速度
    //外推法得到中间时刻b05的速度位置
    for (int i = 0; i < 3; i++)
    {
        Extrap(b2.X(i, 0), b1.X(i, 0), b05.X(i, 0));
        Extrap(b2.V(i, 0), b1.V(i, 0), b05.V(i, 0));
    }
    //速度位置得到n类角速度重力加速度
    VXtoWG(b05.X, b05.V, b05.RM, b05.RN, b05.Wie, b05.Wen, b05.Gp);

    //计算速度V=b1.V+dVf+dVgcor
        //计算dVf=Cnn*b1.Cnb*dV_fb(n类角速度->n类角增量->Cnn,b1.Cnb已知，b1 b0的IMU->积分Cbb*f即dV_fb)
    Matrix b05_Cnn = Matrix().identity(3) - 0.5 * dt * (b05.Wie + b05.Wen).SSM();
    Matrix dV_fb = b0.IMU_dv + 0.5 * b0.IMU_dthe.SSM() * b0.IMU_dv + 1 / 12.0 * (b1.IMU_dthe.SSM() * b0.IMU_dv + b1.IMU_dv.SSM() * b0.IMU_dthe);
    Matrix dVf = b05_Cnn * b1.Cnb * dV_fb;
    //计算dVgcor(n类角速度、重力加速度、速度)
    Matrix dVgcor = (-1 * (2 * b05.Wie + b05.Wen).SSM() * b05.V + b05.Gp) * dt;
    //组装：
    b0.V = b1.V + dVf + dVgcor;
    //零速修正
    //ZeroV(b0.IMU_dv, b0.IMU_dthe, b0.V);
    //时间平均法得到中间时刻b05的新速度
    for (int i = 0; i < 3; i++)TimeAvg(b1.V(i, 0), b0.V(i, 0), b05.V(i, 0));
    //计算位置X=b1.X+dt*b05.V(b05.X、递推的b0.X)
        //计算中间时刻的位置速度对应的中间量
    double FY = b05.X(0, 0) * pi / 180;
    double RM = RE_a * (1 - RE_e * RE_e) / pow(1 - RE_e * RE_e * sin(FY) * sin(FY), 1.5);
    double RN = RE_a / sqrt(1 - RE_e * RE_e * sin(FY) * sin(FY));
    //递推地计算位置b0.X（h->FY->lambda）
    b0.X(2, 0) = b1.X(2, 0) - b05.V(2, 0) * dt;
    TimeAvg(b1.X(2, 0), b0.X(2, 0), b05.X(2, 0));
    b0.X(0, 0) = b1.X(0, 0) + b05.V(0, 0) / (RM + b05.X(2, 0)) * dt * 180 / pi;
    TimeAvg(b1.X(0, 0), b0.X(0, 0), b05.X(0, 0));
    //FY = b05.X(0, 0) * pi / 180;
    //RM = RE_a * (1 - RE_e * RE_e) / pow(1 - RE_e * RE_e * sin(FY) * sin(FY), 1.5);
    //RN = RE_a / sqrt(1 - RE_e * RE_e * sin(FY) * sin(FY));
    b0.X(1, 0) = b1.X(1, 0) + b05.V(1, 0) / (cos(FY) * (RN + b05.X(2, 0))) * dt * 180 / pi;

    //更新当前时刻的中间量以后续INS_P和gnss更新
    VXtoWG(b0.X, b0.V, b0.RM, b0.RN, b0.Wie, b0.Wen, b0.Gp);

    //时间平均法得到中间时刻b05的新位置
    for (int i = 0; i < 3; i++)TimeAvg(b1.X(i, 0), b0.X(i, 0), b05.X(i, 0));
    //速度位置得到n类角速度
    VXtoWG(b05.X, b05.V, b05.RM, b05.RN, b05.Wie, b05.Wen, b05.Gp);

    //计算姿态Cnb=Cnn*b1.Cnb*Cbb->THE（n类角速度->n类角增量->Cnn,b1、直接用的b1.Cnb,b1， b1.IMUdthe、b0.IMUdthe->等效旋转矢量->Cbb）
    Matrix Cnn;
    Matrix SLnn = (b05.Wie + b05.Wen) * dt;
    SLtoCnb(SLnn, Cnn);
    Cnn = Cnn.transpose();
    Matrix Cbb;
    Matrix SLbb = (Matrix().identity(3) + 1 / 12.0 * b1.IMU_dthe.SSM()) * b0.IMU_dthe;
    SLtoCnb(SLbb, Cbb);
    //组装
    b0.Cnb = Cnn * b1.Cnb * Cbb;
    CnbtoEU(b0.Cnb, b0.EU);


}
void Ins_CovUpdate(const State& s, const Matrix& Qc, Matrix& P, double corrtime)
{
    const double lat = s.X(0, 0) * deg2rad;
    const double h = s.X(2, 0);

    const double vn = s.V(0, 0);
    const double ve = s.V(1, 0);
    const double vd = s.V(2, 0);

    const double rmh = s.RM + h;
    const double rnh = s.RN + h;

    Matrix I15 = Matrix().identity(15);
    Matrix I3 = Matrix().identity(3);

    Matrix F(15, 15);
    Matrix G(15, 12);

    Matrix Cnb = s.Cnb;
    Matrix Wie_n = s.Wie;
    Matrix Wen_n = s.Wen;
    Matrix accel = s.IMU_dv * (1.0 / dt);

    Matrix Frr(3, 3);
    Frr(0, 0) = -vd / rmh;
    Frr(0, 1) = 0.0;
    Frr(0, 2) = vn / rmh;
    Frr(1, 0) = ve * tan(lat) / rnh;
    Frr(1, 1) = -(vd + vn * tan(lat)) / rnh;
    Frr(1, 2) = ve / rnh;
    Frr(2, 0) = 0.0;
    Frr(2, 1) = 0.0;
    Frr(2, 2) = 0.0;

    Matrix Fvr(3, 3);
    Fvr(0, 0) = -2.0 * ve * RE_we * cos(lat) / rmh
        - ve * ve / (rmh * rnh * cos(lat) * cos(lat));
    Fvr(0, 1) = 0.0;
    Fvr(0, 2) = vn * vd / (rmh * rmh)
        - ve * ve * tan(lat) / (rnh * rnh);

    Fvr(1, 0) = 2.0 * RE_we * (vn * cos(lat) - vd * sin(lat)) / rmh
        + vn * ve / (rmh * rnh * cos(lat) * cos(lat));
    Fvr(1, 1) = 0.0;
    Fvr(1, 2) = (ve * vd + vn * ve * tan(lat)) / (rnh * rnh);

    Fvr(2, 0) = 2.0 * RE_we * ve * sin(lat) / rmh;
    Fvr(2, 1) = 0.0;
    Fvr(2, 2) = -ve * ve / (rnh * rnh)
        - vn * vn / (rmh * rmh)
        + 2.0 * s.Gp(2, 0) / (sqrt(s.RM * s.RN) + h);

    Matrix Fvv(3, 3);
    Fvv(0, 0) = vd / rmh;
    Fvv(0, 1) = -2.0 * (RE_we * sin(lat) + ve * tan(lat) / rnh);
    Fvv(0, 2) = vn / rmh;

    Fvv(1, 0) = 2.0 * RE_we * sin(lat) + ve * tan(lat) / rnh;
    Fvv(1, 1) = (vd + vn * tan(lat)) / rnh;
    Fvv(1, 2) = 2.0 * RE_we * cos(lat) + ve / rnh;

    Fvv(2, 0) = -2.0 * vn / rmh;
    Fvv(2, 1) = -2.0 * (RE_we * cos(lat) + ve / rnh);
    Fvv(2, 2) = 0.0;

    Matrix Fvphi = (Cnb * accel).SSM();
    Matrix Fvba = Cnb;

    Matrix Fphir(3, 3);
    Fphir(0, 0) = -RE_we * sin(lat) / rmh;
    Fphir(0, 1) = 0.0;
    Fphir(0, 2) = ve / (rnh * rnh);

    Fphir(1, 0) = 0.0;
    Fphir(1, 1) = 0.0;
    Fphir(1, 2) = -vn / (rmh * rmh);

    Fphir(2, 0) = -RE_we * cos(lat) / rmh- ve / (rmh * rnh * cos(lat) * cos(lat));
    Fphir(2, 1) = 0.0;
    Fphir(2, 2) = -ve * tan(lat) / (rnh * rnh);

    Matrix Fphiv(3, 3);
    Fphiv(0, 0) = 0.0;
    Fphiv(0, 1) = 1.0 / rnh;
    Fphiv(0, 2) = 0.0;

    Fphiv(1, 0) = -1.0 / rmh;
    Fphiv(1, 1) = 0.0;
    Fphiv(1, 2) = 0.0;

    Fphiv(2, 0) = 0.0;
    Fphiv(2, 1) = -tan(lat) / rnh;
    Fphiv(2, 2) = 0.0;

    Matrix Fphiphi = -1.0 * (Wie_n + Wen_n).SSM();
    Matrix Fphibg = -1.0 * Cnb;
    Matrix Fbgbg = (-1.0 / corrtime) * I3;
    Matrix Fbaba = (-1.0 / corrtime) * I3;

    F.fillBlock(0, 0, Frr);
    F.fillBlock(0, 3, I3);
    F.fillBlock(3, 0, Fvr);
    F.fillBlock(3, 3, Fvv);
    F.fillBlock(3, 6, Fvphi);
    F.fillBlock(3, 12, Fvba);
    F.fillBlock(6, 0, Fphir);
    F.fillBlock(6, 3, Fphiv);
    F.fillBlock(6, 6, Fphiphi);
    F.fillBlock(6, 9, Fphibg);
    F.fillBlock(9, 9, Fbgbg);
    F.fillBlock(12, 12, Fbaba);

    G.fillBlock(3, 0, Cnb);
    G.fillBlock(6, 3, Cnb);
    G.fillBlock(9, 6, I3);
    G.fillBlock(12, 9, I3);

    Matrix Phi = I15 + F * dt;
    Matrix Qd = G * Qc * G.transpose() * dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) * 0.5;

    P = Phi * P * Phi.transpose() + Qd;
    P = (P + P.transpose()) * 0.5;
}

void GNSSPosUpdate(const State& insstate, const State& gnssstate,const Lever_arm& antlever, Matrix& x, Matrix& P)
{
    
    const double lat_rad = insstate.X(0, 0) * deg2rad;
    const double h = insstate.X(2, 0);

    Matrix posdiff(3, 1);
    posdiff(0, 0) = (insstate.X(0, 0) - gnssstate.X(0, 0)) * deg2rad;
    posdiff(1, 0) = (insstate.X(1, 0) - gnssstate.X(1, 0)) * deg2rad;
    posdiff(2, 0) = insstate.X(2, 0) - gnssstate.X(2, 0);

    Matrix DR(3, 3);
    DR(0, 0) = insstate.RM + h;
    DR(1, 1) = (insstate.RN + h) * cos(lat_rad);
    DR(2, 2) = -1.0;

    Matrix Z = DR * posdiff + insstate.Cnb * antlever.length;

    Matrix R(3, 3);
    R(0, 0) = gnssstate.pos_std(0, 0) * gnssstate.pos_std(0, 0);
    R(1, 1) = gnssstate.pos_std(1, 0) * gnssstate.pos_std(1, 0);
    R(2, 2) = gnssstate.pos_std(2, 0) * gnssstate.pos_std(2, 0);

    Matrix H(3, 15);
    H.fillBlock(0, 0, Matrix().identity(3));
    H.fillBlock(0, 6, (insstate.Cnb * antlever.length).SSM());

    Matrix I = Matrix().identity(15);
    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse();

    
    x = x + K * (Z - H * x);
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    P = (P + P.transpose()) * 0.5;
}

void GNSSVelUpdate(const State& insstate, const State& gnssstate,const Lever_arm& antlever, Matrix& x, Matrix& P)
{

    Matrix win_n = insstate.Wie + insstate.Wen;
    Matrix wib_b = insstate.IMU_dthe * (1.0 / dt);

    Matrix vel_ant = insstate.V
        - win_n.SSM() * insstate.Cnb * antlever.length
        - insstate.Cnb * (antlever.length.SSM() * wib_b);

    Matrix Z = vel_ant - gnssstate.V;

    Matrix R(3, 3);
    R(0, 0) = gnssstate.vel_std(0, 0) * gnssstate.vel_std(0, 0);
    R(1, 1) = gnssstate.vel_std(1, 0) * gnssstate.vel_std(1, 0);
    R(2, 2) = gnssstate.vel_std(2, 0) * gnssstate.vel_std(2, 0);

    Matrix H(3, 15);
    H.fillBlock(0, 3, Matrix().identity(3));

    Matrix Hphi = -1.0 * win_n.SSM() * (insstate.Cnb * antlever.length).SSM()
        - (insstate.Cnb * (antlever.length.SSM() * wib_b)).SSM();
    H.fillBlock(0, 6, Hphi);

    Matrix Hbg = -1.0 * insstate.Cnb * antlever.length.SSM();
    H.fillBlock(0, 9, Hbg);

    Matrix I = Matrix().identity(15);
    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse();

    x = x + K * (Z - H * x);
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    P = (P + P.transpose()) * 0.5;
}

void NHCUpdate(const State& insstate, Matrix& x, Matrix& P)
{
    const double nhc_std = 0.88;

    Matrix Cbn = insstate.Cnb.transpose();
    Matrix Cbn_y = static_cast<const Matrix&>(Cbn).row(1);
    Matrix Cbn_z = static_cast<const Matrix&>(Cbn).row(2);

    Matrix Z(2, 1);
    Z(0, 0) = -(Cbn_y * insstate.V)(0, 0);
    Z(1, 0) = -(Cbn_z * insstate.V)(0, 0);

    Matrix R(2, 2);
    R(0, 0) = nhc_std * nhc_std;
    R(1, 1) = nhc_std * nhc_std;

    Matrix H(2, 15);
    H.fillBlock(0, 3, Cbn_y);
    H.fillBlock(1, 3, Cbn_z);

    Matrix Hphi_y = -1.0 * (Cbn_y * insstate.V.SSM());
    Matrix Hphi_z = -1.0 * (Cbn_z * insstate.V.SSM());
    H.fillBlock(0, 6, Hphi_y);
    H.fillBlock(1, 6, Hphi_z);

    Matrix I = Matrix().identity(15);
    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse();

    x = x + K * (Z - H * x);
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    P = (P + P.transpose()) * 0.5;
}

void Feedback(State& insstate, Matrix& x)
{
    Matrix dr(3, 1);
    dr(0, 0) = x(0, 0);
    dr(1, 0) = x(1, 0);
    dr(2, 0) = x(2, 0);

    Matrix dv(3, 1);
    dv(0, 0) = x(3, 0);
    dv(1, 0) = x(4, 0);
    dv(2, 0) = x(5, 0);

    Matrix dphi(3, 1);
    dphi(0, 0) = x(6, 0);
    dphi(1, 0) = x(7, 0);
    dphi(2, 0) = x(8, 0);

    Matrix dbg(3, 1);
    dbg(0, 0) = x(9, 0);
    dbg(1, 0) = x(10, 0);
    dbg(2, 0) = x(11, 0);

    Matrix dba(3, 1);
    dba(0, 0) = x(12, 0);
    dba(1, 0) = x(13, 0);
    dba(2, 0) = x(14, 0);

    Matrix DR(3, 3);
    DR(0, 0) = insstate.RM + insstate.X(2, 0);
    DR(1, 1) = (insstate.RN + insstate.X(2, 0)) * cos(insstate.X(0, 0) * deg2rad);
    DR(2, 2) = -1.0;

    Matrix dBLH = DR.inverse() * dr;
    dBLH(0, 0) = dBLH(0, 0) / deg2rad;
    dBLH(1, 0) = dBLH(1, 0) / deg2rad;

    insstate.X = insstate.X - dBLH;
    insstate.V = insstate.V - dv;

    Matrix I3 = Matrix().identity(3);
    insstate.Cnb = (I3 + dphi.SSM()) * insstate.Cnb;
    insstate.Cnb = insstate.Cnb.normalizeCnb();
    CnbtoEU(insstate.Cnb, insstate.EU);
    insstate.gy_bias = insstate.gy_bias + dbg;
    insstate.ac_bias = insstate.ac_bias + dba;

    x = Matrix(15, 1);
}

void BiasComp(State& insstate)
{
    insstate.IMU_dthe = insstate.IMU_dthe - insstate.gy_bias * dt;
    insstate.IMU_dv = insstate.IMU_dv - insstate.ac_bias * dt;
}

void OnceUpdate(State b2, State b1, State gnss, State& b0,Matrix& Qc, Matrix& P, Matrix& x, double corrtime, const Lever_arm& antlever, 
    Updataways ways,int direction, int& gnss_idx, double& nhctime)
{
    b0.gy_bias = b1.gy_bias;
    b0.ac_bias = b1.ac_bias;
    BiasComp(b0);

    Ins_StateUpdate(b2, b1, b0);
    Ins_CovUpdate(b0, Qc, P, corrtime);

    bool gnss_coming = fabs(b0.time - gnss.time) <= 0.5 * dt;
    bool nhc_coming = ways.nhc && fabs(b0.time - nhctime) <= 0.5 * dt;

    if (gnss_coming)
    {
        if (ways.gnsspos) GNSSPosUpdate(b0, gnss, antlever, x, P);
        if (ways.gnssvel) GNSSVelUpdate(b0, gnss, antlever, x, P);
        gnss_idx += direction;
    }

    if (nhc_coming)
    {
        NHCUpdate(b0, x, P);
        nhctime += direction / ways.nhc_rate;
    }

    if (gnss_coming || nhc_coming) Feedback(b0, x);
}