#pragma once
#include"function.h"
void BuildQc(const Noise_imu& noise, Matrix& Qc)
{
    Qc = Matrix(12, 12);

    for (size_t i = 0; i < 3; ++i)
    {
        // 块 1 (0~2): 加表白噪声 PSD = VRW²
        Qc(i, i) = pow(noise.ac_walk(i, 0), 2);

        // 块 2 (3~5): 陀螺白噪声 PSD = ARW²
        Qc(3 + i, 3 + i) = pow(noise.gy_walk(i, 0), 2);

        // 块 3 (6~8): 陀螺零偏 GM 驱动噪声 PSD = 2·σ_bg² / τ
        Qc(6 + i, 6 + i) = 2.0 * pow(noise.gy_bias_std(i, 0), 2) / noise.corrtime;

        // 块 4 (9~11): 加表零偏 GM 驱动噪声 PSD = 2·σ_ba² / τ
        Qc(9 + i, 9 + i) = 2.0 * pow(noise.ac_bias_std(i, 0), 2) / noise.corrtime;
    }
}
void BuildP0(const State& init_state, Matrix& P)
{
    P = Matrix(15, 15);

    for (size_t i = 0; i < 3; ++i)
    {
        P(i, i) = pow(init_state.pos_std(i, 0), 2);
        P(3 + i, 3 + i) = pow(init_state.vel_std(i, 0), 2);
        P(6 + i, 6 + i) = pow(init_state.att_std(i, 0), 2);
        P(9 + i, 9 + i) = pow(init_state.gy_bias_std(i, 0), 2);
        P(12 + i, 12 + i) = pow(init_state.ac_bias_std(i, 0), 2);
    }
}

void InitializeAll(vector<State>& imu, const vector<State>& gnss, const Noise_imu& noise, const Aliment& aliment,
    Matrix& Qc, Matrix& P, int& imu_idx, int& gnss_idx, double& nhctime, int direction, bool& initial_is)
{
    int ali_index;
    direction = direction > 0 ? 1 : -1;
    imu_idx = direction > 0 ? 2 : imu.size() - 3;
    gnss_idx = direction > 0 ? 0 : gnss.size() - 1;
    ali_index = imu_idx;

    BuildQc(noise, Qc);

    // 第一个时间同步 imu 与 gnss
    while (0 <= imu_idx && imu_idx < static_cast<int>(imu.size()) &&
        0 <= gnss_idx && gnss_idx < static_cast<int>(gnss.size()))
    {
        double dtime = imu[imu_idx].time - gnss[gnss_idx].time;
        if (fabs(dtime) <= 0.5 * dt)
        {
            initial_is = true;
            break;
        }
        if (direction > 0)
        {
            if (dtime < 0.0) imu_idx += direction;
            else gnss_idx += direction;
        }
        else
        {
            if (dtime > 0.0) imu_idx += direction;
            else gnss_idx += direction;
        }
    }
    if (!initial_is ||
        !(0 <= imu_idx && imu_idx < static_cast<int>(imu.size())) ||
        !(0 <= gnss_idx && gnss_idx < static_cast<int>(gnss.size())))
    {
        initial_is = false;
        return;
    }
    nhctime = imu[imu_idx].time;
    //开头静止数据用于计算初始化姿态
    if (aliment.rough)
    {
        Matrix mean_dv(3, 1);
        Matrix mean_dthe(3, 1);
        size_t ali_num = 0;
        for (; ali_index >= 0 && ali_index < imu.size(); ali_index += direction)
        {
            mean_dv = mean_dv + imu[ali_index].IMU_dv;
            mean_dthe = mean_dthe + imu[ali_index].IMU_dthe;
            ali_num++;
            if (ali_num >= 50000)break;
        }
        mean_dv = mean_dv * (1.0 / static_cast<double>(ali_num));
        mean_dthe = mean_dthe * (1.0 / static_cast<double>(ali_num));

        RoughCnb(Gp_true, Wie_true, mean_dv, mean_dthe, imu[imu_idx].Cnb);
        CnbtoEU(imu[imu_idx].Cnb, imu[imu_idx].EU);
        cout <<"roll:"<< imu[imu_idx].EU(0, 0) << "    pitch: " << imu[imu_idx].EU(1, 0) << "   yaw:" << imu[imu_idx].EU(2, 0) << endl;
    }


    //初始化位姿及其协方差
    imu[imu_idx].X = gnss[gnss_idx].X;
    imu[imu_idx].V = gnss[gnss_idx].V;
    CnbtoEU(imu[imu_idx].Cnb, imu[imu_idx].EU);
    imu[imu_idx].pos_std = gnss[gnss_idx].pos_std;
    imu[imu_idx].vel_std = gnss[gnss_idx].vel_std;
    imu[imu_idx].att_std = { {0.5 * deg2rad},{0.5 * deg2rad},{1.0 * deg2rad} };//经验设定，看要不要关联于imu噪声
    imu[imu_idx].gy_bias_std = noise.gy_bias_std;
    imu[imu_idx].ac_bias_std = noise.ac_bias_std;

    BuildP0(imu[imu_idx], P);
    imu_idx = imu_idx + direction;
    gnss_idx = gnss_idx + direction;
}
