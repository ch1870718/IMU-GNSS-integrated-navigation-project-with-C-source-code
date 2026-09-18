#pragma once
#include"struct.h"

void MixedOne(const State& Forward, const Matrix& P_for, const State& Backward, const Matrix& P_back, State& MixedState, Matrix& P_mix)
{
    if (P_for.rows() != 15 || P_for.cols() != 15 || P_back.rows() != 15 || P_back.cols() != 15)throw invalid_argument("MixedOne函数要求协方差矩阵维度为15x15！");

    const double lat0 = Forward.X(0, 0) * deg2rad;
    const double h0 = Forward.X(2, 0);
    const double RM0 = Forward.RM;
    const double RN0 = Forward.RN;

    MixedState = Forward;
    MixedState.time = Forward.time;

    Matrix pos_diff(3, 1);
    pos_diff(0, 0) = (Backward.X(0, 0) - Forward.X(0, 0)) * deg2rad * (RM0 + h0);
    pos_diff(1, 0) = (Backward.X(1, 0) - Forward.X(1, 0)) * deg2rad * (RN0 + h0) * cos(lat0);
    pos_diff(2, 0) = Forward.X(2, 0) - Backward.X(2, 0);

    P_mix = Matrix(15, 15);

    for (int i = 0; i < 3; ++i)
    {
        const double pf_pos = P_for(i, i);
        const double pb_pos = P_back(i, i);
        const double pos_weight = (pf_pos + pb_pos > 1e-20) ? pf_pos / (pf_pos + pb_pos) : 0.5;

        const double pf_vel = P_for(i + 3, i + 3);
        const double pb_vel = P_back(i + 3, i + 3);
        const double vel_weight = (pf_vel + pb_vel > 1e-20) ? pf_vel / (pf_vel + pb_vel) : 0.5;

        double pos_mix = pos_weight * pos_diff(i, 0);
        if (i == 0)
        {
            double candidate = Forward.X(0, 0) + pos_mix / (RM0 + h0) * 180.0 / pi;
            MixedState.X(0, 0) = min(max(candidate, min(Forward.X(0, 0), Backward.X(0, 0))), max(Forward.X(0, 0), Backward.X(0, 0)));
        }
        else if (i == 1)
        {
            double candidate = Forward.X(1, 0) + pos_mix / ((RN0 + h0) * cos(lat0)) * 180.0 / pi;
            MixedState.X(1, 0) = min(max(candidate, min(Forward.X(1, 0), Backward.X(1, 0))), max(Forward.X(1, 0), Backward.X(1, 0)));
        }
        else
        {
            double candidate = Forward.X(2, 0) - pos_mix;
            MixedState.X(2, 0) = min(max(candidate, min(Forward.X(2, 0), Backward.X(2, 0))), max(Forward.X(2, 0), Backward.X(2, 0)));
        }

        double vel_candidate = Forward.V(i, 0) + vel_weight * (Backward.V(i, 0) - Forward.V(i, 0));
        MixedState.V(i, 0) = min(max(vel_candidate, min(Forward.V(i, 0), Backward.V(i, 0))), max(Forward.V(i, 0), Backward.V(i, 0)));
        MixedState.gy_bias(i, 0) = Forward.gy_bias(i, 0);
        MixedState.ac_bias(i, 0) = Forward.ac_bias(i, 0);

        const double pos_var_mix = (pf_pos + pb_pos > 1e-20) ? (pf_pos * pb_pos / (pf_pos + pb_pos)) : 0.5 * (pf_pos + pb_pos);
        const double vel_var_mix = (pf_vel + pb_vel > 1e-20) ? (pf_vel * pb_vel / (pf_vel + pb_vel)) : 0.5 * (pf_vel + pb_vel);

        P_mix(i, i) = pos_var_mix;
        P_mix(i + 3, i + 3) = vel_var_mix;
        P_mix(i + 6, i + 6) = P_for(i + 6, i + 6);
        P_mix(i + 9, i + 9) = P_for(i + 9, i + 9);
        P_mix(i + 12, i + 12) = P_for(i + 12, i + 12);

        MixedState.pos_std(i, 0) = sqrt(fabs(P_mix(i, i)));
        MixedState.vel_std(i, 0) = sqrt(fabs(P_mix(i + 3, i + 3)));
        MixedState.att_std(i, 0) = sqrt(fabs(P_mix(i + 6, i + 6))) / deg2rad;
        MixedState.gy_bias_std(i, 0) = sqrt(fabs(P_mix(i + 9, i + 9)));
        MixedState.ac_bias_std(i, 0) = sqrt(fabs(P_mix(i + 12, i + 12)));
    }

    MixedState.Cnb = Forward.Cnb;
    MixedState.EU = Forward.EU;
    while (MixedState.EU(2, 0) > 180.0) MixedState.EU(2, 0) -= 360.0;
    while (MixedState.EU(2, 0) < -180.0) MixedState.EU(2, 0) += 360.0;
}

void BuildMixedStates(vector<State>& Forward, vector<Matrix>& P_for,
    vector<State>& Backward, vector<Matrix>& P_back,
    vector<State>& Mixed, vector<Matrix>& P_mix)
{

    reverse(Backward.begin(), Backward.end());
    reverse(P_back.begin(), P_back.end());

    const double tol = 0.5 * dt;
    size_t i = 0, j = 0;

    Mixed.clear();
    P_mix.clear();

    while (i < Forward.size() || j < Backward.size())
    {
        if (i < Forward.size() && (j == Backward.size() || Forward[i].time + tol <
            Backward[j].time))
        {
            Mixed.push_back(Forward[i]);
            P_mix.push_back(P_for[i]);
            ++i;
        }
        else if (j < Backward.size() && (i == Forward.size() || Backward[j].time + tol
            < Forward[i].time))
        {
            Mixed.push_back(Backward[j]);
            P_mix.push_back(P_back[j]);
            ++j;
        }
        else
        {
            State b0;
            Matrix P0;
            MixedOne(Forward[i], P_for[i], Backward[j], P_back[j], b0, P0);
            Mixed.push_back(b0);
            P_mix.push_back(P0);
            ++i;
            ++j;
        }
    }
}