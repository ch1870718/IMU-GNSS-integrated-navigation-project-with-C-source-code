#pragma once
#include"initial.h"
#include"update.h"
#include"write.h"

void RunDirectionalFilter(vector<State>& imu, const vector<State>& gnss,
    const Noise_imu& noise, const Lever_arm& antlever,
    const Aliment& aliment, const Updataways& ways,
    int direction, vector<State>& states, vector<Matrix>& covs)
{
    Matrix Qc, P, x(15, 1);
    bool initial_is = false;
    int imu_idx = 0, gnss_idx = 0; double nhctime = 0;
    
    while(!initial_is)InitializeAll(imu, gnss, noise, aliment, Qc, P, imu_idx, gnss_idx, nhctime,direction, initial_is);

    State b2 = imu[imu_idx - 2 * direction];
    State b1 = imu[imu_idx - direction];
    states.push_back(b1);
    covs.push_back(P);

    for (; imu_idx >= 0 && imu_idx < static_cast<int>(imu.size()); imu_idx +=
        direction)
    {
        State b0 = imu[imu_idx], gnss_now;
        if (gnss_idx >= 0 && gnss_idx < static_cast<int>(gnss.size()))
        {
            gnss_now = gnss[gnss_idx];
        }

        OnceUpdate(b2, b1, gnss_now, b0, Qc, P, x, noise.corrtime,
            antlever, ways, direction, gnss_idx, nhctime);

        states.push_back(b0);
        covs.push_back(P);
        b2 = b1;
        b1 = b0;
    }
}


