#pragma once
#include "struct.h"

bool ParseIMULine(const string& line, State& s)
{
    if (line.empty()) return false;

    istringstream iss(line);
    double t = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double ax = 0.0, ay = 0.0, az = 0.0;
    if (!(iss >> t >> gx >> gy >> gz >> ax >> ay >> az)) return false;

    s = State();
    s.time = t;
    s.IMU_dthe = Matrix(3, 1);
    s.IMU_dv = Matrix(3, 1);
    s.IMU_dthe(0, 0) = gx * deg2rad * dt;
    s.IMU_dthe(1, 0) = gy * deg2rad * dt;
    s.IMU_dthe(2, 0) = gz * deg2rad * dt;
    s.IMU_dv(0, 0) = ax * dt;
    s.IMU_dv(1, 0) = ay * dt;
    s.IMU_dv(2, 0) = az * dt;
    return true;
}

bool ParseGNSSLine(const string& line, State& s)
{
    if (line.empty()) return false;

    istringstream iss(line);
    double t = 0.0;
    double lat = 0.0, lon = 0.0, h = 0.0;
    double ve = 0.0, vn = 0.0, vu = 0.0;
    double east_std = 0.0, north_std = 0.0, up_std = 0.0;
    double east_vel_std = 0.0, north_vel_std = 0.0, up_vel_std = 0.0;
    if (!(iss >> t >> lat >> lon >> h >> ve >> vn >> vu
        >> east_std >> north_std >> up_std
        >> east_vel_std >> north_vel_std >> up_vel_std)) return false;

    s = State();
    s.time = t;
    s.X = Matrix(3, 1);
    s.V = Matrix(3, 1);
    s.pos_std = Matrix(3, 1);
    s.vel_std = Matrix(3, 1);
    s.X(0, 0) = lat;
    s.X(1, 0) = lon;
    s.X(2, 0) = h;
    s.V(0, 0) = vn;
    s.V(1, 0) = ve;
    s.V(2, 0) = -vu;
    s.pos_std(0, 0) = north_std;
    s.pos_std(1, 0) = east_std;
    s.pos_std(2, 0) = up_std;
    s.vel_std(0, 0) = north_vel_std;
    s.vel_std(1, 0) = east_vel_std;
    s.vel_std(2, 0) = up_vel_std;
    return true;
}

void ReadImuFile(const string& filepath, vector<State>& imu_data)
{
    ifstream fin(filepath);
    if (!fin.is_open()) throw runtime_error("无法打开 IMU 文件！");

    imu_data.clear();
    string line;
    getline(fin, line);

    while (getline(fin, line))
    {
        State s;
        if (ParseIMULine(line, s)) imu_data.push_back(s);
    }
}

void ReadGnssFile(const string& filepath, vector<State>& gnss_data)
{
    ifstream fin(filepath);
    if (!fin.is_open()) throw runtime_error("无法打开 GNSS 文件！");

    gnss_data.clear();
    string line;
    getline(fin, line);
    getline(fin, line);

    while (getline(fin, line))
    {
        State s;
        if (ParseGNSSLine(line, s)) gnss_data.push_back(s);
    }
}
