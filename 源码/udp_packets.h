#pragma once
#include "struct.h"

struct ImuPacket
{
    unsigned int seq;
    double sow;
    double gx_deg_s;
    double gy_deg_s;
    double gz_deg_s;
    double ax_m_s2;
    double ay_m_s2;
    double az_m_s2;
};

struct GnssPacket
{
    unsigned int seq;
    double sow;
    double lat_deg;
    double lon_deg;
    double h_m;
    double ve_m_s;
    double vn_m_s;
    double vu_m_s;
    double east_std_m;
    double north_std_m;
    double up_std_m;
    double east_vel_std_m_s;
    double north_vel_std_m_s;
    double up_vel_std_m_s;
};

inline ImuPacket StateToImuPacket(const State& s, unsigned int seq = 0)
{
    ImuPacket pkt{};
    pkt.seq = seq;
    pkt.sow = s.time;
    pkt.gx_deg_s = s.IMU_dthe(0, 0) / (deg2rad * dt);
    pkt.gy_deg_s = s.IMU_dthe(1, 0) / (deg2rad * dt);
    pkt.gz_deg_s = s.IMU_dthe(2, 0) / (deg2rad * dt);
    pkt.ax_m_s2 = s.IMU_dv(0, 0) / dt;
    pkt.ay_m_s2 = s.IMU_dv(1, 0) / dt;
    pkt.az_m_s2 = s.IMU_dv(2, 0) / dt;
    return pkt;
}

inline GnssPacket StateToGnssPacket(const State& s, unsigned int seq = 0)
{
    GnssPacket pkt{};
    pkt.seq = seq;
    pkt.sow = s.time;
    pkt.lat_deg = s.X(0, 0);
    pkt.lon_deg = s.X(1, 0);
    pkt.h_m = s.X(2, 0);
    pkt.ve_m_s = s.V(1, 0);
    pkt.vn_m_s = s.V(0, 0);
    pkt.vu_m_s = -s.V(2, 0);
    pkt.east_std_m = s.pos_std(1, 0);
    pkt.north_std_m = s.pos_std(0, 0);
    pkt.up_std_m = s.pos_std(2, 0);
    pkt.east_vel_std_m_s = s.vel_std(1, 0);
    pkt.north_vel_std_m_s = s.vel_std(0, 0);
    pkt.up_vel_std_m_s = s.vel_std(2, 0);
    return pkt;
}

inline State ImuPacketToState(const ImuPacket& pkt)
{
    State s;
    s.time = pkt.sow;
    s.IMU_dthe = Matrix(3, 1);
    s.IMU_dv = Matrix(3, 1);
    s.IMU_dthe(0, 0) = pkt.gx_deg_s * deg2rad * dt;
    s.IMU_dthe(1, 0) = pkt.gy_deg_s * deg2rad * dt;
    s.IMU_dthe(2, 0) = pkt.gz_deg_s * deg2rad * dt;
    s.IMU_dv(0, 0) = pkt.ax_m_s2 * dt;
    s.IMU_dv(1, 0) = pkt.ay_m_s2 * dt;
    s.IMU_dv(2, 0) = pkt.az_m_s2 * dt;
    return s;
}

inline State GnssPacketToState(const GnssPacket& pkt)
{
    State s;
    s.time = pkt.sow;
    s.X = Matrix(3, 1);
    s.V = Matrix(3, 1);
    s.pos_std = Matrix(3, 1);
    s.vel_std = Matrix(3, 1);
    s.X(0, 0) = pkt.lat_deg;
    s.X(1, 0) = pkt.lon_deg;
    s.X(2, 0) = pkt.h_m;
    s.V(0, 0) = pkt.vn_m_s;
    s.V(1, 0) = pkt.ve_m_s;
    s.V(2, 0) = -pkt.vu_m_s;
    s.pos_std(0, 0) = pkt.north_std_m;
    s.pos_std(1, 0) = pkt.east_std_m;
    s.pos_std(2, 0) = pkt.up_std_m;
    s.vel_std(0, 0) = pkt.north_vel_std_m_s;
    s.vel_std(1, 0) = pkt.east_vel_std_m_s;
    s.vel_std(2, 0) = pkt.up_vel_std_m_s;
    return s;
}
