#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include "read.h"

#pragma comment(lib, "ws2_32.lib")

void ReplayUDP(const string& imu_file, const string& gnss_file,
    const string& host, int imu_port, int gnss_port, double speed)
{
    static bool started = false;
    if (!started)
    {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) throw runtime_error("WSAStartup failed");
        started = true;
    }

    struct ReplayRecord
    {
        double time;
        string type;
        string msg;
    };

    vector<ReplayRecord> records;
    size_t imu_count = 0;
    size_t gnss_count = 0;

    {
        ifstream fin(imu_file);
        if (!fin.is_open()) throw runtime_error("cannot open imu file");
        string line;
        getline(fin, line);
        while (getline(fin, line))
        {
            State s;
            if (!ParseIMULine(line, s)) continue;
            records.push_back({ s.time, "imu", line });
            ++imu_count;
        }
    }

    {
        ifstream fin(gnss_file);
        if (!fin.is_open()) throw runtime_error("cannot open gnss file");
        string line;
        getline(fin, line);
        getline(fin, line);
        while (getline(fin, line))
        {
            State s;
            if (!ParseGNSSLine(line, s)) continue;
            records.push_back({ s.time, "gnss", line });
            ++gnss_count;
        }
    }

    sort(records.begin(), records.end(), [](const ReplayRecord& a, const ReplayRecord& b)
    {
        return a.time == b.time ? a.type < b.type : a.time < b.time;
    });

    if (records.empty()) return;

    cout << "[ReplayStats]"
        << " imu_expected=" << imu_count
        << " gnss_expected=" << gnss_count
        << " total_expected=" << records.size()
        << " speed=" << speed
        << endl;

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) throw runtime_error("socket failed");

    sockaddr_in imu_addr{}, gnss_addr{};
    imu_addr.sin_family = AF_INET;
    gnss_addr.sin_family = AF_INET;
    imu_addr.sin_port = htons(static_cast<u_short>(imu_port));
    gnss_addr.sin_port = htons(static_cast<u_short>(gnss_port));
    inet_pton(AF_INET, host.c_str(), &imu_addr.sin_addr);
    inet_pton(AF_INET, host.c_str(), &gnss_addr.sin_addr);

    double pre_time = records.front().time;
    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto& rec = records[i];
        if (i > 0 && speed > 0.0)
        {
            double dt_send = (rec.time - pre_time) / speed;
            if (dt_send > 0.0) Sleep(static_cast<DWORD>(dt_send * 1000.0));
        }
        pre_time = rec.time;

        const sockaddr_in& addr = rec.type == "imu" ? imu_addr : gnss_addr;
        sendto(sock, rec.msg.c_str(), static_cast<int>(rec.msg.size()), 0,
            reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    }

    cout << "[ReplayStats]"
        << " imu_sent=" << imu_count
        << " gnss_sent=" << gnss_count
        << " total_sent=" << records.size()
        << endl;

    closesocket(sock);
}
