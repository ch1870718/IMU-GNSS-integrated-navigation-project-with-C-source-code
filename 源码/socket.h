#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include "read.h"
#include "initial.h"
#include "update.h"
#include "write.h"
#pragma comment(lib, "ws2_32.lib")

SOCKET OpenUDPSocket(int port)
{
    static bool started = false;
    if (!started)
    {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) throw runtime_error("WSAStartup failed");
        started = true;
    }

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) throw runtime_error("socket failed");

    const int recv_buffer_size = 32 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&recv_buffer_size), sizeof(recv_buffer_size));

    const DWORD recv_timeout_ms = 100;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&recv_timeout_ms), sizeof(recv_timeout_ms));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<u_short>(port));

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR)
    {
        closesocket(sock);
        throw runtime_error("bind failed");
    }
    return sock;
}

bool RecvIMUUDP(SOCKET sock, State& imu_state)
{
    char buffer[1024]{};
    sockaddr_in from{};
    int from_len = sizeof(from);
    int n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n <= 0) return false;
    buffer[n] = '\0';
    return ParseIMULine(buffer, imu_state);
}

bool RecvGNSSUDP(SOCKET sock, State& gnss_state)
{
    char buffer[1024]{};
    sockaddr_in from{};
    int from_len = sizeof(from);
    int n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n <= 0) return false;
    buffer[n] = '\0';
    return ParseGNSSLine(buffer, gnss_state);
}

void LooseCoupleRealtimeUDP(const Filename& file, int imu_port = 123, int gnss_port = 456, int idle_timeout_ms = 10000, double speed = 1.0)
{
    constexpr size_t kOutputBatchSize = 128;
    vector<State> imu_buf, gnss_buf;
    vector<State> output_batch;
    deque<State> imu_pending;
    deque<State> gnss_pending;
    imu_buf.reserve(8192);
    gnss_buf.reserve(256);
    output_batch.reserve(kOutputBatchSize);
    mutex imu_pending_mutex;
    mutex gnss_pending_mutex;
    mutex data_cv_mutex;
    condition_variable data_cv;
    atomic<bool> stop_requested{ false };
    Noise_imu noise;
    Lever_arm antlever;
    Aliment aliment;
    Updataways ways;
    Matrix Qc, P, x(15, 1);
    int imu_idx = 0, gnss_idx = 0, direction = 1;
    double nhctime = 0.0;
    bool initial_is = false;
    bool reported_time_gap = false;
    bool reported_imu_gap = false;
    long long imu_recv_count = 0;
    long long gnss_recv_count = 0;

    State b2, b1;

    ofstream(file.output).close();

    SOCKET imu_sock = OpenUDPSocket(imu_port);
    SOCKET gnss_sock = OpenUDPSocket(gnss_port);
    atomic<DWORD> last_data_tick{ GetTickCount() };

    thread imu_recv_thread([&]()
    {
        while (!stop_requested.load())
        {
            bool got_any = false;
            State imu_now;
            while (RecvIMUUDP(imu_sock, imu_now))
            {
                {
                    lock_guard<mutex> lock(imu_pending_mutex);
                    imu_pending.push_back(imu_now);
                }
                ++imu_recv_count;
                got_any = true;
                last_data_tick.store(GetTickCount());
                data_cv.notify_one();
            }
            if (!got_any && WSAGetLastError() != WSAETIMEDOUT) Sleep(1);
        }
    });

    thread gnss_recv_thread([&]()
    {
        while (!stop_requested.load())
        {
            bool got_any = false;
            State gnss_now;
            while (RecvGNSSUDP(gnss_sock, gnss_now))
            {
                {
                    lock_guard<mutex> lock(gnss_pending_mutex);
                    gnss_pending.push_back(gnss_now);
                }
                ++gnss_recv_count;
                got_any = true;
                last_data_tick.store(GetTickCount());
                data_cv.notify_one();
            }
            if (!got_any && WSAGetLastError() != WSAETIMEDOUT) Sleep(1);
        }
    });

    while (true)
    {
        bool got_data = false;
        deque<State> imu_batch;
        deque<State> gnss_batch;

        {
            lock_guard<mutex> lock(imu_pending_mutex);
            if (!imu_pending.empty())
            {
                imu_batch.swap(imu_pending);
            }
        }
        while (!imu_batch.empty())
        {
            const State imu_now = imu_batch.front();
            imu_batch.pop_front();
            if (!reported_imu_gap && !imu_buf.empty())
            {
                double imu_dt = imu_now.time - imu_buf.back().time;
                if (imu_dt > 0.01)
                {
                    reported_imu_gap = true;
                    cout << "[ImuGapCheck] first_gap_over_0.01s"
                        << " prev_idx=" << (imu_buf.size() - 1)
                        << " next_idx=" << imu_buf.size()
                        << " prev_time=" << imu_buf.back().time
                        << " next_time=" << imu_now.time
                        << " gap=" << imu_dt
                        << endl;
                }
            }
            imu_buf.push_back(imu_now);
            got_data = true;
        }

        {
            lock_guard<mutex> lock(gnss_pending_mutex);
            if (!gnss_pending.empty())
            {
                gnss_batch.swap(gnss_pending);
            }
        }
        while (!gnss_batch.empty())
        {
            gnss_buf.push_back(gnss_batch.front());
            gnss_batch.pop_front();
            got_data = true;
        }

        if (!initial_is)
        {
            const int min_gnss_count = 1 + static_cast<int>(speed);
            if (imu_buf.size() >= 4 && static_cast<int>(gnss_buf.size()) >= min_gnss_count)
            {
                try
                {
                    InitializeAll(imu_buf, gnss_buf, noise, aliment, Qc, P, imu_idx, gnss_idx, nhctime, direction, initial_is);
                    if (initial_is)
                    {
                        const int sync_imu_idx = imu_idx - direction;
                        const int sync_gnss_idx = gnss_idx - direction;
                        if (sync_imu_idx >= 0 && sync_imu_idx < static_cast<int>(imu_buf.size()) &&
                            sync_gnss_idx >= 0 && sync_gnss_idx < static_cast<int>(gnss_buf.size()))
                        {
                            cout << "[InitCheck]"
                                << " sync_imu_idx=" << sync_imu_idx
                                << " sync_gnss_idx=" << sync_gnss_idx
                                << " sync_imu_time=" << imu_buf[sync_imu_idx].time
                                << " sync_gnss_time=" << gnss_buf[sync_gnss_idx].time
                                << " sync_gap=" << (imu_buf[sync_imu_idx].time - gnss_buf[sync_gnss_idx].time)
                                << " b2_time=" << imu_buf[imu_idx - 2 * direction].time
                                << " b1_time=" << imu_buf[imu_idx - direction].time
                                << endl;
                        }
                        b2 = imu_buf[imu_idx - 2 * direction];
                        b1 = imu_buf[imu_idx - direction];
                        output_batch.push_back(b1);
                        if (output_batch.size() >= kOutputBatchSize)
                        {
                            AppendMany(file.output, output_batch);
                            output_batch.clear();
                        }
                    }
                }
                catch (...) {}
            }
        }
        else
        {
            while (imu_idx >= 0 && imu_idx < static_cast<int>(imu_buf.size()))
            {
                if (gnss_buf.empty() || gnss_idx < 0 || gnss_idx >= static_cast<int>(gnss_buf.size()))
                {
                    break;
                }

                const double imu_time = imu_buf[imu_idx].time;
                const double latest_gnss_time = gnss_buf.back().time;
                if (latest_gnss_time < imu_time + dt)
                {
                    break;
                }

                while (gnss_idx + 1 < static_cast<int>(gnss_buf.size()) &&
                    gnss_buf[gnss_idx].time < imu_time - dt)
                {
                    ++gnss_idx;
                }

                if (gnss_idx < 0 || gnss_idx >= static_cast<int>(gnss_buf.size()))
                {
                    break;
                }

                State b0 = imu_buf[imu_idx];
                State gnss_now = gnss_buf[gnss_idx];

                if (!reported_time_gap &&
                    gnss_idx >= 0 &&
                    gnss_idx < static_cast<int>(gnss_buf.size()) &&
                    imu_buf[imu_idx].time - gnss_buf[gnss_idx].time > 1.0)
                {
                    reported_time_gap = true;
                    cout << "[AlignCheck] first_gap_over_1s"
                        << " imu_idx=" << imu_idx
                        << " gnss_idx=" << gnss_idx
                        << " imu_time=" << imu_buf[imu_idx].time
                        << " gnss_time=" << gnss_buf[gnss_idx].time
                        << " gap=" << (imu_buf[imu_idx].time - gnss_buf[gnss_idx].time)
                        << endl;
                }

                OnceUpdate(b2, b1, gnss_now, b0, Qc, P, x, noise.corrtime,
                    antlever, ways, direction, gnss_idx, nhctime);

                output_batch.push_back(b0);
                if (output_batch.size() >= kOutputBatchSize)
                {
                    AppendMany(file.output, output_batch);
                    output_batch.clear();
                }

                b2 = b1;
                b1 = b0;
                imu_idx += direction;
            }

            if (imu_idx > 4096)
            {
                const int erase_count = imu_idx - 2;
                imu_buf.erase(imu_buf.begin(), imu_buf.begin() + erase_count);
                imu_idx -= erase_count;
            }

            if (gnss_idx > 256)
            {
                const int erase_count = gnss_idx - 1;
                gnss_buf.erase(gnss_buf.begin(), gnss_buf.begin() + erase_count);
                gnss_idx -= erase_count;
            }

            if (imu_buf.size() > 16384 && imu_idx >= 2)
            {
                const int erase_count = imu_idx - 2;
                imu_buf.erase(imu_buf.begin(), imu_buf.begin() + erase_count);
                imu_idx -= erase_count;
            }

            if (gnss_buf.size() > 2048 && gnss_idx >= 1)
            {
                const int erase_count = gnss_idx - 1;
                gnss_buf.erase(gnss_buf.begin(), gnss_buf.begin() + erase_count);
                gnss_idx -= erase_count;
            }
        }

        if (!got_data)
        {
            unique_lock<mutex> wait_lock(data_cv_mutex);
            data_cv.wait_for(wait_lock, chrono::milliseconds(2));
        }
        if (GetTickCount() - last_data_tick.load() > static_cast<DWORD>(idle_timeout_ms)) break;
    }

    if (!output_batch.empty())
    {
        AppendMany(file.output, output_batch);
        output_batch.clear();
    }

    stop_requested.store(true);
    data_cv.notify_all();
    imu_recv_thread.join();
    gnss_recv_thread.join();

    cout << "[RecvStats]"
        << " imu_received=" << imu_recv_count
        << " gnss_received=" << gnss_recv_count
        << " imu_buf_size=" << imu_buf.size()
        << " gnss_buf_size=" << gnss_buf.size()
        << " imu_idx=" << imu_idx
        << " gnss_idx=" << gnss_idx
        << endl;

    closesocket(imu_sock);
    closesocket(gnss_sock);
    WSACleanup();
}
