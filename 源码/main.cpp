#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <shellapi.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "read.h"
#include "decode.h"
#include "mixed.h"
#include "write.h"
#include "replay.h"
#include "socket.h"

#pragma comment(lib, "shell32.lib")
using namespace std;

wstring GetExecutableDirectory()
{
    wchar_t buffer[MAX_PATH];
    DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (length == 0 || length == MAX_PATH) return L".";

    wstring full(buffer);
    size_t pos = full.find_last_of(L"\\/");
    if (pos == wstring::npos) return L".";
    return full.substr(0, pos);
}

wstring JoinPath(const wstring& a, const wstring& b)
{
    if (a.empty()) return b;
    if (a.back() == L'\\' || a.back() == L'/') return a + b;
    return a + L"\\" + b;
}

wstring QuoteWide(const wstring& s)
{
    return L"\"" + s + L"\"";
}

string ToNarrow(const wstring& text)
{
    if (text.empty()) return string();
    int size = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), -1, nullptr, 0, nullptr,
        nullptr);
    if (size <= 0) return string(text.begin(), text.end());
    string narrow(size - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), -1, &narrow[0], size, nullptr,
        nullptr);
    return narrow;
}

void EnsureDirectory(const wstring& dir)
{
    CreateDirectoryW(dir.c_str(), nullptr);
}

void LaunchPython(const wchar_t* args, const wchar_t* working_dir)
{
    HINSTANCE result = ShellExecuteW(nullptr, L"open", L"py", args, working_dir,
        SW_SHOWNORMAL);
    if ((intptr_t)result <= 32)
    {
        wcerr << L"Failed to launch py, code = " << (intptr_t)result << endl;
    }
}

double InputRealtimeSpeed()
{
    cout << "Input replay speed for mode 5 (default 50): ";
    string line;
    getline(cin >> ws, line);

    if (line.empty()) return 50.0;

    try
    {
        double speed = stod(line);
        if (speed > 0.0) return speed;
    }
    catch (...) {}

    cout << "Invalid input. Using default speed 50.\n";
    return 50.0;
}

int main()
{
    const wstring exe_dir = GetExecutableDirectory();
    SetCurrentDirectoryW(exe_dir.c_str());
    const wstring figure_dir = JoinPath(exe_dir, L"output_figures");
    EnsureDirectory(figure_dir);

    Filename file;
    file.imu = "imu.txt";
    file.gnss = "gnss_20260602_100202_517487.pos";
    file.output = "output.txt";

    const wstring truth_file = JoinPath(exe_dir, L"LCI_20260602_100202_517487.pos");
    const wstring horizontal_script = JoinPath(exe_dir, L"plot_horizontal_track.py");
    const wstring state_script = JoinPath(exe_dir, L"plot_state_components.py");
    const wstring error_script = JoinPath(exe_dir, L"plot_ned_error.py");
    const wstring realtime_script = JoinPath(exe_dir, L"plot_ne_realtime.py");

    cout << "Select mode:\n";
    cout << "1. Pure INS\n";
    cout << "2. GNSS loose coupled\n";
    cout << "3. NHC + GNSS loose coupled\n";
    cout << "4. Forward/backward fusion\n";
    cout << "5. Realtime GNSS loose coupled\n";
    cout << "Input mode number: ";

    int mode = 0;
    cin >> mode;

    if (mode < 1 || mode > 5)
    {
        cerr << "Invalid mode.\n";
        return 1;
    }

    wcout << L"EXE DIR: " << exe_dir << endl;
    cout << "IMU: " << file.imu << endl;
    cout << "GNSS: " << file.gnss << endl;
    cout << "OUTPUT: " << file.output << endl;

    if (mode == 5)
    {
        double speed = InputRealtimeSpeed();

        ofstream(file.output).close();

        thread recv_thread([&]()
        {
            LooseCoupleRealtimeUDP(file, 9001, 9002, 60000, speed);
        });

        this_thread::sleep_for(chrono::milliseconds(1000));

        double plot_step = 1.0 / speed;
        if (plot_step < 0.02) plot_step = 0.02;

        wstring plot_args =
            L"-3.12 " + QuoteWide(realtime_script) +
            L" " + QuoteWide(JoinPath(exe_dir, L"output.txt")) +
            L" --plot-step " + to_wstring(plot_step);

        LaunchPython(plot_args.c_str(), exe_dir.c_str());

        ReplayUDP(file.imu, file.gnss, "127.0.0.1", 9001, 9002, speed);

        recv_thread.join();
        return 0;
    }

    vector<State> imu, gnss;
    ReadImuFile(file.imu, imu);
    ReadGnssFile(file.gnss, gnss);

    Noise_imu noise;
    Lever_arm antlever;
    Aliment aliment;
    Updataways ways;

    ways.gnsspos = false;
    ways.gnssvel = false;
    ways.nhc = false;

    if (mode >= 2)
    {
        ways.gnsspos = true;
        ways.gnssvel = true;
    }
    if (mode == 3)
    {
        ways.nhc = true;
    }

    vector<State> output_states;

    if (mode == 4)
    {
        vector<State> forward_states, backward_states, mixed_states;
        vector<Matrix> forward_covs, backward_covs, mixed_covs;

        RunDirectionalFilter(imu, gnss, noise, antlever, aliment, ways, 1,
            forward_states, forward_covs);
        RunDirectionalFilter(imu, gnss, noise, antlever, aliment, ways, -1,
            backward_states, backward_covs);
        BuildMixedStates(forward_states, forward_covs, backward_states, backward_covs,
            mixed_states, mixed_covs);

        output_states.swap(mixed_states);
    }
    else
    {
        vector<Matrix> covs;
        RunDirectionalFilter(imu, gnss, noise, antlever, aliment, ways, 1,
            output_states, covs);
    }

    WriteOnes(file.output, output_states);
    cout << "Output written to " << file.output << endl;

    wstring horizontal_args =
        L"-3.12 " + QuoteWide(horizontal_script) +
        L" " + QuoteWide(JoinPath(exe_dir, L"output.txt")) +
        L" --ref " + QuoteWide(truth_file) +
        L" --save " + QuoteWide(JoinPath(figure_dir, L"horizontal_track.png")) +
        L" --show";
    LaunchPython(horizontal_args.c_str(), exe_dir.c_str());

    wstring state_args =
        L"-3.12 " + QuoteWide(state_script) +
        L" " + QuoteWide(JoinPath(exe_dir, L"output.txt")) +
        L" --prefix " + QuoteWide(JoinPath(figure_dir, L"state"));
    LaunchPython(state_args.c_str(), exe_dir.c_str());

    wstring error_args =
        L"-3.12 " + QuoteWide(error_script) +
        L" " + QuoteWide(JoinPath(exe_dir, L"output.txt")) +
        L" " + QuoteWide(truth_file) +
        L" --prefix " + QuoteWide(JoinPath(figure_dir, L"error"));
    LaunchPython(error_args.c_str(), exe_dir.c_str());

    return 0;
}
