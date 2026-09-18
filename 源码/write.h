#pragma once
#include"struct.h"

void WriteHeader(ostream& out)
{
    out << "time lat lon h vn ve vd roll pitch yaw\n";
}

void WriteOnce(ostream& out, const State& s)
{
    out << s.time << ' '
        << s.X(0, 0) << ' '
        << s.X(1, 0) << ' '
        << s.X(2, 0) << ' '
        << s.V(0, 0) << ' '
        << s.V(1, 0) << ' '
        << s.V(2, 0) << ' '
        << s.EU(0, 0) << ' '
        << s.EU(1, 0) << ' '
        << s.EU(2, 0) << '\n';
}

void WriteOnes(const string& filepath, const vector<State>& states)
{
    ofstream fout(filepath);
    if (!fout.is_open()) throw runtime_error("无法打开输出文件！");

    fout << fixed << setprecision(10);
    WriteHeader(fout);

    for (const auto& s : states)
    {
        WriteOnce(fout, s);
    }
}

void AppendOnce(const string& filepath, const State& s)
{
    const bool new_file = !ifstream(filepath).good();

    ofstream fout(filepath, ios::app);
    if (!fout.is_open()) throw runtime_error("无法打开输出文件！");

    fout << fixed << setprecision(10);
    if (new_file)
    {
        WriteHeader(fout);
    }

    WriteOnce(fout, s);
}

void AppendMany(const string& filepath, const vector<State>& states)
{
    if (states.empty()) return;

    const bool new_file = !ifstream(filepath).good();

    ofstream fout(filepath, ios::app);
    if (!fout.is_open()) throw runtime_error("无法打开输出文件！");

    fout << fixed << setprecision(10);
    if (new_file)
    {
        WriteHeader(fout);
    }

    for (const auto& s : states)
    {
        WriteOnce(fout, s);
    }
}
