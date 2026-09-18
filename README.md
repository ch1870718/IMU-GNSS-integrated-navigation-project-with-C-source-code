# IMU/GNSS Integrated Navigation Project

基于 C++ 和 Python 的 IMU/GNSS 组合导航项目，包含 C++ 核心解算程序、示例数据、Windows 可执行程序以及 Python 结果绘图脚本。

项目支持纯惯导、GNSS 松组合、NHC 约束、正反向滤波融合和实时 GNSS 松组合回放等功能。

## 目录

- [1. 使用的编程环境](#1-使用的编程环境)
- [2. 项目文件结构](#2-项目文件结构)
  - [2.1 总体结构](#21-总体结构)
  - [2.2 文件及函数功能](#22-文件及函数功能)
  - [2.3 主程序与各模块的关系](#23-主程序与各模块的关系)
- [3. 可执行程序使用方法](#3-可执行程序使用方法)
- [4. 辅助绘图程序](#4-辅助绘图程序)
- [5. 输出结果](#5-输出结果)
- [6. 从源码编译](#6-从源码编译)
- [7. 数据与公开说明](#7-数据与公开说明)

## 1. 使用的编程环境

本项目采用 C++ 与 Python 混合实现。

### C++ 部分

- 语言标准：C++17
- 主要功能：IMU 数据处理、惯导状态更新、GNSS 松组合、NHC 更新、正反向滤波融合和实时数据接收
- Windows 可执行程序：`solver_menu.exe`
- Windows 依赖：Winsock、Windows API

### Python 部分

Python 脚本用于生成水平轨迹图、位置/速度/姿态曲线、导航误差曲线以及实时轨迹图。

主要依赖：

- Python 3.12
- NumPy
- Matplotlib

安装依赖：

```powershell
py -3.12 -m pip install -r requirements.txt
```

如果系统中的 Python 启动命令不是 `py -3.12`，也可以使用：

```powershell
python -m pip install numpy matplotlib
```

## 2. 项目文件结构

```text
导航综合实习/
├─ 源码/
│  ├─ main.cpp
│  ├─ struct.h
│  ├─ matrix.h
│  ├─ function.h
│  ├─ read.h
│  ├─ initial.h
│  ├─ update.h
│  ├─ decode.h
│  ├─ mixed.h
│  ├─ write.h
│  ├─ replay.h
│  ├─ socket.h
│  └─ udp_packets.h
├─ 可执行程序/
│  ├─ solver_menu.exe
│  ├─ imu.txt
│  ├─ gnss_20260602_100202_517487.pos
│  ├─ LCI_20260602_100202_517487.pos
│  ├─ plot_horizontal_track.py
│  ├─ plot_state_components.py
│  ├─ plot_ned_error.py
│  └─ plot_ne_realtime.py
├─ requirements.txt
├─ .gitignore
└─ README.md
```

### 2.1 总体结构

项目采用“统一菜单式主程序 + 分模块头文件”的结构，主要分为以下几层：

- **数据结构层**：定义导航状态、IMU/GNSS 数据和程序配置。
- **矩阵与基础数学层**：实现矩阵运算、坐标转换和姿态计算。
- **数据读取层**：解析 IMU 和 GNSS 文本数据。
- **初始化层**：完成时间同步、初始状态和初始协方差设置。
- **离线解算层**：完成惯导传播、GNSS 更新、NHC 更新和滤波流程。
- **正反向融合层**：融合正向滤波和反向滤波结果。
- **实时通信层**：通过 UDP 完成 IMU/GNSS 数据回放和接收。
- **结果输出层**：写出导航结果并调用 Python 脚本绘图。

### 2.2 文件及函数功能

#### `struct.h`

定义项目中使用的主要数据结构：

- `Filename`：保存 IMU、GNSS、真值和输出文件名。
- `Lever_arm`：保存杆臂参数。
- `Noise_imu`：保存 IMU 噪声模型参数。
- `Aliment`：保存粗对准相关开关和参数。
- `Updataways`：控制 GNSS 和 NHC 更新是否启用。
- `State`：保存单个历元的导航状态、IMU 数据和协方差信息。

#### `matrix.h`

提供矩阵和向量基础运算：

- 矩阵构造、行列数获取和元素访问
- 加法、减法、乘法和转置
- 单位矩阵、行提取和列提取
- 矩阵求逆和向量模长计算
- 单位向量、叉乘和反对称矩阵
- 姿态矩阵正交化
- 子矩阵填充

#### `function.h`

提供惯导计算和姿态处理函数：

- `Extrap()`：计算中间历元量。
- `TimeAvg()`：计算时间平均量。
- `RoughCnb()`：完成粗对准并计算初始姿态矩阵。
- `VXtoWG()`：根据位置和速度计算曲率半径、角速度和重力。
- `EUtoCnb()`：将姿态角转换为姿态矩阵。
- `CnbtoEU()`：将姿态矩阵转换为姿态角。
- `SLtoCnb()`：将等效旋转矢量转换为姿态矩阵。
- `ZeroV()`：完成零速检测和零速修正。

#### `read.h`

负责读取和解析输入数据：

- `ParseIMULine()`：解析单行 IMU 数据。
- `ParseGNSSLine()`：解析单行 GNSS 数据。
- `ReadImuFile()`：读取 IMU 数据文件。
- `ReadGnssFile()`：读取 GNSS 数据文件。

#### `initial.h`

负责滤波初始化：

- `BuildQc()`：构造连续系统噪声阵。
- `BuildP0()`：构造初始状态协方差阵。
- `InitializeAll()`：完成时间同步、初值设置和初始化。

#### `update.h`

负责惯导状态传播和滤波更新：

- `Ins_StateUpdate()`：完成惯导状态传播。
- `Ins_CovUpdate()`：完成误差协方差传播。
- `GNSSPosUpdate()`：执行 GNSS 位置更新。
- `GNSSVelUpdate()`：执行 GNSS 速度更新。
- `NHCUpdate()`：执行非完整约束更新。
- `Feedback()`：将滤波误差反馈到导航状态。
- `BiasComp()`：补偿 IMU 零偏。
- `OnceUpdate()`：完成单历元的完整更新。

#### `decode.h`

- `RunDirectionalFilter()`：执行正向或反向滤波，是离线解算的主要流程。

#### `mixed.h`

- `MixedOne()`：融合同一时刻的正向和反向滤波结果。
- `BuildMixedStates()`：生成整段正反向滤波融合结果。

#### `write.h`

负责输出导航结果：

- `WriteHeader()`：写入结果文件表头。
- `WriteOnce()`：写入单个历元结果。
- `WriteOnes()`：写入一组导航状态。
- `AppendOnce()`：实时模式下追加单个历元结果。
- `AppendMany()`：实时模式下批量追加结果。

#### `replay.h`

- `ReplayUDP()`：按照时间顺序通过 UDP 播发 IMU 和 GNSS 数据。

#### `socket.h`

负责实时 UDP 通信和在线解算：

- `OpenUDPSocket()`：打开并绑定 UDP 套接字。
- `RecvIMUUDP()`：接收并解析 IMU UDP 数据。
- `RecvGNSSUDP()`：接收并解析 GNSS UDP 数据。
- `LooseCoupleRealtimeUDP()`：完成实时接收、初始化、逐历元解算和结果输出。

#### `udp_packets.h`

定义 UDP 数据包及其转换函数：

- `ImuPacket`：IMU UDP 数据包结构。
- `GnssPacket`：GNSS UDP 数据包结构。
- `StateToImuPacket()`：将状态转换为 IMU 数据包。
- `StateToGnssPacket()`：将状态转换为 GNSS 数据包。
- `ImuPacketToState()`：将 IMU 数据包转换为状态。
- `GnssPacketToState()`：将 GNSS 数据包转换为状态。

### 2.3 主程序与各模块的关系

程序入口为 `源码/main.cpp`。

离线解算流程：

1. `main.cpp` 设置输入文件名和输出文件名。
2. `read.h` 读取 IMU 和 GNSS 数据。
3. `decode.h` 调用 `RunDirectionalFilter()` 完成正向或反向滤波。
4. 模式 4 额外调用 `mixed.h` 融合正向和反向滤波结果。
5. `write.h` 将结果写入 `output.txt`。
6. `main.cpp` 调用 Python 脚本生成结果图。

实时解算流程：

1. `replay.h` 读取示例数据并通过 UDP 进行回放。
2. `socket.h` 接收 IMU/GNSS 数据。
3. `socket.h` 完成实时初始化和组合导航解算。
4. `write.h` 持续追加写入 `output.txt`。
5. `plot_ne_realtime.py` 读取输出文件并动态更新轨迹图。

## 3. 可执行程序使用方法

进入可执行程序目录：

```powershell
cd 可执行程序
```

运行程序：

```powershell
.\solver_menu.exe
```

程序会显示菜单，输入 `1` 到 `5` 选择解算模式。

### 支持的解算模式

| 模式 | 名称 | 说明 |
|---:|---|---|
| 1 | 纯惯导 | 关闭 GNSS 更新和 NHC 更新。 |
| 2 | GNSS 松组合 | 开启 GNSS 位置和速度更新。 |
| 3 | NHC + GNSS 松组合 | 在模式 2 的基础上增加非完整约束更新。 |
| 4 | 正反向滤波融合 | 分别进行正向和反向滤波，然后融合两组结果。 |
| 5 | 实时 GNSS 松组合 | 通过 UDP 回放数据并进行实时组合导航解算。 |

### 输入文件

程序默认从 `solver_menu.exe` 所在目录读取以下文件：

- `imu.txt`：IMU 原始观测数据。
- `gnss_20260602_100202_517487.pos`：GNSS 位置、速度和标准差数据。
- `LCI_20260602_100202_517487.pos`：参考真值数据，用于误差对比。

绘图脚本也应放在可执行程序所在目录。

### 实时模式

选择模式 `5` 后，程序会询问回放速度：

```text
Input replay speed for mode 5 (default 50):
```

直接回车使用默认值 `50`，也可以输入其他正数，例如：

```text
100
```

实时模式使用本机 UDP 端口：

- IMU：`9001`
- GNSS：`9002`

运行前请确认这两个端口没有被其他程序占用。

## 4. 辅助绘图程序

### `plot_horizontal_track.py`

读取导航输出结果和参考轨迹，绘制水平轨迹对比图：

```powershell
py -3.12 .\plot_horizontal_track.py `
  .\output.txt `
  --ref .\LCI_20260602_100202_517487.pos `
  --save .\output_figures\horizontal_track.png
```

### `plot_state_components.py`

绘制位置、速度和姿态随时间变化的曲线：

```powershell
py -3.12 .\plot_state_components.py `
  .\output.txt `
  --prefix .\output_figures\state
```

### `plot_ned_error.py`

绘制相对于参考轨迹的位置误差和速度误差：

```powershell
py -3.12 .\plot_ned_error.py `
  .\output.txt `
  .\LCI_20260602_100202_517487.pos `
  --prefix .\output_figures\error
```

### `plot_ne_realtime.py`

实时读取 `output.txt`，动态绘制北向-东向轨迹，主要用于模式 `5`：

```powershell
py -3.12 .\plot_ne_realtime.py .\output.txt
```

## 5. 输出结果

运行模式 `1` 到 `4` 后，程序会生成：

```text
output.txt
output_figures/
├─ horizontal_track.png
├─ state_pos.png
├─ state_vel.png
├─ state_att.png
├─ error_pos_err_ned.png
└─ error_vel_err_ned.png
```

其中：

- `output.txt`：导航解算结果。
- `horizontal_track.png`：水平轨迹对比图。
- `state_pos.png`：位置曲线。
- `state_vel.png`：速度曲线。
- `state_att.png`：姿态曲线。
- `error_pos_err_ned.png`：NED 位置误差曲线。
- `error_vel_err_ned.png`：NED 速度误差曲线。

`output.txt` 和 `output_figures/` 属于程序运行生成物，已在 `.gitignore` 中排除，不需要提交到 Git。

## 6. 从源码编译

使用 Visual Studio Developer PowerShell 编译：

```powershell
cd 源码
cl /std:c++17 /EHsc main.cpp /Fe:solver_menu.exe
```

编译完成后，应将生成的 `solver_menu.exe` 与以下文件放在同一个目录：

```text
imu.txt
gnss_20260602_100202_517487.pos
LCI_20260602_100202_517487.pos
plot_horizontal_track.py
plot_state_components.py
plot_ned_error.py
plot_ne_realtime.py
```

也可以直接使用仓库中已经提供的 `可执行程序/solver_menu.exe`。

## 7. 数据与公开说明

仓库中的数据仅作为程序运行示例。公开项目时，请确认 IMU、GNSS 和参考轨迹数据具有公开授权。

如果数据来自课程、实习单位、实验室或其他项目，请先确认是否允许上传到公开 GitHub 仓库。
