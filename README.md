# 机器人控制系统 / Robot Control System

一个基于 C++ 的智能机器人控制系统，集成图像处理、运动控制与串口通信功能。
An intelligent robot control system in C++ integrating vision, motion control, and serial comms.

## 功能特性 / Features

- 🤖 实时图像处理（OpenCV） / Real-time vision with OpenCV
- 🎮 运动控制（WiringPi） / Precise motor control via WiringPi
- 📡 串口通信 / Serial communication for commands & telemetry
- ⚡ 多线程并发 / Multithreaded concurrency with pthreads

## 技术栈 / Tech Stack

- 语言 / Language: C++
- 图像处理 / Vision: OpenCV
- 硬件控制 / Hardware Control: WiringPi
- 通信 / Comms: Serial
- 并发 / Concurrency: pthread

## 系统要求 / Requirements

- Linux（推荐树莓派）/ Linux (Raspberry Pi recommended)
- GCC/G++
- OpenCV ≥ 3.0
- WiringPi
- pthread

## 编译与运行 / Build & Run

### 安装依赖 / Install Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y build-essential libopencv-dev wiringpi
# 可选：OpenCV contrib | optional
sudo apt-get install -y libopencv-contrib-dev
```

### 编译项目 / Build

```bash
# 直接编译 | direct
g++ -o robot_control main.cpp -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lwiringPi -lpthread -I/usr/include/opencv4

# 使用 Makefile | with Makefile
make
```

### 运行 / Run

```bash
sudo ./robot_control
```

## 项目结构 / Project Structure

```
.
├── main.cpp              # 主程序 | main program
├── Makefile              # 编译配置 | build rules
├── README.md             # 项目说明 | this file
├── LICENSE               # 许可证 | license
└── docs/                 # 文档 | docs
```

## 主要模块 / Modules

### 图像处理 / Vision
- 实时视频捕获 / live video capture
- 目标检测识别 / detection & recognition
- 预处理与滤波 / preprocessing & filtering

### 运动控制 / Motion Control
- 电机驱动控制 / motor driver control
- 位置速度控制 / position & speed control
- 轨迹规划 / trajectory planning

### 通信 / Communication
- 串口收发 / serial RX/TX
- 命令解析执行 / command parsing
- 状态反馈 / status feedback

## 使用说明 / Usage

1) 确保硬件连接正确 / verify hardware wiring
2) 编译并运行程序 / build and run
3) 通过串口发送命令 / send commands over serial
4) 观察响应与图像窗口 / observe behavior & UI windows

## 贡献 / Contributing

欢迎提交 Issue/PR；请遵循常见的 C++ 代码规范。
Issues and PRs are welcome; follow common C++ style guidelines.

## 许可证 / License

本项目采用 MIT 许可证。
Licensed under the MIT License.

## 联系方式 / Contact

如有问题，请创建 Issue。
For questions, please open an issue.
