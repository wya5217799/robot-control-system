# 机器人控制系统

一个基于C++的智能机器人控制系统，集成了图像处理、运动控制和串口通信功能。

## 功能特性

- 🤖 **实时图像处理**: 使用OpenCV进行目标检测和跟踪
- 🎮 **运动控制**: 基于WiringPi的精确电机控制
- 📡 **串口通信**: 高效的数据传输和命令处理
- ⚡ **多线程处理**: 并发执行多个任务以提高性能

## 技术栈

- **编程语言**: C++
- **图像处理**: OpenCV
- **硬件控制**: WiringPi
- **通信**: 串口通信 (Serial)
- **并发**: pthread

## 系统要求

- Linux系统 (推荐树莓派)
- GCC/G++ 编译器
- OpenCV 库 (≥ 3.0)
- WiringPi 库
- pthread 支持

## 编译和运行

### 安装依赖

```bash
# Ubuntu/Debian系统
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libopencv-dev
sudo apt-get install wiringpi

# 或使用包管理器安装OpenCV
sudo apt-get install libopencv-contrib-dev
```

### 编译项目

```bash
# 编译主程序
g++ -o robot_control main.cpp -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lwiringPi -lpthread

# 或使用Makefile
make
```

### 运行

```bash
sudo ./robot_control
```

## 项目结构

```
.
├── main.cpp              # 主程序文件
├── Makefile             # 编译配置
├── README.md            # 项目说明
├── .github/             # GitHub配置
│   └── copilot-instructions.md
└── docs/                # 文档目录
```

## 主要模块

### 图像处理模块
- 实时视频捕获
- 目标检测和识别
- 图像预处理和滤波

### 运动控制模块
- 电机驱动控制
- 位置和速度控制
- 轨迹规划

### 通信模块
- 串口数据收发
- 命令解析和执行
- 状态反馈

## 使用说明

1. 确保硬件连接正确
2. 编译并运行程序
3. 通过串口发送控制命令
4. 观察机器人响应和图像处理结果

## 贡献

欢迎提交问题和改进建议！

## 许可证

本项目采用 MIT 许可证。

## 联系方式

如有问题，请创建 Issue 或联系项目维护者。
