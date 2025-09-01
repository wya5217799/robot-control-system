# API参考文档 / API Reference

## 串口命令 / Serial Commands

### 运动控制命令 / Motion Control

| 命令 / Command | 功能 / Function | 示例 / Example |
|---|---|---|
| `FORWARD` | 机器人前进 / Move forward | `FORWARD` |
| `BACKWARD` | 机器人后退 / Move backward | `BACKWARD` |
| `LEFT` | 机器人左转 / Turn left | `LEFT` |
| `RIGHT` | 机器人右转 / Turn right | `RIGHT` |
| `STOP` | 机器人停止 / Stop | `STOP` |

### 响应格式 / Response

系统会在控制台输出相应的执行信息。
Execution logs are printed to console.

## 函数接口 / Function Interfaces

### 电机控制函数 / Motor Control Functions

#### `moveForward()`
控制机器人前进 | Move robot forward
- 参数 / Params: 无 / none
- 返回值 / Returns: 无 / void

#### `moveBackward()`
控制机器人后退 | Move robot backward
- 参数 / Params: 无 / none
- 返回值 / Returns: 无 / void

#### `turnLeft()`
控制机器人左转 | Turn robot left
- 参数 / Params: 无 / none
- 返回值 / Returns: 无 / void

#### `turnRight()`
控制机器人右转 | Turn robot right
- 参数 / Params: 无 / none
- 返回值 / Returns: 无 / void

#### `stopMotors()`
停止所有电机 | Stop all motors
- 参数 / Params: 无 / none
- 返回值 / Returns: 无 / void

### 线程函数 / Thread Functions

#### `serialThread(void* arg)`
串口通信处理线程 | Serial handling thread
- 参数 / Params: void* arg (未使用 / unused)
- 返回值 / Returns: void*
- 功能 / Purpose: 持续监听串口数据并处理命令 | listen for serial data and handle commands

#### `imageProcessingThread(void* arg)`
图像处理线程 | Image processing thread
- 参数 / Params: void* arg (未使用 / unused)
- 返回值 / Returns: void*
- 功能 / Purpose: 捕获和处理摄像头图像 | capture and process camera frames

## 配置参数 / Configuration

### 串口配置 / Serial
- 设备 / Device: `/dev/ttyUSB0`
- 波特率 / Baud: 9600
- 数据格式 / Data: 8N1

### 摄像头配置 / Camera
- 设备ID / Device ID: 0
- 分辨率 / Resolution: 默认 / default
- 帧率 / FPS: 默认 / default

### GPIO引脚配置 / GPIO Pins
- 左电机 / Left Motor: 引脚 0, 1
- 右电机 / Right Motor: 引脚 2, 3
