# 硬件连接说明 / Hardware Setup

## GPIO引脚连接 / GPIO Pinout

### 电机控制引脚 / Motor Control Pins
- **左电机**:
  - 引脚0 (WiringPi) -> 电机驱动器 IN1 | Pin 0 -> Driver IN1
  - 引脚1 (WiringPi) -> 电机驱动器 IN2 | Pin 1 -> Driver IN2
  
- **右电机**:
  - 引脚2 (WiringPi) -> 电机驱动器 IN3 | Pin 2 -> Driver IN3
  - 引脚3 (WiringPi) -> 电机驱动器 IN4 | Pin 3 -> Driver IN4

### 串口连接 / Serial
- **USB转串口模块**:
  - 设备路径: `/dev/ttyUSB0`
  - 波特率: 9600 | Baud: 9600
  - 数据位: 8 | Data bits: 8
  - 停止位: 1 | Stop bits: 1
  - 校验位: 无 | Parity: None

### 摄像头连接 / Camera
- **USB摄像头**:
  - 设备编号: 0 (默认) | Device ID: 0 (default)
  - 支持标准 UVC 协议 | UVC compliant

## 电源要求 / Power

- 树莓派: 5V 2.5A | Raspberry Pi: 5V 2.5A
- 电机: 按规格选择 | Motors: per datasheet
- 摄像头: USB供电 | Camera: via USB

## 安全注意事项 / Safety Notes

1) 确保电源电压正确 | Verify correct voltage
2) 检查连线是否牢固 | Check wiring is secure
3) 避免短路 | Avoid shorts
4) 断电后操作接线 | Wire only with power off
