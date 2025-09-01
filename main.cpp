/*
 * 机器人控制系统 | Robot Control System
 * ------------------------------------------------------------
 * 功能概述 | Overview:
 * - 实时图像处理（OpenCV） | Real-time image processing (OpenCV)
 * - GPIO电机控制（WiringPi） | GPIO motor control (WiringPi)
 * - 串口通信 | Serial communication
 * - 多线程（pthread） | Multi-threading (pthread)
 *
 * 平台与依赖 | Platform & Dependencies:
 * - 推荐在树莓派/Linux环境运行 | Targeted for Raspberry Pi/Linux
 * - 需要安装 OpenCV / WiringPi / pthread | Requires OpenCV / WiringPi / pthread
 *
 * 构建 | Build:
 * - 使用随附的 Makefile，或参考 README | Use provided Makefile or README
 */

#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <cstring>

using namespace cv;
using namespace std;

// 全局变量 | Globals
int serialPort;
bool running = true;

// 串口通信线程 | Serial communication thread
void* serialThread(void* arg) {
    char buffer[256];
    while (running) {
        if (serialDataAvail(serialPort)) {
            int bytes = read(serialPort, buffer, sizeof(buffer) - 1);
            if (bytes > 0) {
                buffer[bytes] = '\0';
                cout << "收到数据 / Received data: " << buffer << endl;
                
                // 处理接收到的命令 | Handle incoming commands
                if (strstr(buffer, "FORWARD")) {
                    cout << "执行前进命令 / Execute FORWARD" << endl;
                    // 这里添加前进控制代码 | Add forward control logic here
                } else if (strstr(buffer, "BACKWARD")) {
                    cout << "执行后退命令 / Execute BACKWARD" << endl;
                    // 这里添加后退控制代码 | Add backward control logic here
                } else if (strstr(buffer, "LEFT")) {
                    cout << "执行左转命令 / Execute LEFT" << endl;
                    // 这里添加左转控制代码 | Add turn-left control logic here
                } else if (strstr(buffer, "RIGHT")) {
                    cout << "执行右转命令 / Execute RIGHT" << endl;
                    // 这里添加右转控制代码 | Add turn-right control logic here
                } else if (strstr(buffer, "STOP")) {
                    cout << "执行停止命令 / Execute STOP" << endl;
                    // 这里添加停止控制代码 | Add stop control logic here
                }
            }
        }
        usleep(100000); // 100ms延迟 | 100ms delay
    }
    return NULL;
}

// 图像处理线程 | Image processing thread
void* imageProcessingThread(void* arg) {
    VideoCapture cap(0); // 打开摄像头 | Open default camera
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头 / Failed to open camera" << endl;
        return NULL;
    }
    
    Mat frame;
    while (running) {
        cap >> frame;
        if (frame.empty()) {
            continue;
        }
        
        // 图像处理 - 转换为灰度图 | Convert to grayscale
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        
        // 简单的边缘检测 | Simple edge detection
        Mat edges;
        Canny(gray, edges, 100, 200);
        
        // 显示图像 (如果有显示器) | Show images (if display available)
        imshow("原图 / Original", frame);
        imshow("边缘检测 / Edges", edges);
        
        // 简单的目标检测逻辑 | Placeholder for target detection
        // 这里可以添加更复杂的图像处理算法 | You can add more advanced algorithms here
        
        if (waitKey(1) == 27) { // ESC键退出 | Exit on ESC
            running = false;
            break;
        }
    }
    
    cap.release();
    destroyAllWindows();
    return NULL;
}

// 电机控制函数 | Motor control functions
void moveForward() {
    // 控制电机前进 | Drive forward
    digitalWrite(0, HIGH);  // 左电机正转 | Left motor forward
    digitalWrite(1, LOW);
    digitalWrite(2, HIGH);  // 右电机正转 | Right motor forward
    digitalWrite(3, LOW);
    cout << "机器人前进 / Moving forward" << endl;
}

void moveBackward() {
    // 控制电机后退 | Drive backward
    digitalWrite(0, LOW);   // 左电机反转 | Left motor backward
    digitalWrite(1, HIGH);
    digitalWrite(2, LOW);   // 右电机反转 | Right motor backward
    digitalWrite(3, HIGH);
    cout << "机器人后退 / Moving backward" << endl;
}

void turnLeft() {
    // 控制电机左转 | Turn left
    digitalWrite(0, LOW);   // 左电机反转 | Left motor backward
    digitalWrite(1, HIGH);
    digitalWrite(2, HIGH);  // 右电机正转 | Right motor forward
    digitalWrite(3, LOW);
    cout << "机器人左转 / Turning left" << endl;
}

void turnRight() {
    // 控制电机右转 | Turn right
    digitalWrite(0, HIGH);  // 左电机正转 | Left motor forward
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);   // 右电机反转 | Right motor backward
    digitalWrite(3, HIGH);
    cout << "机器人右转 / Turning right" << endl;
}

void stopMotors() {
    // 停止所有电机 | Stop all motors
    digitalWrite(0, LOW);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    cout << "机器人停止 / Stopped" << endl;
}

int main() {
    cout << "启动机器人控制系统 / Starting Robot Control System..." << endl;
    
    // 初始化WiringPi | Init WiringPi
    if (wiringPiSetup() == -1) {
        cerr << "WiringPi初始化失败 / Failed to initialize WiringPi" << endl;
        return 1;
    }
    
    // 设置GPIO引脚为输出模式 (电机控制引脚) | Configure GPIO pins as outputs (motor control)
    pinMode(0, OUTPUT);  // 左电机引脚1 | Left motor pin 1
    pinMode(1, OUTPUT);  // 左电机引脚2 | Left motor pin 2
    pinMode(2, OUTPUT);  // 右电机引脚1 | Right motor pin 1
    pinMode(3, OUTPUT);  // 右电机引脚2 | Right motor pin 2
    
    // 初始化串口 | Open serial port
    serialPort = serialOpen("/dev/ttyUSB0", 9600);
    if (serialPort < 0) {
        cerr << "无法打开串口 / Failed to open serial port" << endl;
        return 1;
    }
    
    cout << "串口已打开，波特率: 9600 / Serial opened at 9600 baud" << endl;
    
    // 创建线程 | Create threads
    pthread_t serialThreadId, imageThreadId;
    
    // 启动串口通信线程 | Start serial thread
    if (pthread_create(&serialThreadId, NULL, serialThread, NULL) != 0) {
        cerr << "创建串口线程失败 / Failed to create serial thread" << endl;
        return 1;
    }
    
    // 启动图像处理线程 | Start image processing thread
    if (pthread_create(&imageThreadId, NULL, imageProcessingThread, NULL) != 0) {
        cerr << "创建图像处理线程失败 / Failed to create image processing thread" << endl;
        return 1;
    }
    
    cout << "所有线程已启动，系统运行中 / All threads started, system running..." << endl;
    cout << "按 Ctrl+C 退出程序 / Press Ctrl+C to exit" << endl;
    
    // 主循环 | Main loop
    while (running) {
        sleep(1);
        
        // 这里可以添加主控制逻辑 | Add main control logic here
        // 例如：根据图像处理结果自动控制机器人 | e.g., auto control based on vision results
    }
    
    // 等待线程结束 | Join threads
    pthread_join(serialThreadId, NULL);
    pthread_join(imageThreadId, NULL);
    
    // 清理资源 | Cleanup
    stopMotors();
    serialClose(serialPort);
    cout << "程序正常退出 / Exiting cleanly" << endl;
    
    return 0;
}
