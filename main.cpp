#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <cstring>

using namespace cv;
using namespace std;

// 全局变量
int serialPort;
bool running = true;

// 串口通信线程
void* serialThread(void* arg) {
    char buffer[256];
    while (running) {
        if (serialDataAvail(serialPort)) {
            int bytes = read(serialPort, buffer, sizeof(buffer) - 1);
            if (bytes > 0) {
                buffer[bytes] = '\0';
                cout << "收到数据: " << buffer << endl;
                
                // 处理接收到的命令
                if (strstr(buffer, "FORWARD")) {
                    cout << "执行前进命令" << endl;
                    // 这里添加前进控制代码
                } else if (strstr(buffer, "BACKWARD")) {
                    cout << "执行后退命令" << endl;
                    // 这里添加后退控制代码
                } else if (strstr(buffer, "LEFT")) {
                    cout << "执行左转命令" << endl;
                    // 这里添加左转控制代码
                } else if (strstr(buffer, "RIGHT")) {
                    cout << "执行右转命令" << endl;
                    // 这里添加右转控制代码
                } else if (strstr(buffer, "STOP")) {
                    cout << "执行停止命令" << endl;
                    // 这里添加停止控制代码
                }
            }
        }
        usleep(100000); // 100ms延迟
    }
    return NULL;
}

// 图像处理线程
void* imageProcessingThread(void* arg) {
    VideoCapture cap(0); // 打开摄像头
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头" << endl;
        return NULL;
    }
    
    Mat frame;
    while (running) {
        cap >> frame;
        if (frame.empty()) {
            continue;
        }
        
        // 图像处理 - 转换为灰度图
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        
        // 简单的边缘检测
        Mat edges;
        Canny(gray, edges, 100, 200);
        
        // 显示图像 (如果有显示器)
        imshow("原图", frame);
        imshow("边缘检测", edges);
        
        // 简单的目标检测逻辑
        // 这里可以添加更复杂的图像处理算法
        
        if (waitKey(1) == 27) { // ESC键退出
            running = false;
            break;
        }
    }
    
    cap.release();
    destroyAllWindows();
    return NULL;
}

// 电机控制函数
void moveForward() {
    // 控制电机前进
    digitalWrite(0, HIGH);  // 左电机正转
    digitalWrite(1, LOW);
    digitalWrite(2, HIGH);  // 右电机正转
    digitalWrite(3, LOW);
    cout << "机器人前进" << endl;
}

void moveBackward() {
    // 控制电机后退
    digitalWrite(0, LOW);   // 左电机反转
    digitalWrite(1, HIGH);
    digitalWrite(2, LOW);   // 右电机反转
    digitalWrite(3, HIGH);
    cout << "机器人后退" << endl;
}

void turnLeft() {
    // 控制电机左转
    digitalWrite(0, LOW);   // 左电机反转
    digitalWrite(1, HIGH);
    digitalWrite(2, HIGH);  // 右电机正转
    digitalWrite(3, LOW);
    cout << "机器人左转" << endl;
}

void turnRight() {
    // 控制电机右转
    digitalWrite(0, HIGH);  // 左电机正转
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);   // 右电机反转
    digitalWrite(3, HIGH);
    cout << "机器人右转" << endl;
}

void stopMotors() {
    // 停止所有电机
    digitalWrite(0, LOW);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    cout << "机器人停止" << endl;
}

int main() {
    cout << "启动机器人控制系统..." << endl;
    
    // 初始化WiringPi
    if (wiringPiSetup() == -1) {
        cerr << "WiringPi初始化失败" << endl;
        return 1;
    }
    
    // 设置GPIO引脚为输出模式 (电机控制引脚)
    pinMode(0, OUTPUT);  // 左电机引脚1
    pinMode(1, OUTPUT);  // 左电机引脚2
    pinMode(2, OUTPUT);  // 右电机引脚1
    pinMode(3, OUTPUT);  // 右电机引脚2
    
    // 初始化串口
    serialPort = serialOpen("/dev/ttyUSB0", 9600);
    if (serialPort < 0) {
        cerr << "无法打开串口" << endl;
        return 1;
    }
    
    cout << "串口已打开，波特率: 9600" << endl;
    
    // 创建线程
    pthread_t serialThreadId, imageThreadId;
    
    // 启动串口通信线程
    if (pthread_create(&serialThreadId, NULL, serialThread, NULL) != 0) {
        cerr << "创建串口线程失败" << endl;
        return 1;
    }
    
    // 启动图像处理线程
    if (pthread_create(&imageThreadId, NULL, imageProcessingThread, NULL) != 0) {
        cerr << "创建图像处理线程失败" << endl;
        return 1;
    }
    
    cout << "所有线程已启动，系统运行中..." << endl;
    cout << "按Ctrl+C退出程序" << endl;
    
    // 主循环
    while (running) {
        sleep(1);
        
        // 这里可以添加主控制逻辑
        // 例如：根据图像处理结果自动控制机器人
    }
    
    // 等待线程结束
    pthread_join(serialThreadId, NULL);
    pthread_join(imageThreadId, NULL);
    
    // 清理资源
    stopMotors();
    serialClose(serialPort);
    cout << "程序正常退出" << endl;
    
    return 0;
}
