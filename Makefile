# Makefile for Robot Control System

# 编译器设置
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2

# 库链接
LIBS = -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lwiringPi -lpthread

# 目标文件
TARGET = robot_control
SOURCES = main.cpp

# 包含路径 (如需要)
INCLUDES = -I/usr/include/opencv4

# 默认目标
all: $(TARGET)

# 编译规则
$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCES) $(LIBS)

# 清理
clean:
	rm -f $(TARGET)

# 安装依赖 (仅供参考)
install-deps:
	sudo apt-get update
	sudo apt-get install build-essential
	sudo apt-get install libopencv-dev
	sudo apt-get install wiringpi

# 运行
run: $(TARGET)
	sudo ./$(TARGET)

.PHONY: all clean install-deps run
