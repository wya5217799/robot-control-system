# 机器人控制系统的 Makefile | Makefile for Robot Control System

# 编译器设置 | Compiler settings
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2

# 库链接 | Linked libraries
LIBS = -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui -lwiringPi -lpthread

# 目标文件 | Target
TARGET = robot_control
SOURCES = main.cpp

# 包含路径 (如需要) | Include paths (if needed)
INCLUDES = -I/usr/include/opencv4

# 默认目标 | Default target
all: $(TARGET)

# 编译规则 | Build rule
$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCES) $(LIBS)

# 清理 | Clean
clean:
	rm -f $(TARGET)

# 安装依赖 (仅供参考) | Install dependencies (reference)
install-deps:
	sudo apt-get update
	sudo apt-get install build-essential
	sudo apt-get install libopencv-dev
	sudo apt-get install wiringpi

# 运行 | Run
run: $(TARGET)
	sudo ./$(TARGET)

.PHONY: all clean install-deps run
