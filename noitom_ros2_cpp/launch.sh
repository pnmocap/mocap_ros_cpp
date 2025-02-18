#!/bin/bash
# 检查文件是否存在
if [ -f "./build/noitom_ros2_cpp/noitom_ros2_cpp" ]; then 
	echo "文件存在，正在执行..." 
	./build/noitom_ros2_cpp/noitom_ros2_cpp 
else 
	echo "错误：文件 ./build/noitom_ros2_cpp/noitom_ros2_cpp 不存在。" 
	exit 1 
fi