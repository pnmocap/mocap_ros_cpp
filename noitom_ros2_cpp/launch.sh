#!/bin/bash
# check the file exists
if [ -f "./build/noitom_ros2_cpp/noitom_ros2_cpp" ]; then 
	echo "The file exists. Execution is in progress..." 
	./build/noitom_ros2_cpp/noitom_ros2_cpp 
else 
	echo "Error: File ./build/noitom_ros2_cpp/noitom_ros2_cpp not exists" 
	exit 1 
fi