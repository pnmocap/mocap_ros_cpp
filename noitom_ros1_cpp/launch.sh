#!/bin/bash
# check the file exists
if [ -f "./build/noitom_ros1_cpp/noitom_ros1_cpp" ]; then 
	echo "The file exists. Execution is in progress..." 
	./build/noitom_ros1_cpp/noitom_ros1_cpp 
else 
	echo "Error: file ./build/noitom_ros1_cpp/noitom_ros1_cpp not exists" 
	exit 1 
fi
