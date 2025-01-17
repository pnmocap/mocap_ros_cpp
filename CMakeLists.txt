cmake_minimum_required(VERSION 3.8)
project(noitom_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(include)
include_directories(include/noitom_cpp)  # 添加MocapApi头文件的路径

# Add executable
add_executable(noitom_cpp src/robot_api_demo.cpp)

# Link libraries
ament_target_dependencies(noitom_cpp rclcpp sensor_msgs nlohmann_json)
target_link_libraries(noitom_cpp ${CMAKE_CURRENT_SOURCE_DIR}/libMocapApi.so)
# Install executable
install(TARGETS
	  noitom_cpp
	)
ament_package()
