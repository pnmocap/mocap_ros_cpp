cmake_minimum_required(VERSION 3.8)
project(noitom_ros1_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs roslib)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
    include
	include/noitom_ros1_cpp
    ${catkin_INCLUDE_DIRS}
)

# Add executable
add_executable(noitom_ros1_cpp src/robot_api_demo.cpp)

# Link libraries
target_link_libraries(noitom_ros1_cpp
    ${catkin_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/libMocapApi.so
)
set(CATKIN_PACKAGE_BIN_DESTINATION bin)

# Install executable
install(TARGETS noitom_ros1_cpp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Catkin package configuration
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs roslib
)

# Add warnings suppression if using GCC or Clang
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-w)
#endif()