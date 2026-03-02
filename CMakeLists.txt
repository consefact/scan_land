cmake_minimum_required(VERSION 3.10)
project(scan_land)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  message_generation
  livox_ros_driver
  image_transport
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
)

# 查找 OpenCV 包
find_package(OpenCV 4.2.0 REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OpenCV not found")
endif()

# 包含头文件目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 收集所有源文件
file(GLOB SOURCES "src/*.cpp" "src/*.c")
# cpp compile part
add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
)

#python part
catkin_install_python(PROGRAMS
  scripts/yolo_detector.py   # 新增Python脚本
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
