cmake_minimum_required(VERSION 3.10)
project(scan_land)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OpenCV not found")
endif()

# 包含头文件目录
include_directories(include)

# 收集所有源文件
file(GLOB SOURCES "src/*.cpp" "src/*.c")

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接 OpenCV 库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})