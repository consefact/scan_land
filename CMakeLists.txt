cmake_minimum_required(VERSION 3.12)
project(scan_land)

find_package(OpenCV REQUIRED) 

set(sc_executable_name ${PROJECT_NAME})
aux_source_directory(src sc_executable_source)
add_executable(${sc_executable_name} ${sc_executable_source})
target_include_directories(${sc_executable_name} BEFORE PUBLIC include)
target_link_libraries(${sc_executable_name}  ${OpenCV_LIBS})

