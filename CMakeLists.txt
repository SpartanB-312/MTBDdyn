cmake_minimum_required(VERSION 3.12)
project(MTBDdyn)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)

# 添加源文件
file(GLOB SOURCES src/*.cpp)

# 调试输出源文件列表
#message("SOURCES: ${SOURCES}")

# 添加头文件搜索路径
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(MTBDdyn ${SOURCES})