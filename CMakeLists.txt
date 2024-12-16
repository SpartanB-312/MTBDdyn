cmake_minimum_required(VERSION 3.12)
project(MTBDdyn)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)

# 添加头文件搜索路径
include_directories(${CMAKE_SOURCE_DIR}/src/core)
include_directories(${CMAKE_SOURCE_DIR}/lib)

# 添加库函数源文件
file(GLOB CORE_SOURCES src/core/*.cpp)

# 添加主程序源文件
file(GLOB MAIN_SOURCES src/demos/main.cpp ${CORE_SOURCES})

# 添加主程序可执行文件
add_executable(MTBDdyn ${MAIN_SOURCES})

# 添加 demo1 源文件
file(GLOB DEMO1_SOURCES src/demos/Pendulum.cpp ${CORE_SOURCES})

# 添加 demo1 可执行文件
add_executable(demo1 ${DEMO1_SOURCES})