cmake_minimum_required(VERSION 3.12)
project(MTBDdyn)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)

# 添加头文件搜索路径
include_directories(${CMAKE_SOURCE_DIR}/src/core)
include_directories(${CMAKE_SOURCE_DIR}/lib)

# 添加库函数源文件
file(GLOB CORE_SOURCES src/core/*.cpp)

# 添加demo/SingleBody源文件
file(GLOB DEMO_SINGLEBODY_SOURCES src/demos/SingleBody.cpp ${CORE_SOURCES})

# 添加 demo/SingleBody 可执行文件
add_executable(Demo_SingleBody ${DEMO_SINGLEBODY_SOURCES})

# 添加 demo/Pendulum 源文件
file(GLOB DEMO_PENDULUM_SOURCES src/demos/Pendulum.cpp ${CORE_SOURCES})

# 添加 demo/Pendulum 可执行文件
add_executable(Demo_Pendulum ${DEMO_PENDULUM_SOURCES})