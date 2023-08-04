# ***************************************************************** #
cmake_minimum_required(VERSION 3.0)
project(ceres_example)

# # 设置C++标准
set(CMAKE_BUILD_TYPE "Debug")

# 寻找Ceres Solver库
find_Package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package (glog 0.6.0 REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})
include_directories("/home/wsp/Desktop/programm/radar_odo_local_optimization/include")

option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" ON)
if(USE_UBUNTU_20)
    message("You are using Ubuntu 20.04, fmt::fmt will be linked")
    find_package(fmt REQUIRED)
    set(FMT_LIBRARIES fmt::fmt)
endif()


# 添加可执行文件
add_executable(readData src/readData.cc)

add_executable(g2o_solver_local_optimization src/g2o_solver_local_optimization.cc)
# 链接Ceres Solver库

target_link_libraries(readData fmt::fmt)

target_link_libraries(g2o_solver_local_optimization glog::glog fmt::fmt g2o_core g2o_stuff)