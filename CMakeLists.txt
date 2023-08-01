# ***************************************************************** #
cmake_minimum_required(VERSION 3.0)
project(ceres_example)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3 -std=c++17")

# 寻找Ceres Solver库
find_package(Ceres REQUIRED)
Find_Package(Eigen3 REQUIRED
find_package(Sophus REQUIRED))

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})
include_directories("/home/meta_iot/radar_odo_local_optimization/include")

option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" ON)
if(USE_UBUNTU_20)
    message("You are using Ubuntu 20.04, fmt::fmt will be linked")
    find_package(fmt REQUIRED)
    set(FMT_LIBRARIES fmt::fmt)
endif()


# add_library("/home/meta_iot/radar_odo_local_optimization/include/error.h")

# 添加可执行文件
add_executable(readData src/readData.cc)

add_executable(curveFitting src/curveFitting.cc)

add_executable(ceres_with_eigen src/ceres_with_eigen.cc)

add_executable(ceres_slover_local_optimization src/ceres_slover_local_optimization.cc src/utils.cc)

add_executable(g2o_solver_local_optimization src/g2o_solver_local_optimization.cc)
# 链接Ceres Solver库
target_link_libraries(curveFitting ${CERES_LIBRARIES})

target_link_libraries(ceres_with_eigen ${CERES_LIBRARIES})

# target_link_libraries(ceres_slover_local_optimization ${CERES_LIBRARIES})
 
target_link_libraries(readData ${EIGEN3_INCLUDE_DIRS} fmt::fmt)

target_link_libraries(g2o_solver_local_optimization ${EIGEN3_INCLUDE_DIRS} fmt::fmt g2o_core g2o_stuff)

target_include_directories(ceres_slover_local_optimization
PRIVATE
${CERES_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
include)

target_link_libraries(ceres_slover_local_optimization
    PRIVATE
    ${CERES_LIBRARIES} ${FMT_LIBRARIES}
)