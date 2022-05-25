cmake_minimum_required(VERSION 3.2)

project(test)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

find_package (Eigen3 3.4 REQUIRED NO_MODULE)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

include_directories(src)

set(GLOG_LIBRARIES /usr/local/lib/libglog.so)

add_executable(t src/test.cpp)
target_link_libraries(t Eigen3::Eigen ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} pthread)

add_executable(m src/main.cpp)
target_link_libraries(m Eigen3::Eigen ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} pthread)
