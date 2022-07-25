cmake_minimum_required(VERSION 3.2)

project(test)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "-O2 -g")

#find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package (Eigen3 3.4 REQUIRED)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# link openblas
set(BLA_VENDER OpenBLAS)
find_package(BLAS REQUIRED)
if(BLAS_FOUND)
    message("OpenBLAS found.")
    include_directories(${BLAS_INCLUDE_DIRS})
    message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
    message(STATUS "BLAS LIBRARIES:    " ${BLAS_LIBRARIES})
endif(BLAS_FOUND)

include_directories(src)

#set(GLOG_LIBRARIES /usr/local/lib/libglog.so)
set(GLOG_LIBRARIES glog::glog)

#add_definitions(-DEIGEN_USE_LAPACKE_STRICT)
#add_compile_definitions(EIGEN_USE_LAPACK EIGEN_USE_BLAS)
#add_definitions(-DEIGEN_USE_BLAS -DEIGEN_USE_LAPACKE)
# add_definitions(-DENABLE_LOG)
add_definitions(-DENABLE_PLOT)

add_executable(m src/main.cpp)
target_link_libraries(m Eigen3::Eigen ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} pthread ${BLAS_LIBRARIES})

add_executable(t src/test.cpp)
target_link_libraries(t Eigen3::Eigen ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} pthread ${BLAS_LIBRARIES})
#add_executable(t src/test.cpp)
#target_link_libraries(t Eigen3::Eigen ${GTEST_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} pthread ${BLAS_LIBRARIES})
