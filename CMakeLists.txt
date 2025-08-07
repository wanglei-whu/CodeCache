cmake_minimum_required(VERSION 3.5.1)
project(rthpva)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-pthread -std=c++14 -pipe -O2 -Wall -Wextra -fPIC -Wno-unused-result")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
set(CMAKE_INSTALL_PREFIX /opt/rthpva)

add_definitions(-DBOOST_EXCEPTION_DISABLE
                -DBOOST_GEOMETRY_EXPERIMENTAL_ENABLE_INITIALIZER_LIST
                -DBOOST_GEOMETRY_OVERLAY_NO_THROW
                -DEIGEN_NO_DEBUG
                -DPCL_NO_PRECOMPILE
                -DROSCPP_USE_TCP_NODELAY
                -DCAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(PCL REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(lidar_inertial_odometry)
