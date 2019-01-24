cmake_minimum_required(VERSION 2.8)
project(ceres_zm)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(Ceres REQUIRED )
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ceres_fitting ceres-main.cc)

target_link_libraries(ceres_fitting ${OpenCV_LIBS} ${CERES_LIBRARIES}  )