cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

project(HumanCounter)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (HumanCounter HumanCounter.cpp)
target_link_libraries (HumanCounter ${PCL_LIBRARIES})

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=gnu++11")
set(CMAKE_CXX_FLAGS_DEBUG "-std=gnu++11 -g -O0")

find_package(PkgConfig)
pkg_check_modules(LIBOPENCV2 REQUIRED opencv)
include_directories(${LIBOPENCV2_INCLUDE_DIRS})

target_link_libraries(HumanCounter
	 ${LIBOPENCV2_LDFLAGS} ${LIBOPENCV2_LIBRARIES} /usr/lib/hldstof/libtof.a /usr/lib/hldstof/libtof_cv.so rt m pthread stdc++)
