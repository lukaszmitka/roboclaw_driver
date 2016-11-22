cmake_minimum_required(VERSION 2.8.3)
project(roboclaw_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        )
catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(roboclaw_driver_node src/roboclaw_driver_node.cpp src/ComPortDriver.cpp src/rs232.c src/Roboclaw.cpp src/Roboclaw.h)

target_link_libraries(roboclaw_driver_node
        ${catkin_LIBRARIES}
        )