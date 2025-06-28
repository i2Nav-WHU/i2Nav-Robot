cmake_minimum_required(VERSION 3.5)
project(insprobe_msgs)

set(CMAKE_AUTOMOC ON)

add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )

# Generate messages in the 'msg' folder
add_message_files(
        FILES
        RangerOdometer.msg
        Imu.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
