cmake_minimum_required(VERSION 2.8.3)
project(ocean_battery_driver)

find_package(catkin REQUIRED COMPONENTS roscpp pr2_msgs diagnostic_updater diagnostic_msgs)

catkin_package()

add_subdirectory(src)
