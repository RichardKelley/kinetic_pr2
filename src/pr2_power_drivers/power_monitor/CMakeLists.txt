# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(power_monitor)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pr2_msgs dynamic_reconfigure)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/PowerMonitor.cfg)

add_executable(power_monitor 
   src/power_monitor.cpp
   src/power_state_estimator.cpp
   src/observation.cpp)

add_dependencies(power_monitor 
   ${power_monitor_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS})

target_link_libraries(power_monitor ${catkin_LIBRARIES})

install(TARGETS power_monitor
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_package()
