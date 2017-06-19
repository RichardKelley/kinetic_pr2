find_library(LOG4CXX_LIBRARY log4cxx)
find_package(Boost REQUIRED COMPONENTS thread program_options date_time)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(ocean_server server.cpp ocean.cpp)
add_dependencies(ocean_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(ocean_server 
   ${catkin_LIBRARIES}
   ${LOG4CXX_LIBRARY}
   ${Boost_LIBRARIES})

add_executable(battery_check battery_check.cpp ocean.cpp)
add_dependencies(battery_check ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_check
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES})

install(TARGETS ocean_server battery_check
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
