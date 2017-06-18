cmake_minimum_required(VERSION 2.8.3)
project(shape_tools)

find_package(catkin COMPONENTS shape_msgs visualization_msgs REQUIRED)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/shape_to_marker.cpp
  src/shape_extents.cpp)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
