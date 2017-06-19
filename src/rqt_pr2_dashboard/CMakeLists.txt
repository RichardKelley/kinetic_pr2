cmake_minimum_required(VERSION 2.8.3)
project(rqt_pr2_dashboard)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_pr2_dashboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
