cmake_minimum_required(VERSION 3.0.2)
project(rosbag_record_gui)

find_package(catkin)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/record_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

