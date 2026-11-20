cmake_minimum_required(VERSION 3.0.2)
project(x30_recorder)

find_package(catkin REQUIRED COMPONENTS x30_utils)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/recorder_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/recorder.launch)
  x30_add_pytests(test)
endif()
