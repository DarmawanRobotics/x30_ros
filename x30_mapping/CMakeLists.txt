cmake_minimum_required(VERSION 3.0.2)
project(x30_mapping)

find_package(catkin REQUIRED COMPONENTS x30_utils)
catkin_python_setup()
catkin_package()

catkin_install_python(
  PROGRAMS scripts/site_manager_node scripts/route_recorder_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  x30_add_pytests(test)
endif()
