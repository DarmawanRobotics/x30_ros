cmake_minimum_required(VERSION 3.0.2)
project(x30_odometry)

find_package(catkin REQUIRED COMPONENTS x30_utils)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/odometry_monitor_node scripts/wait_still
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  # FAST-LIO is built from source on the robot, not here: check the launch without it
  roslaunch_add_file_check(launch/odometry.launch method:=external)
  x30_add_pytests(test)
endif()
