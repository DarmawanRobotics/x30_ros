cmake_minimum_required(VERSION 3.0.2)
project(x30_driver)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/driver_node test/fake_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
  roslaunch_add_file_check(launch)
  add_rostest(test/smoke.test)
endif()
