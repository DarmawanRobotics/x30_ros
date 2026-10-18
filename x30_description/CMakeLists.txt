cmake_minimum_required(VERSION 3.0.2)
project(x30_description)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch meshes rviz urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE.deep_robotics_model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_nosetests(test)
  # display.launch needs rviz and joint_state_publisher_gui, which CI does not install
  roslaunch_add_file_check(launch/description.launch)
endif()
