cmake_minimum_required(VERSION 3.0.2)
project(x30_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  # status
  ControlStatus.msg
  DockingStatus.msg
  HealthStatus.msg
  LocalizationStatus.msg
  MappingStatus.msg
  NavigationStatus.msg
  OdometryStatus.msg
  PlannerStatus.msg
  SiteStatus.msg
  TripStatus.msg
  # perception and navigation data
  AvoidanceCommand.msg
  SectorScan.msg
  StopPointEvent.msg
  Waypoint.msg
  WaypointPath.msg
)

add_service_files(
  FILES
  GoTo.srv
  ListSites.srv
  LoadFile.srv
  LoadPath.srv
  LoadSite.srv
  MarkStopPoint.srv
  PlanPath.srv
  SetMode.srv
  SetPath.srv
  StartMapping.srv
)

generate_messages(DEPENDENCIES std_msgs)

# generate_messages() creates the devel space's Python directory at configure time. Under
# catkin_make the environment cache of the build targets (the tests) was taken before it existed,
# and catkin_python_setup() of a later package refreshes the cache only when it creates that
# directory itself: refresh it here, or no test finds a Python package of the workspace.
if(GENERATE_ENVIRONMENT_CACHE_COMMAND)
  safe_execute_process(COMMAND ${GENERATE_ENVIRONMENT_CACHE_COMMAND})
endif()

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
