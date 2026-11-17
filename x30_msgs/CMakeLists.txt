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

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
