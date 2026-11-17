# path_follower_node -> navigation/stop_point at each stop point of a route.

uint8 ARRIVED = 0
uint8 INSPECTED = 1
uint8 DEPARTED = 2
uint8 SKIPPED = 3

std_msgs/Header header
uint8 event
uint32 index              # waypoint index of the stop point
float64 x                 # m, map frame
float64 y
float64 yaw               # rad
float32 dwell_sec
string message
