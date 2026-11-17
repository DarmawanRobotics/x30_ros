# planner_node -> planner/status (latched).

uint8 IDLE = 0
uint8 PLANNING = 1
uint8 FOLLOWING = 2
uint8 REPLANNING = 3
uint8 ARRIVED = 4
uint8 FAILED = 5
uint8 CANCELLED = 6

std_msgs/Header header
uint8 state
float64 goal_x            # m, map frame
float64 goal_y
float64 goal_yaw          # rad
float32 path_length       # m
float32 remaining         # m
uint32 replans
string message
