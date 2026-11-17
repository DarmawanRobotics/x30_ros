# path_follower_node -> navigation/status.

uint8 INACTIVE = 0
uint8 JOINING = 1         # walking to the path
uint8 FOLLOWING = 2
uint8 APPROACHING = 3     # final x, y, yaw approach to a stop point or the goal
uint8 DWELLING = 4        # waiting at a stop point
uint8 PAUSED = 5          # see pause_reasons
uint8 BLOCKED = 6
uint8 OFF_PATH = 7
uint8 GOAL_REACHED = 8
uint8 FAILED = 9

std_msgs/Header header
uint8 state
string[] pause_reasons    # operator, estop, remote, localization, battery
uint32 index              # current waypoint
uint32 count              # waypoints in the path
float32 remaining         # m along the path
float32 cross_track       # m, signed (+left)
string message
