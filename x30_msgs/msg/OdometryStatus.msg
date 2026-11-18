# odometry_monitor_node -> odometry/status (latched).

uint8 INITIALIZING = 0
uint8 OK = 1
uint8 DEGRADED = 2        # still usable, e.g. speed disagrees with the robot odometry
uint8 LOST = 3            # stale or jumping, do not drive on it

std_msgs/Header header
uint8 state
float32 speed             # m/s, estimator
float32 tilt              # rad, roll / pitch of body in odom while standing still
string message
