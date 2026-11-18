# odometry_monitor_node -> odometry/trip (latched): distance and speeds since the last reset
# (odometry/trip_reset), from the LiDAR-inertial odometry; kept across restarts.

std_msgs/Header header
time since                # when it was last reset
float64 distance          # m travelled
float64 moving_sec        # s spent moving
float32 speed             # m/s now (over the last second)
float32 average           # m/s while moving
float32 max_speed         # m/s, the highest speed since the reset
