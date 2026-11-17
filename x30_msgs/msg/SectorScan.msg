# Nearest obstacle per angular sector (REP-103: +CCW from x), from lidar_sector_node.
# Sector i is centred at angle_min + (i + 0.5) * angle_increment.
# ranges[i] >= range_max: nothing within range_max. ranges[i] <= BLOCKED or NaN: blocked or
# unknown (blind sensor, no data); consumers must treat it as an obstacle at the robot.

float32 BLOCKED = 0.0

std_msgs/Header header
float32 angle_min         # rad
float32 angle_increment   # rad
float32 range_max         # m
float32[] ranges          # m
