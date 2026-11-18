# obstacle_avoidance_node -> navigation/avoidance: how reactive avoidance bends the command.

std_msgs/Header header
float32 heading_offset    # rad, rotate the commanded direction (+left)
float32 yaw_rate_bias     # rad/s, added to the commanded yaw rate (+left)
float32 lateral_bias      # m/s, sideways push (+left), omni robots only
float32 velocity_scale    # 0..1 multiplier on the commanded speed
bool emergency            # obstacle inside the emergency cone: stop
