# health_monitor_node -> health/status (latched): the overall state and what it asks for.

uint8 OK = 0
uint8 WARN = 1
uint8 ERROR = 2

std_msgs/Header header
uint8 level
string[] issues           # one line per problem
float32 speed_scale       # 0..1 speed limit requested from the control mux (1: none)
