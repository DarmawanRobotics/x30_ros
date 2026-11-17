# site_manager_node -> site/status (latched): the active site and what it holds.

std_msgs/Header header
string site               # empty: none loaded
string[] sites            # every site on the robot
bool has_map
bool mapping              # a mapping session is running
uint32 tags               # recorded fiducials
uint32 routes             # taught routes
string message
