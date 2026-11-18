# inspection_node -> inspection/status (latched): ready, busy, what the last point found.

uint8 IDLE = 0
uint8 INSPECTING = 1
uint8 DISABLED = 2        # enabled: false, stop points pass without inspection

std_msgs/Header header
uint8 state
string site
uint32 points             # inspection points of the site
uint32 targets            # targets over every point
bool thermal_connected    # HIKMICRO PTZ (pan, tilt, zoom, temperatures, snapshots)
bool acoustic_connected   # Hertzino HZ-FA-371 (partial discharge)
string current            # the point being inspected
string last_point
uint8 last_severity
float32 ambient_c         # degC, the reference of the ambient criterion
string message
