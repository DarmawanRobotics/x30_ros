# grid_mapper_node -> mapping/status (latched).

std_msgs/Header header
bool active               # scans are being integrated
string site
uint32 scans              # integrated in this session
uint32 rejected           # skipped: jumps, corrections in flight, poor matches
float32 resolution        # m per cell
uint32 width              # cells
uint32 height             # cells
float32 known_area_m2     # free + occupied
string message
