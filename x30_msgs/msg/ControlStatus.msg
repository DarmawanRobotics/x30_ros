# control_mux_node -> control/status (latched): who drives, the e-stop and the protective field.

uint8 MODE_AUTO = 0
uint8 MODE_REMOTE = 1

uint8 FIELD_OFF = 0       # not applied in this mode
uint8 FIELD_CLEAR = 1
uint8 FIELD_LIMITED = 2   # slowed down for an obstacle
uint8 FIELD_STOP = 3      # obstacle within the stopping distance
uint8 FIELD_STALE = 4     # no fresh safety scan
uint8 FIELD_BLIND = 5     # the scan reports blocked or unknown sectors

std_msgs/Header header
uint8 mode
string remote_source      # the operator source that drives in MODE_REMOTE
bool estop                # latched e-stop (software or hardware)
string[] estop_reasons    # oldest first
bool armed                # false after a release until the source sent a neutral command
uint8 field
float32 clearance         # m, free distance along the commanded motion (inf: clear)
float32 speed_scale       # 0..1, applied to the commanded speed
