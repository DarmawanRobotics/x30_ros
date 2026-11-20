# localization_node -> localization/status (latched).

uint8 NONE = 0            # no map -> odom yet
uint8 LOCALIZED = 1
uint8 UNCERTAIN = 2       # localized, but the uncertainty grew past the limits

std_msgs/Header header
uint8 state
string source             # last correction: tag:<frame>, pose_estimate, origin
float32 sigma_xy          # m, 1-sigma position uncertainty (largest axis)
float32 sigma_yaw         # rad, 1-sigma heading uncertainty
float32 since_correction  # s since the last accepted correction
string message
