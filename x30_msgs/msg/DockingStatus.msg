# docking_node -> docking/status (latched): docking onto a charger marked by a fiducial tag.

uint8 IDLE = 0
uint8 SEARCHING = 1       # waiting for the dock tag to come into view
uint8 CENTERING = 2       # turning / sliding until the tag is centred on the approach line
uint8 APPROACHING = 3     # walking toward the dock, kept centred, until the goal distance
uint8 SETTLING = 4        # standing still, then checking the final pose against the tag
uint8 BACKING_OFF = 5     # moving away from the dock before another attempt
uint8 SITTING = 6         # the sit hooks run
uint8 CHARGING = 7        # the charge hooks run, then waiting for the battery to charge
uint8 DOCKED = 8          # sitting on the dock, charging
uint8 UNDOCKING = 9       # stop-charge and stand hooks, then backing away from the dock
uint8 FAILED = 10

std_msgs/Header header
uint8 state
string[] pause_reasons    # disarmed (the control mux has not armed navigation yet)
bool tag_visible          # a fresh detection of the dock tag
float32 tag_age           # s since the last detection (inf: never)
float32 remaining         # m still to walk toward the dock (goal frame)
float32 lateral           # m, robot beside the approach line (+left of the line)
float32 yaw_error         # rad, robot heading minus the goal heading
uint32 attempt            # 1 for the first approach, +1 for every retry
bool charging             # the battery reports charging (or full)
string message
