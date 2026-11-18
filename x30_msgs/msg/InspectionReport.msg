# inspection_node -> inspection/report: everything measured at one inspection point.

uint8 DONE = 0            # every target measured
uint8 PARTIAL = 1         # something could not be measured (a camera missing, a timeout)
uint8 FAILED = 2          # nothing could be measured
uint8 NO_TARGETS = 3      # no inspection point here: nothing to do

std_msgs/Header header
string site
string point              # inspection point name
float64 x                 # m, map frame: where the robot stood
float64 y
uint8 state
uint8 severity            # the highest severity of the findings (InspectionFinding.NORMAL ...)
InspectionFinding[] findings
string directory          # the report on the robot: report.json and the images
float32 duration_sec
string message
