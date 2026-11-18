# One result of an inspection target: a thermal criterion or an acoustic detection.

uint8 NORMAL = 0
uint8 ATTENTION = 1       # possible deficiency: look again on the next round
uint8 WARNING = 2         # probable deficiency: plan a repair
uint8 CRITICAL = 3        # major deficiency: act now

string target             # target name (inspection plan)
string sensor             # thermal | acoustic
string criterion          # absolute | ambient | similar | discharge
string label              # thermal: the region; acoustic: the discharge type
uint8 severity
float32 value             # thermal: hottest temperature (degC); acoustic: intensity
float32 reference         # thermal: ambient or the similar components (degC); acoustic: 0
float32 delta             # thermal: value - reference (degC); acoustic: 0
string unit               # degC | dB
string message
string[] files            # evidence on the robot: thermal and visible images, sound map, PRPD
