Header header

float64 heading_angle    	# commanded heading angle 1 meter in front of robot center (in radians)
float64 heading_velocity 	# max servo velocity (in radians per second)
float64 speed       		# speed setpoint of robot (in meters per second)
float64 acceleration    	# max robot acceleration (in meters per second^2)
uint8 	mode        		# drive mode of robot [0 = AUTO (PIVOT & SPIN) / 1 = PIVOT / 2 = SPIN / 3 = SIDEWAYS]

uint8 DRIVE_MODE_AUTO=0
uint8 DRIVE_MODE_PIVOT=1
uint8 DRIVE_MODE_SPIN=2
uint8 DRIVE_MODE_SIDEWAYS=3