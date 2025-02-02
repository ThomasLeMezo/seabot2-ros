std_msgs/Header header
uint16 waypoint_id
uint16 waypoint_length
float64 time_to_next_waypoint

# Regulation mode
uint16 MODE_IDLE=0
uint16 MODE_DEPTH_PROFILE=1
uint16 MODE_SEAFLOOR_LANDING=2
uint16 MODE_TEMPERATURE_KEEPING=3
uint16 MODE_TEMPERATURE_PROFILE=4
uint16 MODE_GNSS_PROFILE=5
uint16 MODE_STOP=6
uint16 mode

# Mission state
uint16 MODE_NOT_STARTED=0
uint16 MODE_RUNNING=1
uint16 MODE_ENDING=2
uint16 state
