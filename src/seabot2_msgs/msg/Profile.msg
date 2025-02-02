std_msgs/Header header

# The current return distance determined for the most recent acoustic measurement.
# Units mm
uint32 distance

# Confidence in the most recent range measurement.
# Unit: %
uint16 	confidence

# The acoustic pulse length during acoustic transmission/activation.
# Unit: us
uint16 	transmit_duration

# The pulse/measurement count since boot.
uint32 	ping_number

# The beginning of the scan region in mm from the transducer.
# Unit: mm
uint32 	scan_start

# The length of the scan region.
# Unit: mm
uint32 	scan_length

# The current gain setting. 0: 0.6, 1: 1.8, 2: 5.5, 3: 12.9, 4: 30.2, 5: 66.1, 6: 144
uint32 	gain_setting

# The length of the proceeding vector field
uint16 	profile_data_length

# An array of return strength measurements taken at regular intervals across the scan region.
# The first element is the closest measurement to the sensor, and the last element is
# the farthest measurement in the scanned range.
uint8[] 	profile_data
