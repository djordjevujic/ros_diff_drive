"""
Static configuration of the move_to_point module
"""

# Controller frequency in Hertz
controller_freq = 50.0

# Rotation speed limit
rot_speed_limit = 2.0

# Speed limit of moving forward
fwd_speed_limit = 0.7

# Angle error tolerance in degrees
angle_err_tolerance = 0.04

# Distance error tolerance
dist_err_tolerance = 0.005

##  @defgroup pidInit Pre initial values of PID regulators
##  Values of PID parameters before the update from the dynamic reconfigure module.
##  Advice is to keep current values
##  @{

# PID Parameters initialization
KP_ROT = 0.0
TI_ROT = 0.000001
TD_ROT = 0.0
INT_LIMIT_ROT = 0.0
P_ANG_DST = 0.0
P_ANG_THT = 0.0

KP_FWD = 0.0
TI_FWD = 0.000001
TD_FWD = 0.0
P_FWD_DST = 0.0
P_FWD_CUR = 0.0
INT_LIMIT_FWD = 0.0

##  @}

