"""
Static configuration of the move_to_point module
"""

## Controller frequency in Hertz
controller_freq = 50.0

## Rotation speed limit
rot_speed_limit = 2.0

## Speed limit of moving forward
fwd_speed_limit = 0.7

## Angle error tolerance for normal rotation state - in degrees
angle_err_tolerance_rot = 0.1

## Distance error tolerance
dist_err_tolerance = 0.1

## Number of cycles to spend in forward state while the robot is in the distance tolerance region.
## Ensures that robot stops properly, and regulates position even if it reached tolerance region.
## Especially important if we count that inertion will keep robot moving for some small distance even
## if we send 0 as the desired linear velocity value
NO_OF_CYCLES_LIN_MOVEMENT_STOP = 20

## Angle error tolerance for angle fix while moving forwad - in degrees
angle_err_tolerance_fwd = 0.04

##  @defgroup pidInit Pre initial values of PID regulators
##  Values of PID parameters before the update from the dynamic reconfigure module.
##  Advice is to keep current values
##  @{

# PID Parameters initialization

# ROTATION PARAMS

## Rotation KP constant. Overwritten later by dynamic reconfigure module.
KP_ROT = 0.0

## Rotation TI constant. Overwritten later by dynamic reconfigure module.
TI_ROT = 0.000001

## Rotation TD constant. Overwritten later by dynamic reconfigure module.
TD_ROT = 0.0

## Limit of the control value used as protection from the wind-up - used for rotation. Overwritten later by dynamic reconfigure module.
INT_LIMIT_ROT = 0.0

## Gaol angle filter constant. Overwritten later by dynamic reconfigure module.
P_ANG_DST = 0.0

## Current angle filter constant. Overwritten later by dynamic reconfigure module.
P_ANG_THT = 0.0

# MOVE FORWARD PARAMS

## Move forward KP constant. Overwritten later by dynamic reconfigure module.
KP_FWD = 0.0

## Move forward TI constant. Overwritten later by dynamic reconfigure module.
TI_FWD = 0.000001

## Move forward TD constant. Overwritten later by dynamic reconfigure module.
TD_FWD = 0.0

## Gaol distance filter constant. Overwritten later by dynamic reconfigure module.
P_FWD_DST = 0.0

## Current distance filter constant. Overwritten later by dynamic reconfigure module.
P_FWD_CUR = 0.0

## Limit of the control value used as protection from the wind-up - used for moving forward. Overwritten later by dynamic reconfigure module.
INT_LIMIT_FWD = 0.0

##  @}

