"""
Contains debug variables and publisher/subscriber definitions
"""

import rospy
from std_msgs.msg import Float32

# Debug publishers
#   Rotation

## Angle error values publisher
pub_dbg_angle_err = rospy.Publisher("debug/angle_err", Float32, queue_size = 5)

## Current angle publisher
pub_dbg_theta = rospy.Publisher("debug/theta", Float32, queue_size=5)

## Publisher for filtered value of the current value
pub_dbg_theta_filtr = rospy.Publisher("debug/theta_filtr", Float32, queue_size=5)

## Publisher for the goal angle
pub_dbg_ang_to_goal = rospy.Publisher("debug/angle_to_goal", Float32, queue_size=5)

## Publisher for the filtered value of the goal angle
pub_dbg_ang_to_goal_filtr = rospy.Publisher("debug/angle_to_goal_filtr", Float32, queue_size=5)

## Publisher for the desired rotation velocity
pub_dbg_rot = rospy.Publisher("debug/rot_vel", Float32, queue_size=5)

# Publisher for theta unwrapped
pub_dbg_theta_uw = rospy.Publisher("/debug/theta_uw", Float32, queue_size=5)

# Publisher for angle to goal unwrapped
pub_dbg_ang_to_goal_uw = rospy.Publisher("/debug/angle_to_goal_uw", Float32, queue_size=5)

#   Moving forward

## Current distance publisher
pub_dbg_distance = rospy.Publisher("/debug/distance", Float32, queue_size = 5)

## Filtered distance publisher
pub_dbg_distance_filtr = rospy.Publisher("/debug/dist_filtr", Float32, queue_size = 5)

## Goal distance publisher
pub_dbg_dist_to_goal = rospy.Publisher("/debug/dist_to_goal", Float32, queue_size = 5)

## Filtered goal distance publisher
pub_dbg_dist_to_goal_filtr = rospy.Publisher("/debug/dist_to_goal_filtr", Float32, queue_size = 5)

## Distance velocity publisher
pub_dbg_fwd = rospy.Publisher("/debug/dist_velocity", Float32, queue_size = 5)

## Angle error during moving forward - publisher
pub_dbg_fwd_rot = rospy.Publisher("/debug/fwd_rot", Float32, queue_size = 5)

## Rotation velocity command during moving forward - publisher
pub_dbg_fwd_rot_vel = rospy.Publisher("/debug/fwd_rot_vel", Float32, queue_size = 5)
