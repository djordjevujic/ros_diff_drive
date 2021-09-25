"""
Contains debug variables and publisher/subscriber definitions
"""

import rospy
from std_msgs.msg import Float32

# Debug publishers
#   Rotation
pub_dbg_angle_err = rospy.Publisher("debug/angle_err", Float32, queue_size = 5)
pub_dbg_theta = rospy.Publisher("debug/theta", Float32, queue_size=5)
pub_dbg_theta_filtr = rospy.Publisher("debug/theta_filtr", Float32, queue_size=5)
pub_dbg_ang_to_goal = rospy.Publisher("debug/angle_to_goal", Float32, queue_size=5)
pub_dbg_ang_to_goal_filtr = rospy.Publisher("debug/angle_to_goal_filtr", Float32, queue_size=5)
pub_dbg_rot = rospy.Publisher("debug/rot_vel", Float32, queue_size=5)

#   Moving forward
pub_dbg_distance = rospy.Publisher("/debug/distance", Float32, queue_size = 5)
pub_dbg_distance_filtr = rospy.Publisher("/debug/dist_filtr", Float32, queue_size = 5)
pub_dbg_dist_to_goal = rospy.Publisher("/debug/dist_to_goal", Float32, queue_size = 5)
pub_dbg_dist_to_goal_filtr = rospy.Publisher("/debug/dist_to_goal_filtr", Float32, queue_size = 5)
pub_dbg_fwd = rospy.Publisher("/debug/dist_velocity", Float32, queue_size = 5)
pub_dbg_fwd_rot = rospy.Publisher("/debug/fwd_rot", Float32, queue_size = 5)
pub_dbg_fwd_rot_vel = rospy.Publisher("/debug/fwd_rot_vel", Float32, queue_size = 5)
