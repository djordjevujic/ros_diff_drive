#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import sys

roll = pitch = yaw = 0.0

target = 50.0
Kp = 0.5

def get_rotation(msg):
  global roll, pitch, yaw
  orientation_q = msg.pose.pose.orientation
  orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
  (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

# Init node
rospy.init_node('rotate_robot')

# Init subscriber
sub = rospy.Subscriber('/m2xr_diff_drive_controller/odom', Odometry, get_rotation)

# Init publisher
pub = rospy.Publisher('/m2xr_diff_drive_controller/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(100)
command = Twist()

while not rospy.is_shutdown():
  # Do the calculations
  target_rad = target * math.pi/180
  error_angle = target_rad - yaw
  command.angular.z = Kp * (error_angle)

  # Publish command
  pub.publish(command)
  print("Target:{}   Current:{} error_angle:{}".format(target_rad, yaw, error_angle) )
  
  # Sleep for some time
  r.sleep()