#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point
import math
from math import atan2, sqrt, pow

# Values updated during the callback
x = 0.0
y = 0.0
theta = 0.0

# Odometry subscriber callback function
def position_callback(msg):
  global x
  global y
  global theta

  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y

  rot_q = msg.pose.pose.orientation
  (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

# Initialize odometry subscriber
sub = rospy.Subscriber("/m2xr_diff_drive_controller/odom", Odometry, position_callback)

# Initialize velocity publisher
pub = rospy.Publisher("/m2xr_diff_drive_controller/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 0.0
goal.y = 0.0

while not rospy.is_shutdown():
  inc_x = goal.x - x
  inc_y = goal.y - y

  # Calculate goal point details
  angle_to_goal = atan2(inc_y, inc_x)
  dist_to_goal = sqrt(pow(inc_x, 2) + pow(inc_y,2))


  # Decide whether to rotate or to move forward
  if dist_to_goal > 0.1:
    if abs(angle_to_goal - theta) > 0.5:
      speed.linear.x = 0.0
      speed.angular.z = 0.3
    else:
      speed.linear.x = 0.5
      speed.angular.z = 0.0
  else:
    speed.linear.x = 0.0
    speed.angular.z = 0.0

  # Publish command
  pub.publish(speed)
  print("Dist: {}  AngleErr[Deg]: {}".format(dist_to_goal, (angle_to_goal - theta)))
  print("X: {}  Y: {} Theta: {}".format(x, y, theta))

  # Sleep some time...
  r.sleep()
