#!/usr/bin/env python

# TODO:
# - Error direction calculation, remove current "normalize" func
# - Filtr referencu
# - D dejstvo?
# - Inkrementalni PID

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from math import atan2, sqrt, pow, pi, degrees
from fsm import FsmState, FsmStates, FsmRobot
from regulator import Regulator
from geometry_msgs.msg import Twist, Point, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32
# from RotDbgOut.msg import dbg_angle_err

# Controller frequency in Hertz
controller_freq = 10.0

T = 1.0 / controller_freq

# Rotation speed limit
rot_speed_limit = 2.0

# Angle error tolerance in degrees
angle_err_tolerance = 0.5

# Values updated during the callback
x = 0.0
y = 0.0
theta = 0.0

# Goal input coordinates
goal = Point()
goal.x = 0
goal.y = 0
# TODO remove later - debug var
GOAL_THETA = 0.0

# Goal coordinates that are under processing
active_goal = Point()
active_goal.x = 0
active_goal.y = 0

## @brief  Calculate rotation direction
##         Positive rotation direction - clockwise
##         Negative rotation direction - counter clockwise
## @param  target_angle  Angle to be reached
## @param  current_angle Current angle of the robot
## @return error Error including direction
def angle_error_calc(target_angle, current_angle):

  # Normalize angles
  if target_angle < 0 or current_angle < 0:
    target_angle += 180.0
    current_angle += 180.0
    
  error = target_angle - current_angle

  # Change direction if absolute error is bigger than 180 degrees
  if abs(error) > 180.0:
    if error >= 0.0:
      error = -1 * (360.0 - abs(error))
    else:
      error = (360 - abs(error))

  return error

## @brief  Odometry subscriber callback function
## @param  msg The message base on Odometry message type
def position_callback(msg):
  global x
  global y
  global theta

  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y

  rot_q = msg.pose.pose.orientation
  (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
  # Convert theta to degrees
  theta = degrees(theta)

def goal_position_callback(msg):
    global goal, GOAL_THETA # TODO: Remove later

    if msg.x != goal.x or msg.y != goal.y:
      goal.x = msg.x
      goal.y = msg.y
      GOAL_THETA = msg.theta
      print("/target_position/position Goal: ({}, {})".format(goal.x, goal.y))
    
    # TODO: Remove later
    GOAL_THETA = msg.theta 

def idle():
  global robot_fsm, StateRotation, goal, active_goal

  # Check if desired goal has been changed during the time
  if active_goal.x != goal.x or active_goal.y != goal.y:
    
    # Update active goal
    active_goal.x = goal.x
    active_goal.y = goal.y

    # Switch to rotation state
    robot_fsm.switch_state(StateRotation)

    rospy.logdebug("New active goal: (%d,%d)", active_goal.x, active_goal.y)
    print("[IDL] New goal: ({},{})".format(active_goal.x, active_goal.y))

def rotate():
  global robot_fsm, active_goal, theta, speed, goal

  # Calculate angle to goal
  """
  angle_to_goal = degrees(atan2(active_goal.y, active_goal.x))
  """
  angle_to_goal = GOAL_THETA

  # Positive - CW, Negative - CCW
  angle_error = angle_error_calc(angle_to_goal, theta)
 
  # Calculate velocity inputs
  if abs(angle_error) > angle_err_tolerance:
    speed_calc = rot_pid.pid_positional(angle_error)
    velocity.angular.z = speed_calc
  else:
    velocity.angular.z = 0
    # robot_fsm.switch_state(StateIdle) TODO: Uncomment later

  # Publish command
  pub_cmd_vel.publish(velocity)
 
  # Debug publish  
  pub_dbg_angle_err.publish(angle_error)
  pub_dbg_theta.publish(theta)
  pub_dbg_ang_to_goal.publish(angle_to_goal)
  pub_dbg_rot.publish(velocity.angular.z)
  print("[ROT] Angle to goal: {}, theta: {}".format(angle_to_goal, theta))
  print("[ROT] Angle error: {}".format(angle_error))

# Initialize goal destination subscriber
sub_goal_position = rospy.Subscriber("/target_position/position", Pose2D, goal_position_callback)

# Initialize odometry subscriber
sub_odom = rospy.Subscriber("/m2xr_diff_drive_controller/odom", Odometry, position_callback)

# Initialize velocity command publisher
pub_cmd_vel = rospy.Publisher("/m2xr_diff_drive_controller/cmd_vel", Twist, queue_size=1)

#TODO: Commented out due to import error# pub_angle_error = rospy.Publisher("m2xr_diff_drive_controller/debug", Pose2D, queue_size=5)
#pub_rot_dbg_out = rospy.Publisher("debug_out", dbg_angle_err, queue_size=5)

# Debug publishers
pub_dbg_angle_err = rospy.Publisher("debug/angle_err", Float32, queue_size = 5)
pub_dbg_theta = rospy.Publisher("debug/theta", Float32, queue_size=5)
pub_dbg_ang_to_goal = rospy.Publisher("debug/angle_to_goal", Float32, queue_size=5)
pub_dbg_rot = rospy.Publisher("debug/rot_vel", Float32, queue_size=5)

velocity = Twist()

rospy.init_node("speed_controller")

r = rospy.Rate(controller_freq)

# Define Robot Finite State Machine
StateIdle = FsmState(FsmStates.Idle, idle)
StateRotation = FsmState(FsmStates.Rotating, rotate)
#StateForward = FsmState(FsmStates.Forward, forward)

StatesList = [StateRotation]

# Initialize state machine
robot_fsm = FsmRobot("M2XR", StatesList, StatesList[0])

# Get PID parameters
KP_ROT = rospy.get_param('pid_dynamic_reconfigure/KP_ROT')
KI_ROT = rospy.get_param('pid_dynamic_reconfigure/KI_ROT')

# Initialize PID regulators
rot_pid = Regulator(KP_ROT, KI_ROT, T, rot_speed_limit)

while not rospy.is_shutdown():
  # Update PID parameters
  KP_ROT = rospy.get_param('pid_dynamic_reconfigure/KP_ROT')
  KI_ROT = rospy.get_param('pid_dynamic_reconfigure/KI_ROT')
  rot_pid.update_params(KP_ROT, KI_ROT)

  # Execute state from the FSM
  robot_fsm.execute()

  # Sleep some time...
  r.sleep()
