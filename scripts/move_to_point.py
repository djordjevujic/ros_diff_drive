"""
Implements differential drive robot control
"""

#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, degrees
from fsm import FsmState, FsmStates, FsmRobot
from regulator import Regulator
from ros_diff_drive.cfg import DynRecPIDConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, Point, Pose2D
from nav_msgs.msg import Odometry
from config import *
from debug import *

## Controller's period in seconds
T = 1.0 / controller_freq

## @defgroup xyInit Values updated during the callback of the odometry
## @{

## Initial x coordinate from the start of moving forward
xInitial = 0.0

## Initial y coordinate from the start of moving forward
yInitial = 0.0
## @}

## @defgroup callbackPos Values updated during the callback of the odometry
## These values are updated during \ref position_callback
## @{

## Contains X and Y coordinates of the current position
cur_pos = Point()

## X coordinate of the current position - initialization
cur_pos.x = 0.0

## Y coordinate of the current position - initialization
cur_pos.y = 0.0

## Represents current angle of the robot
theta = 0.0
## @}

## Goal input coordinates
## Updated in \ref goal_position_callback
goal = Point(0, 0, 0)

## @defgroup debugVar Variables used for debugging purposes
## If debugging only rotation/forward, they receive the value
## from the dynamic reconfigure.
## @{

## Goal angle
GOAL_THETA = 0.0

## Goal distance
GOAL_DIST = 0.0
## @}

## @brief Goal coordinates that are under processing
active_goal = Point(0, 0, 0)

# Rotation values
## Filtered value of the goal angle
angle_to_goal_filt = 0.0

## Filtered value of the current angle
theta_filt = 0.0

## Constant for indicating positive angle
POSITIVE = True

## Constant for indicating negative angle
NEGATIVE = False

## Variable indicating signess of the current iteration angle: \ref POSITIVE or \ref NEGATIVE
theta_sign = POSITIVE

## Variable indicating signess of the angle from the previous iteration: \ref POSITIVE or \ref NEGATIVE
theta_sign_prev = POSITIVE

# Moving forward values
## Calculated goal distance
goal_distance = 0.0

## Filtered value of the goal distance
goal_distance_filt = 0.0

## Filtered value of the desired distance
dist_filt = 0.0

## Counts up each time when robot calculated distance to goal is in the tolerance range
stop_lin_movement_cnt = 0

## Indicates if moving forward started or not
move_fwd_started = False

## Used for correcting angle error which accumulates while moving forward
angle_to_goal_fwd = 0.0


## @brief  Calculate rotation direction \n
##         Positive rotation direction - clockwise \n
##         Negative rotation direction - counter clockwise
## @param  target_angle  Angle to be reached [degrees]
## @param  current_angle Current angle of the robot [degrees]
## @return Error including direction [degrees]
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

## @brief  Dynamic reconfigure callback function
##
## @param  config Contains all dynamic parameters
## @param  level  Not used
## @return config
def dyn_reconf_callback(config, level):
  global KP_ROT, TI_ROT, TD_ROT, INT_LIMIT_ROT, P_ANG_DST, P_ANG_THT, \
         KP_FWD, TI_FWD, TD_FWD, P_FWD_DST, P_FWD_CUR, INT_LIMIT_FWD

  # Update rotation parameters
  KP_ROT = config['KP_ROT']
  TI_ROT = config['TI_ROT']
  TD_ROT = config['TD_ROT']
  INT_LIMIT_ROT = config['UI_LIMIT_ROT']
  P_ANG_DST = config['P_ANG_DST']
  P_ANG_THT = config['P_ANG_THT']

  # Update moving forward parameters
  KP_FWD = config['KP_FWD']
  TI_FWD = config['TI_FWD']
  TD_FWD = config['TD_FWD']
  P_FWD_DST = config['P_FWD_DST']
  P_FWD_CUR = config['P_FWD_CUR']
  INT_LIMIT_FWD = config['UI_LIMIT_FWD']

  return config

## @brief  Odometry subscriber callback function
## @param  msg The message base on Odometry message type
def position_callback(msg):
  global cur_pos
  global theta

  cur_pos.x = msg.pose.pose.position.x
  cur_pos.y = msg.pose.pose.position.y

  rot_q = msg.pose.pose.orientation
  (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
  # Convert theta to degrees
  theta = degrees(theta)

## @brief Callback function for processing goal values
## @param msg Message to be processed - contains desired goal position
def goal_position_callback(msg):
  global goal, GOAL_THETA, GOAL_DIST, xInitial, yInitial

  if msg.x != goal.x or msg.y != goal.y or msg.theta != GOAL_THETA:
    goal.x = msg.x
    goal.y = msg.y
    GOAL_THETA = msg.theta
    GOAL_DIST = msg.x
    xInitial = cur_pos.x
    yInitial = cur_pos.y
    print("/target_position/position Goal: ({}, {})".format(goal.x, goal.y))

## @brief Idle function of the robot state machine
##
## In this state robot is waiting for the new command to arrive.
def idle():
  global robot_fsm, StateRotation, goal, active_goal, cur_pos

  # Check if desired goal has been changed during the time
  if active_goal.x != goal.x or active_goal.y != goal.y:

    # Update active goal
    active_goal.x = goal.x
    active_goal.y = goal.y

    # Switch to rotation state
    robot_fsm.switch_state(StateRotation)
    #robot_fsm.switch_state(StateForward) # Can be used for debugging purposes

    rospy.logdebug("New active goal: (%d,%d)", active_goal.x, active_goal.y)
    print("[IDL] New goal: ({},{})".format(active_goal.x, active_goal.y))

  #print("[IDL] Point({}, {})".format(cur_pos.x, cur_pos.y))

## @brief Rotation function of the robot state machine
##
## This state controls robot rotation. It filters current angle and desired angle values, calculates an error in degrees
## and generates desired rotation speed calculated in PID routine.
## Desired rotation speed is then published to the velocity publisher \ref pub_cmd_vel.
def rotate():
  global robot_fsm, active_goal, theta, goal, angle_to_goal_filt, theta_filt, theta_sign, theta_sign_prev

  # Used for the velocity command storage
  velocity = Twist()

  # Check signess of the current angle
  if theta > 0.0:
    theta_sign = POSITIVE
  else:
    theta_sign = NEGATIVE

  # Workaround to overcome an issue caused by changed sign of the theta
  # Issue is caused by theta going from positive (~180) to negative (~ -180),
  # filtering slows down this conversion and induces an error spike which
  # is reacted with the huge jump in the control value
  if theta_sign != theta_sign_prev:
    theta_filt = -theta_filt

  # Remember current sign
  theta_sign_prev = theta_sign

  # Calculate angle to goal
  angle_to_goal = degrees(atan2(active_goal.y - cur_pos.y, active_goal.x - cur_pos.x))

  # Debug option: Uncomment for rotation parameters tuning
  #angle_to_goal = GOAL_THETA

  # Filter readings and reference
  angle_to_goal_filt = P_ANG_DST * angle_to_goal_filt + (1 - P_ANG_DST) * angle_to_goal
  theta_filt = P_ANG_THT * theta_filt + (1 - P_ANG_THT) * theta

  # Positive - CW, Negative - CCW
  angle_error = angle_error_calc(angle_to_goal_filt, theta_filt)

  # Calculate velocity inputs
  if abs(angle_error) > angle_err_tolerance_rot:
    speed_calc = rot_pid.pid_incremental(angle_error)
    velocity.angular.z = speed_calc
  else:
    velocity.angular.z = 0
    robot_fsm.switch_state(StateForward)
    #robot_fsm.switch_state(StateIdle) # Can be used for debugging purposes

  # Publish command
  pub_cmd_vel.publish(velocity)

  # Debug publish
  pub_dbg_angle_err.publish(angle_error)
  pub_dbg_theta.publish(theta)
  pub_dbg_theta_filtr.publish(theta_filt)
  pub_dbg_ang_to_goal.publish(angle_to_goal)
  pub_dbg_ang_to_goal_filtr.publish(angle_to_goal_filt)
  pub_dbg_rot.publish(velocity.angular.z)

  print("[ROT] Angle to goal: {}, theta: {}".format(angle_to_goal, theta))
  #print("[ROT] Angle error: {}".format(angle_error))

## @brief State machine functionality for moving forward
##
## This state controls robot when moving forward. It filters current distance and desired distance values, calculates an error,
## and generates desired linear speed calculated in PID routine.
## Desired rotation speed is then published to the velocity publisher \ref pub_cmd_vel.
def forward():
  global robot_fsm, active_goal, goal_distance_filt, goal_distance, dist_filt, move_fwd_started, xInitial, yInitial, angle_to_goal_fwd, \
          stop_lin_movement_cnt

  # Used for the velocity command storage
  velocity = Twist()

  if move_fwd_started == False:
    # Calculate current distance to the point
    xInitial = cur_pos.x
    yInitial = cur_pos.y
    goal_distance = sqrt((active_goal.x - cur_pos.x)**2 + (active_goal.y - cur_pos.y)**2)
    #goal_distance = GOAL_DIST # This line is only for debugging purposes
    move_fwd_started = True

  # Update a distance from the initial point
  distance = sqrt((cur_pos.x - xInitial)**2 + (cur_pos.y - yInitial)**2)

  # Filter the reference
  goal_distance_filt = P_FWD_DST * goal_distance_filt + (1 - P_FWD_DST) * goal_distance

  # Filter current reading
  dist_filt = P_FWD_CUR * dist_filt + (1 - P_FWD_CUR) * distance

  # Calculate the distance error
  dist_error = goal_distance_filt - dist_filt

  if (abs(dist_error) > dist_err_tolerance):
    speed_calc = fwd_pid.pid_incremental(dist_error)
    velocity.linear.x = speed_calc
  else:
    # Initiate stop and prepare for the next moving iteration
    velocity.linear.x = 0
    stop_lin_movement_cnt = stop_lin_movement_cnt + 1

  # Switch to the another state only if robot was enough cycles in the tolerance region
  if stop_lin_movement_cnt > NO_OF_CYCLES_LIN_MOVEMENT_STOP:
    stop_lin_movement_cnt = 0
    move_fwd_started = False
    robot_fsm.switch_state(StateIdle)

  # Correct angle error which accumulates while moving forward
  angle_to_goal_fwd = degrees(atan2(active_goal.y - cur_pos.y, active_goal.x - cur_pos.x))
  angle_error = angle_error_calc(angle_to_goal_fwd, theta)

  # Correct angle only if distance is longer than 1 meter
  if dist_error > 1:
  
    if abs(angle_error) > angle_err_tolerance_fwd:
      speed_calc = fwd_pid_rot.pid_incremental(angle_error)
      velocity.angular.z = speed_calc
    else:
      velocity.angular.z = 0

  # Publish command
  pub_cmd_vel.publish(velocity)

  pub_dbg_distance.publish(distance)
  pub_dbg_distance_filtr.publish(dist_filt)
  pub_dbg_dist_to_goal.publish(goal_distance)
  pub_dbg_dist_to_goal_filtr.publish(goal_distance_filt)
  pub_dbg_fwd.publish(velocity.linear.x)
  pub_dbg_fwd_rot.publish(angle_error)
  pub_dbg_fwd_rot_vel.publish(velocity.angular.z)

  print("[FWD] Point({}, {}) Dist to goal: {}, Current: {}".format(cur_pos.x, cur_pos.y, goal_distance, distance))

## Goal destination subscriber
sub_goal_position = rospy.Subscriber("/target_position/position", Pose2D, goal_position_callback)

## Odometry (current position) subscriber
sub_odom = rospy.Subscriber("/m2xr_diff_drive_controller/odom", Odometry, position_callback)

##  @brief cmd_vel publisher
##  Used to publish desired velocity to the next layer of the control.
pub_cmd_vel = rospy.Publisher("/m2xr_diff_drive_controller/cmd_vel", Twist, queue_size=1)

rospy.init_node("speed_controller")

## Initialization of the speed_controller node
r = rospy.Rate(controller_freq)

## FSM idle state definition
StateIdle = FsmState(FsmStates.Idle, idle)

## FSM rotation state definition
StateRotation = FsmState(FsmStates.Rotating, rotate)

## FSM moving forward state definition
StateForward = FsmState(FsmStates.Forward, forward)

## List of permitted states
StatesList = [StateIdle, StateRotation, StateForward]
#StatesList = [StateRotation] # Used only for debugging purposes

## FSM Initialization
robot_fsm = FsmRobot("M2XR", StatesList, StatesList[0])

## Dynamic reconfigure server initialization
srv__dyn_reconf = Server(DynRecPIDConfig, dyn_reconf_callback)

## Normal rotation PID regulator initialization
rot_pid = Regulator(KP_ROT, TI_ROT, TD_ROT, T, rot_speed_limit, INT_LIMIT_ROT)

## Moving forward PID regulator initialization
fwd_pid = Regulator(KP_FWD, TI_FWD, TD_FWD, T, fwd_speed_limit, INT_LIMIT_FWD)

## Rotation while moving forward PID regulator initialization
fwd_pid_rot = Regulator(KP_ROT, TI_ROT, TD_ROT, T, rot_speed_limit, INT_LIMIT_ROT)

while not rospy.is_shutdown():

  # Update PID parameters
  rot_pid.update_params(KP_ROT, TI_ROT, TD_ROT, INT_LIMIT_ROT)
  fwd_pid.update_params(KP_FWD, TI_FWD, TD_FWD, INT_LIMIT_FWD)
  fwd_pid_rot.update_params(KP_ROT, TI_ROT, TD_ROT, INT_LIMIT_ROT)

  # Execute state from the FSM
  robot_fsm.execute()

  # Sleep for some time...
  r.sleep()
