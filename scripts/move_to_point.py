#!/usr/bin/env python

# TODO:
# - Obicno x i y, korisceno u callback-u: Izbaciti, dodati u objekat klase Pose2D ili slicno?
# - Preimenuj: PACKAGE = "dynamic_tutorials"
# - Bug: 0 -> 180 (preskok -> divljanje)

import rospy
from tf.transformations import euler_from_quaternion
import math
from math import atan2, sqrt, pow, pi, degrees
from fsm import FsmState, FsmStates, FsmRobot
from regulator import Regulator
from ros_diff_drive.cfg import DynRecPIDConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, Point, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32

# Controller frequency in Hertz
controller_freq = 50.0

T = 1.0 / controller_freq

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

# Rotation speed limit
rot_speed_limit = 2.0

# Speed limit of moving forward
fwd_speed_limit = 0.7

# Angle error tolerance in degrees
angle_err_tolerance = 0.04

# Distance error tolerance
dist_err_tolerance = 0.005

# Values updated during the callback
x = 0.0
y = 0.0
theta = 0.0

# Goal input coordinates
goal = Point(0, 0, 0)

# TODO remove later - debug var
GOAL_THETA = 0.0
GOAL_DIST = 0.0

# Goal coordinates that are under processing
active_goal = Point(0, 0, 0)

## @brief  Calculate rotation direction
##         Positive rotation direction - clockwise
##         Negative rotation direction - counter clockwise
## @param  target_angle  Angle to be reached [degrees]
## @param  current_angle Current angle of the robot [degrees]
## @return error Error including direction [degrees]
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
  global x
  global y
  global theta

  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y

  rot_q = msg.pose.pose.orientation
  (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
  # Convert theta to degrees
  theta = degrees(theta)

# TODO Move later
xInitial = 0.0
yInitial = 0.0
def goal_position_callback(msg):
  global goal, GOAL_THETA, GOAL_DIST, xInitial, yInitial # TODO: Remove later

  if msg.x != goal.x or msg.y != goal.y or msg.theta != GOAL_THETA:
    goal.x = msg.x
    goal.y = msg.y
    GOAL_THETA = msg.theta
    GOAL_DIST = msg.x
    xInitial = x
    yInitial = y
    print("/target_position/position Goal: ({}, {})".format(goal.x, goal.y))

def idle():
  global robot_fsm, StateRotation, goal, active_goal

  # Check if desired goal has been changed during the time
  if active_goal.x != goal.x or active_goal.y != goal.y:

    # Update active goal
    active_goal.x = goal.x
    active_goal.y = goal.y

    # Switch to rotation state
    robot_fsm.switch_state(StateRotation)
    #robot_fsm.switch_state(StateForward) # TODO: Delete

    rospy.logdebug("New active goal: (%d,%d)", active_goal.x, active_goal.y)
    print("[IDL] New goal: ({},{})".format(active_goal.x, active_goal.y))

# Filtered values used for the rotation
angle_to_goal_filt = 0.0
theta_filt = 0.0

def rotate():
  global robot_fsm, active_goal, theta, goal, angle_to_goal_filt, theta_filt

  # Calculate angle to goal
  angle_to_goal = degrees(atan2(active_goal.y - y, active_goal.x - x))

  # Debug option: Uncomment for rotation parameters tuning
  #angle_to_goal = GOAL_THETA

  # Filter readings and reference
  angle_to_goal_filt = P_ANG_DST * angle_to_goal_filt + (1 - P_ANG_DST) * angle_to_goal
  theta_filt = P_ANG_THT * theta_filt + (1 - P_ANG_THT) * theta

  # Positive - CW, Negative - CCW
  angle_error = angle_error_calc(angle_to_goal_filt, theta_filt)

  # Calculate velocity inputs
  if abs(angle_error) > angle_err_tolerance:
    speed_calc = rot_pid.pid_incremental(angle_error)
    velocity.angular.z = speed_calc
  else:
    velocity.angular.z = 0
    robot_fsm.switch_state(StateForward)
    #robot_fsm.switch_state(StateIdle) #TODO: Remove

  # Publish command
  pub_cmd_vel.publish(velocity)

  # Debug publish
  #pub_dbg_angle_err.publish(angle_error)
  pub_dbg_theta.publish(theta)
  pub_dbg_theta_filtr.publish(theta_filt)
  pub_dbg_ang_to_goal.publish(angle_to_goal)
  pub_dbg_ang_to_goal_filtr.publish(angle_to_goal_filt)
  pub_dbg_rot.publish(velocity.angular.z)
  
  print("[ROT] Angle to goal: {}, theta: {}".format(angle_to_goal, theta))
  #print("[ROT] Angle error: {}".format(angle_error))

goal_distance = 0.0
goal_distance_filt = 0.0
dist_filt = 0.0
move_fwd_started = False

def forward():
  global robot_fsm, active_goal, goal_distance_filt, goal_distance, dist_filt, move_fwd_started, xInitial, yInitial

  if move_fwd_started == False:
    # Calculate current distance to the point
    xInitial = x # TODO: Uncomment
    yInitial = y # TODO: Uncomment
    goal_distance = sqrt((active_goal.x - x)**2 + (active_goal.y - y)**2)
    #goal_distance = GOAL_DIST # TODO: Comment out later, and uncomment previous line
    move_fwd_started = True

  # Update a distance from the initial point
  distance = sqrt((x - xInitial)**2 + (y - yInitial)**2)

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
    # Stop the robot and prepare for the next moving iteration
    velocity.linear.x = 0
    # distance = 0
    move_fwd_started = False
    robot_fsm.switch_state(StateIdle)

  # Publish command
  pub_cmd_vel.publish(velocity)

  pub_dbg_distance.publish(distance)
  pub_dbg_distance_filtr.publish(dist_filt)
  pub_dbg_dist_to_goal.publish(goal_distance)
  pub_dbg_dist_to_goal_filtr.publish(goal_distance_filt)
  pub_dbg_fwd.publish(velocity.linear.x)

  print("[FWD] Point({}, {}) Dist to goal: {}, Current: {}".format(x, y, goal_distance, distance))

# Initialize goal destination subscriber
sub_goal_position = rospy.Subscriber("/target_position/position", Pose2D, goal_position_callback)

# Initialize odometry subscriber
sub_odom = rospy.Subscriber("/m2xr_diff_drive_controller/odom", Odometry, position_callback)

# Initialize velocity command publisher
pub_cmd_vel = rospy.Publisher("/m2xr_diff_drive_controller/cmd_vel", Twist, queue_size=1)

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

velocity = Twist()

rospy.init_node("speed_controller")

r = rospy.Rate(controller_freq)

# Define Robot Finite State Machine states
StateIdle = FsmState(FsmStates.Idle, idle)
StateRotation = FsmState(FsmStates.Rotating, rotate)
StateForward = FsmState(FsmStates.Forward, forward)

StatesList = [StateIdle, StateRotation, StateForward]
#StatesList = [StateRotation] # TODO: Remove

# Initialize the state machine
robot_fsm = FsmRobot("M2XR", StatesList, StatesList[0])

# Initialize dynamic reconfigure server
srv__dyn_reconf = Server(DynRecPIDConfig, dyn_reconf_callback)

# Initialize PID regulators
rot_pid = Regulator(KP_ROT, TI_ROT, TD_ROT, T, rot_speed_limit, INT_LIMIT_ROT)
fwd_pid = Regulator(KP_FWD, TI_FWD, TD_FWD, T, fwd_speed_limit, INT_LIMIT_FWD)

while not rospy.is_shutdown():
  # Update PID parameters

  #  print("KP_ROT: "+ str(KP_ROT))

  rot_pid.update_params(KP_ROT, TI_ROT, TD_ROT, INT_LIMIT_ROT)
  fwd_pid.update_params(KP_FWD, TI_FWD, TD_FWD, INT_LIMIT_FWD)

  # Execute state from the FSM
  robot_fsm.execute()

  # Sleep some time...
  r.sleep()
