"""@package fsm
Finite State Machine library
"""

#!/usr/bin/env python
import enum
from rospy import logerr, loginfo
import time

class FsmStates(enum.Enum):
  """ FSM states enumeration
  """
  Default = 0   # Used only as initial value for previous_state
  Idle = 1      # Waiting for a new command
  Rotating = 2  # Rotating
  Forward = 3   # Moving forward

class FsmState:
  def __init__(self, state, method):
    self.state = state
    self.method = method

class FsmRobot:
  """ Finite State Machine class
  Provides data structure for FSM as well as main
  methods for normal functioning and events logging
  """
  def __init__(self, name, states_list, state):
    """ Default constructor for FSM
    name: Name of the FSM
    states_list: List of all available states
    state: initial state
    """
    self.name = name
    self.states_list = states_list
    self.current_state = state
    self.previous_state = FsmState(FsmStates.Default, self.default)

  def switch_state(self, new_state):
    """ Method used for switching between states of the FSM
    """
    # Validate new state
    self.validate_state(new_state)

    self.previous_state = self.current_state
    self.current_state = new_state
    loginfo("Robot %s: Switching state to %s", self.name, self.current_state.state)

  def validate_state(self, state):
    state_valid = False

    for st in self.states_list:
      if state.state == st.state and state.method == st.method:
        state_valid = True

    if state_valid == False:
      logerr("%s: Invalid state, st: %s met: %s! Exiting...", self.name, state.state, state.method)
      exit()

  def default(self):
    logerr("Robot %s: Default state should never be reached! Exiting...", self.name)
    exit()

  def execute(self):
    self.current_state.method()

def methodA():
  print("This is method A")

idleCounter = 0
def idle():
  print("This is IDLE method")
  global idleCounter 
  idleCounter = idleCounter + 1
  if idleCounter > 2:
    robot_fsm.switch_state(FsmState(FsmStates.Rotating, rot))

def fwd():
  print("This is FWD method")
  robot_fsm.switch_state(FsmState(FsmStates.Idle, inv))

rotCnt = 0
def rot():
  global rotCnt
  rotCnt = rotCnt + 1
  print("This is ROT method")

  if rotCnt > 2:
    robot_fsm.switch_state(FsmState(FsmStates.Forward, fwd))

def inv():
  print("This is invalid state")

"""
machine1 = FsmState(FsmStates.Idle, methodA)
machine2 = FsmState(FsmStates.Rotating, FsmRobot.default)

FsmList = [FsmState(FsmStates.Idle, idle), FsmState(FsmStates.Rotating, rot), FsmState(FsmStates.Forward, fwd)]

robot_fsm = FsmRobot("MoveToPoint", FsmList, FsmList[0])

while 1:
  robot_fsm.execute()
  time.sleep(0.5)
  """