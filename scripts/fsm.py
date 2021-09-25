"""
Finite State Machine library
"""

#!/usr/bin/env python
import enum
from rospy import logerr, loginfo
import time

## Enumeration containing state machine states definitions
class FsmStates(enum.Enum):
  """ FSM states enumeration
  """
  ## Default state and should not be used by the user. Used only as initial value of previous_state. 
  Default = 0 
  
  ## Waiting for a new command
  Idle = 1    
  
  ## State for the robot rotation
  Rotating = 2
  
  ## State for moving forward
  Forward = 3

## FSM state class. Contains state enumerator defined \ref FsmStates, as well as
## the method which shall be executed with this state.
class FsmState:
  ## Constructor which contains desired state enumerator and method
  def __init__(self, state, method):
    ## state which stores enumeration value from \ref FsmStates
    self.state = state

    ## method which will be executed once this state is ongoing
    self.method = method


## Finite State Machine class
## Provides data structure for FSM as well as main
## methods for normal functioning and events logging
class FsmRobot:
  ## Default constructor for FSM
  ## name: Name of the FSM
  ## @param name Desired name of the FSM
  ## @param states_list List of permitted states
  ## @param state Desired to be current (initial) state 
  def __init__(self, name, states_list, state):
    ## Name of the FSM
    self.name = name

    ## List of states
    self.states_list = states_list

    ## State which is currently under the execution
    self.current_state = state

    ## Previous state
    self.previous_state = FsmState(FsmStates.Default, self.default)

  ## Method used for switching between states of the FSM
  ## @param new_state State which FSM will switch to
  def switch_state(self, new_state):
    # Validate new state
    self.validate_state(new_state)

    self.previous_state = self.current_state
    self.current_state = new_state
    loginfo("Robot %s: Switching state to %s", self.name, self.current_state.state)

  ## Method used for state validation. It basically checks if state is in the list \ref states_list of predefined states.
  ## @param state State to be validated
  def validate_state(self, state):
    state_valid = False

    for st in self.states_list:
      if state.state == st.state and state.method == st.method:
        state_valid = True

    if state_valid == False:
      logerr("%s: Invalid state, st: %s met: %s! Exiting...", self.name, state.state, state.method)
      exit()

  ## Default method of the FSM. If FSM is initialized properly, this state must not be executed!
  def default(self):
    logerr("Robot %s: Default state should never be reached! Exiting...", self.name)
    exit()

  ## Method used to execute \ref current_state of the FSM
  def execute(self):
    self.current_state.method()
