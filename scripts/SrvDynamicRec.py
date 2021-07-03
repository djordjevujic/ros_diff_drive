#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ros_diff_drive.cfg import DynRecPIDConfig

def callback(config, level):
  rospy.loginfo("""Reconfigure Request: {KP_ROT}, {TI_ROT}, {TD_ROT} {UI_LIMIT}\ 
        """.format(**config))

  return config

## @brief  Reads rotation PID parameters
## @return KP, KI, KD respectively
#def get_rot_dyn_conf():
  #KP_ROT = rospy.get_param('pid_dynamic_reconfigure/KP_ROT')
  #KI_ROT = rospy.get_param('pid_dynamic_reconfigure/TI_ROT')
  #KD_ROT = rospy.get_param('pid_dynamic_reconfigure/TD_ROT')
  #INT_LIMIT = rospy.get_param('pid_dynamic_reconfigure/UI_LIMIT')

  #return KP_ROT, KI_ROT, KD_ROT, INT_LIMIT

if __name__ == "__main__":
  rospy.init_node("dyn_pid_params", anonymous = False)

  srv = Server(DynRecPIDConfig, callback)
  rospy.spin()
