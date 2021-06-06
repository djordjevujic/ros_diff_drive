#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ros_diff_drive.cfg import DynRecPIDConfig

KP_ROT = float()
KP_ROT = float()

def callback(config, level):
    global KP_ROT, KP_ROT

    rospy.loginfo("""Reconfigure Request: {KP_ROT}, {KI_ROT},\ 
          """.format(**config))

    return config

if __name__ == "__main__":
    rospy.init_node("dyn_pid_params", anonymous = False)

    srv = Server(DynRecPIDConfig, callback)
    rospy.spin()
