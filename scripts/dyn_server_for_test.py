#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ros_awsiot_thingshadow.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv1 = Server(TutorialsConfig, callback,namespace="testServer1")
    srv2 = Server(TutorialsConfig, callback,namespace="testServer2")
    
    rospy.spin()