#!/usr/bin/env python

import os
import argparse

masterState = "Recovery"

# ROSpy and dynamic reconfigration
import rospy

from shadowTool import shadowSyncClient
from std_msgs.msg import String

def shadowCallback(self,delta):
    # Called by delta message published

    state = delta.get("state")
    if state:
        print(state)
        global masterState
        masterState = state



    
# Init dynamic reconfigration
rospy.init_node("aws_iot_bridge", anonymous = True)
#rospy.Subscriber("/state", String, stateListnshadowRegisterDeltaCallbacker)


args = {}
args["host"]            = rospy.get_param("~host")
args["port"]            = rospy.get_param("~port",None)
args["useWebsocket"]    = rospy.get_param("~useWebsocket",None)
args["thingName"]       = rospy.get_param("~thingName")
args["clientId"]        = rospy.get_param("~clientId","ROS")
args["certificatePath"] = rospy.get_param("~certificatePath", os.path.dirname(__file__) + '/' + "../certs/certificate.pem.crt")
args["privateKeyPath"]  = rospy.get_param("~privateKeyPath", os.path.dirname(__file__) + '/' + "../certs/private.pem.key")
args["rootCAPath"]      = rospy.get_param("~rootCAPath",os.path.dirname(__file__) + '/' + "../certs/rootCA.pem")

shadow = shadowSyncClient(args,shadowCallback)
shadow.get()

a = 0
rate = rospy.Rate(0.5) # 10hz

while not shadow.has_initialized:
    print("shadow has not initialized")
    rate.sleep()

while not rospy.is_shutdown():
    shadow.report({'state':masterState})
    rate.sleep()

