#!/usr/bin/env python

import os
import sys
import argparse
import traceback
from time import sleep
import yaml

# ROSpy and dynamic reconfigration
import rospy
from std_msgs.msg import String

from shadow import shadowSyncClient
from dyn_client import dynClientsManager


def shadowDeltaCallback(self,delta):  # Callback for Thing Shadow Receives Delta

    params = delta.get("params")
    if params and dynManager:
        for name,config in params.items():
            try:
                rospy.logwarn("Delta Received '"+str(name)+"' to "+str(config))
                dynManager.clients[name].update(config,True)
            except KeyError:
                rospy.logerr("Request Ignored. Dynamic reconfigure server:"+str(name)+" is not registerd.")
            except:
                rospy.logerr("Unknown error: "+str(sys.exc_info()))
            
                
def dynCallback(self,name,config,isInitial):   # Callback for when Dynamic Reconfigure Updated
    rospy.loginfo("Dynamic Reconfigure Callback")

    try:
        if shadow and shadow.online:
            rospy.loginfo("report to")
            rospy.loginfo(str(name)+str(config))
            if isInitial:
                shadow.report({'params':{name:config}})
            else:
                shadow.report_and_desire({'params':{name:config}})
        else:
            rospy.logwarn("Shadow is offline")

    except NameError:
        rospy.logwarn("Shadow is not configured yet")



# Init dynamic reconfigration
rospy.init_node("aws_iot_bridge", anonymous = True)
#rospy.Subscriber("/state", String, stateListnshadowRegisterDeltaCallbacker)


dyn_config  = rospy.get_param("~dyn_reconf_args",os.path.dirname(__file__) + '/' + "../config/skelton.yaml") 

rospy.loginfo(dyn_config)
with open(dyn_config, 'rt') as fp:
    yaml_text = fp.read()

dyn_configs = yaml.safe_load(yaml_text).get("dyn_reconfigures")


args = {}
args["host"]            = rospy.get_param("~host")
args["port"]            = rospy.get_param("~port",None)
args["useWebsocket"]    = rospy.get_param("~useWebsocket",None)
args["thingName"]       = rospy.get_param("~thingName")
args["clientId"]        = rospy.get_param("~clientId","ROS")
args["certificatePath"] = rospy.get_param("~certificatePath", os.path.dirname(__file__) + '/' + "../certs/certificate.pem.crt")
args["privateKeyPath"]  = rospy.get_param("~privateKeyPath", os.path.dirname(__file__) + '/' + "../certs/private.pem.key")
args["rootCAPath"]      = rospy.get_param("~rootCAPath",os.path.dirname(__file__) + '/' + "../certs/rootCA.pem")


while not rospy.is_shutdown():

    try:
        rospy.loginfo("Waiting for AWS IoT connection.")
        shadow = shadowSyncClient(args,shadowDeltaCallback)

        waiting_rate = rospy.Rate(1.0) # 10hz

        while not shadow.online and not rospy.is_shutdown():
            rospy.loginfo("Still waiting...")
            waiting_rate.sleep()

        rospy.loginfo("Registering to Dynamic Reconfigure Server.")
        dynManager = dynClientsManager(dynCallback)

        for name,args in dyn_configs.items():
            dynManager.add(name,args)

        rospy.logwarn("Getting Configrations from dynamic reconfigure servers.")

        # Publish initial state to Shadow
        initial_params = dynManager.get_configurations()
        shadow.report({'params':initial_params})

        while not rospy.is_shutdown() and len(shadow.queue) != 0:
            waiting_rate.sleep()

        rospy.logwarn("Updated!!")
        shadow.enable_delta_callback = True
            
        r = rospy.Rate(1.0/10.0)  # Delta get

        while not rospy.is_shutdown():
            shadow.get()
            if shadow.online == False:
                raise "Shadow is not online!"
            r.sleep()

    except:

        rospy.logerr("Error occured.")
        t, v, tb = sys.exc_info()
        rospy.logerr(traceback.format_exception(t,v,tb))
        rospy.logwarn("Retrying soon..")
        sleep(5)
