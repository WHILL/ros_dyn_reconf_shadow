#!/usr/bin/env python

import os
import sys
import argparse
import traceback
from time import sleep
import yaml
from datetime import datetime

# ROSpy and dynamic reconfigration
import rospy
from std_msgs.msg import String
from std_srvs.srv import *


from shadow import shadowSyncClient
from dyn_client import dynClientsManager


def callChangeStateService(state):
    rospy.wait_for_service('/maintain/setState/'+str(state))
    try:
        changeState = rospy.ServiceProxy('/maintain/setState/'+str(state), Trigger)
        resp = changeState()
        return resp
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)


state = None


def stateListener(data):

    try:
        stateListener.timestamp_before
    except:
        stateListener.timestamp_before = datetime.now()

    # @TODO Need to consider a case of reconnect
    try:
        global state
        global shadow
        if shadow.online and (state != data.data or (datetime.now() - stateListener.timestamp_before).total_seconds() > 5):
            rospy.loginfo('Report state:'+str(data.data))
            shadow.report({'state':data.data})
            state = data.data
            stateListener.timestamp_before = datetime.now()
    except:
        pass
    

def stateReportAndDesireListner(data):

    try:
        global state
        global shadow
        if shadow.online and (state != data.data or (datetime.now() - stateListener.timestamp_before).total_seconds() > 5):
            rospy.loginfo(' state:'+str(data.data))
            shadow.report_and_desire({'state':data.data})
            state = data.data
    except:
        pass



def shadowDeltaCallback(self,delta):  # Callback for Thing Shadow Receives Delta

    new_state = delta.get("state")
    if new_state:
        global state
        rospy.loginfo("State delta:"+str(new_state)+" Current:"+str(state))
        state_delta_pub.publish(str(new_state))

    params = delta.get("params")
    if params and dynManager:
        for name,config in params.items():
            try:
                rospy.logwarn("Delta Received "+str(name)+" to "+str(config))
                dynManager.clients[str(name)].update(config,True)
            except KeyError:
                rospy.logerr("Request Ignored. Dynamic reconfigure server:"+str(name)+" is not registerd.")
            except:
                rospy.logerr("Unknown error at "+str(name)+": "+str(sys.exc_info()))
            
                
def dynCallback(self,name,config,isInitial):   # Callback for when Dynamic Reconfigure Updated

    try:
        if shadow and shadow.online:
            rospy.loginfo("Reporting: "+str(name)+" to "+str(config))
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

state_delta_pub = rospy.Publisher("state/delta",String,queue_size=10)
rospy.Subscriber("state/report", String, stateListener)
rospy.Subscriber("state/report_and_desire",String,stateReportAndDesireListner)

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

        shadow.desire({'state':None})

        rospy.loginfo("Registering to Dynamic Reconfigure Server.")
        dynManager = dynClientsManager(dynCallback)

        for name,args in dyn_configs.items():
            dynManager.add(name,args)

        r = rospy.Rate(1.0/5.0)
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
