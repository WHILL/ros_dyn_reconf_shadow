#!/usr/bin/env python

'''
/*
 * Copyright 2010-2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
 '''

import os

# AWS Iot
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTShadowClient
import logging
import time
import json
import argparse


# ROSpy and dynamic reconfigration
import rospy

from dynamic_reconfigure.server import Server


# Custom Shadow callback
def customShadowCallback_Update(payload, responseStatus, token):
    # payload is a JSON string ready to be parsed using json.loads(...)
    # in both Py2.x and Py3.x
    if responseStatus == "timeout":
        print("Update request " + token + " time out!")
    if responseStatus == "accepted":
        payloadDict = json.loads(payload)
        print("~~~~~~~~~~~~~~~~~~~~~~~")
        print("Update request with token: " + token + " accepted!")
        print("property: " + str(payloadDict["state"]))
        print("~~~~~~~~~~~~~~~~~~~~~~~\n\n")
    if responseStatus == "rejected":
        print("Update request " + token + " rejected!")

class shadowCallbackContainer:
    def __init__(self, deviceShadowInstance):
        self.deviceShadowInstance = deviceShadowInstance

    def customShadowGetCallback(self, payload, responseStatus, token):
        print("Received Shadow Get response:")
        payloadDict = json.loads(payload)
    
        if(not payloadDict["state"].has_key("delta")):
            print("delta is empty")
        else:
            delta = payloadDict["state"]["delta"]

            newPayload = '{"state":{"reported":' + json.dumps(delta) + '}}'

            if delta.has_key('params'):
                if srv:
                    srv.update_configuration(delta['params'])
                    #self.deviceShadowInstance.shadowUpdate(newPayload, None, 5)
                print("Sent.")
            
        
    # Custom Shadow callback
    def customShadowCallback_Delta(self, payload, responseStatus, token):
        # payload is a JSON string ready to be parsed using json.loads(...)
        # in both Py2.x and Py3.x
        print("Received a delta message:")
        payloadDict = json.loads(payload)
        deltaMessage = json.dumps(payloadDict["state"])

        if payloadDict["state"].has_key('params'):
            params = payloadDict["state"]["params"]
            
            print(payloadDict["state"]["params"])

            print(payloadDict)
            srv.update_configuration(payloadDict["state"]["params"])
            #self.deviceShadowInstance.shadowUpdate(newPayload, None, 5)
            print("Sent.")


def dynReconfigureCallback(config, level):
    
    params = {}
    for key in config.groups.parameters:
        params[key] = config[key]

    state = {}
    state["reported"] = {"params":params}

    if not (level == -1):  # Not Self Initialization
        desired = {"params":params}
        state["desired"] = desired

    try:
        deviceShadowHandler.shadowUpdate(json.dumps({"state":state}), customShadowCallback_Update, 5)
    except:
        rospy.loginfo("deviceShadowHandler is not declared yet")

    return config




def connectDeviceShadow(args):

    # Read in command-line parameters

    host            = args.get("host")
    port            = args.get("port")
    useWebsocket    = args.get("useWebsocket")
    thingName       = args.get("thingName")
    clientId        = args.get("cliendID")
    certificatePath = args.get("certificatePath")
    privateKeyPath  = args.get("privateKeyPath")
    rootCAPath      = args.get("rootCAPath")

    if useWebsocket and certificatePath and privateKeyPath:
        rospy.logfatal("X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
        exit(2)

    if not useWebsocket and (not certificatePath or not privateKeyPath):
        rospy.logfatal("Missing credentials for authentication.")
        exit(2)

    # Port defaults
    if useWebsocket and not port:  # When no port override for WebSocket, default to 443
        port = 443
    if not useWebsocket and not port:  # When no port override for non-WebSocket, default to 8883
        port = 8883

    # Configure logging
    logger = logging.getLogger("AWSIoTPythonSDK.core")
    logger.setLevel(logging.DEBUG)
    streamHandler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    streamHandler.setFormatter(formatter)
    logger.addHandler(streamHandler)

    # Init AWSIoTMQTTShadowClient
    myAWSIoTMQTTShadowClient = None
    if useWebsocket:
        myAWSIoTMQTTShadowClient = AWSIoTMQTTShadowClient(clientId, useWebsocket=True)
        myAWSIoTMQTTShadowClient.configureEndpoint(host, port)
        myAWSIoTMQTTShadowClient.configureCredentials(rootCAPath)
    else:
        myAWSIoTMQTTShadowClient = AWSIoTMQTTShadowClient(clientId)
        myAWSIoTMQTTShadowClient.configureEndpoint(host, port)
        myAWSIoTMQTTShadowClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)

    # AWSIoTMQTTShadowClient configuration
    myAWSIoTMQTTShadowClient.configureAutoReconnectBackoffTime(1, 32, 20)
    myAWSIoTMQTTShadowClient.configureConnectDisconnectTimeout(10)  # 10 sec
    myAWSIoTMQTTShadowClient.configureMQTTOperationTimeout(5)  # 5 sec

    # Connect to AWS IoT
    myAWSIoTMQTTShadowClient.connect()

    # Create a deviceShadow with persistent subscription
    global deviceShadowHandler
    deviceShadowHandler = myAWSIoTMQTTShadowClient.createShadowHandlerWithName(thingName, True)
    global shadowCallbackContainer_Bot
    shadowCallbackContainer_Bot = shadowCallbackContainer(deviceShadowHandler)



# Init dynamic reconfigration
rospy.init_node("aws_iot_bridge", anonymous = True)

args = {}

args["host"]            = rospy.get_param("~host")
args["port"]            = rospy.get_param("~port",None)
args["useWebsocket"]    = rospy.get_param("~useWebsocket",None)
args["thingName"]       = rospy.get_param("~thingName")
args["clientId"]        = rospy.get_param("~clientId","ROS")
args["certificatePath"] = rospy.get_param("~certificatePath", os.path.dirname(__file__) + '/' + "../certs/certificate.pem.crt")
args["privateKeyPath"]  = rospy.get_param("~privateKeyPath", os.path.dirname(__file__) + '/' + "../certs/private.pem.key")
args["rootCAPath"]      = rospy.get_param("~rootCAPath",os.path.dirname(__file__) + '/' + "../certs/rootCA.pem")

connectDeviceShadow(args)

from ros_aws_iot_bridge.cfg import sampleConfig as DynParams
srv = Server(DynParams, dynReconfigureCallback)

deviceShadowHandler.shadowGet(shadowCallbackContainer_Bot.customShadowGetCallback, 5)

# Listen on deltas
deviceShadowHandler.shadowRegisterDeltaCallback(shadowCallbackContainer_Bot.customShadowCallback_Delta)


rospy.spin()