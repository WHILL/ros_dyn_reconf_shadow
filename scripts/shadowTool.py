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

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTShadowClient
import json
import logging
import rospy

class shadowSyncClient:

    def __init__(self,args,callback):

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
        #logger.addHandler(streamHandler)

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

        self.callback = callback

        # Create a deviceShadow with persistent subscription
        self.handler = myAWSIoTMQTTShadowClient.createShadowHandlerWithName(thingName, True)
        self.handler.shadowRegisterDeltaCallback(self.__deltaCallback)

        self.__reported = None
        self.has_initialized = False

    def get(self):
        self.handler.shadowGet(self.__getCallback, 5)

    def report(self,reported):
        if reported != self.__reported:
            state = {'reported':reported}
            self.__update(state)
    
    def desire(self,desired):
        state = {'desired':desired}
        self.__update(state)

    def __update(self,state):
        data = json.dumps({'state':state})
        self.handler.shadowUpdate(data, self.__updateCallback, 5)

    def __updateCallback(self, data, responseStatus, token):
        # payload is a JSON string ready to be parsed using json.loads(...)
        # in both Py2.x and Py3.x

        payload = json.loads(data)

        if responseStatus == "timeout":
            rospy.logwarn("Update request " + token + " time out!")

        if responseStatus == "rejected":
            rospy.logwarn("Update request " + token + " rejected!") 

        if responseStatus == "accepted":
            rospy.loginfo("Update request accepted")
            payload = json.loads(data)
            state = payload["state"]
            self.__reported = state.get("reported")


    # Custom Shadow callback
    def __deltaCallback(self, data, responseStatus, token):
        # payload is a JSON string ready to be parsed using json.loads(...)
        payload = json.loads(data)
        if payload.has_key("state"):
            delta    = payload.get("state")
            self.callback(self,delta)
    
    def __getCallback(self,data,responseStatus,token):
        self.has_initialized = True
        payload = json.loads(data)
        if payload.has_key("state"):
            state = payload["state"]
            self.__reported = state.get("reported")
            self.callback(self,state.get("delta"))
            
