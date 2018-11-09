#!/usr/bin/env python

import rospy

from pprint import pprint
import threading

import sys
from time import sleep

import dynamic_reconfigure.client


class dynClient:

    def __init__(self,name,args,callback,connected_callback,combine=True):
        # @TODO Needs type check for arguments!
        self.name       = name
        self.args       = args
        self.__callback = callback
        self.__connected_callback = connected_callback
        self.__combine  = combine
        self.__config   = {}
        
        self.__status             = "Connecting"
        self.__callback_even_same = False

        self.__has_initialized = False

        # Connect to dynamic reconfigure server in another thread since the function is asyncronized.
        self.__connection_thread = threading.Thread(target=self.connection_thread_process)
        self.__connection_thread.start()


    def connection_thread_process(self):
        rate = rospy.Rate(1.0/3.0)
        while True and not rospy.is_shutdown():
            try:
                self.__client   = dynamic_reconfigure.client.Client(self.name, timeout=30)

                # Following codes run when connection succeess.
                self.__ignore_callback = False  # ignore the callback for timming of registering
                self.__client.set_config_callback(self.__going_callback)
                self.__status = "Connected"
                rospy.loginfo("Connected to dyn server:"+str(self.name)+".")
                self.__connected_callback(self.name,self)
                return  # Finish this thread when connected.

            except:
                # When connection timeout, retry.
                #rospy.logwarn()
                #print("Trying to reconnect dyn server:"+str(self.name))
                rate.sleep()

    def is_connected(self):
        if self.__status == "Connected":
            return True
        else:
            return False

    def update(self,config,must_fire_callback=False):

        self.__callback_even_same = must_fire_callback

        try:
            self.__client.update_configuration(config)
        except:
            self.__status = "Disconnected"
            rospy.logerr("Error at "+str(self.name)+" "+str(sys.exc_info()))


    def get_configurations(self,timeout=None):

        try:
            return self.__client.get_configuration(timeout)
        except:
            self.__status = "Disconnected"
            rospy.logerr("Error at "+str(self.name)+" "+str(sys.exc_info()))

        return None

    def __filter_configs(self,config):
        lists = {}
        for key in self.args:
            lists[key] = config.get(key)

        return lists

    def __going_callback(self,config):

        if config is None:
            return   # If the dynamic reconfigure contains is empty.


        if not self.is_connected():   # Recovering from disconnected
            self.__callback_even_same = True
            self.__has_initialized    = False
            self.__config = {}   # Reset cache
            self.__status = "Connected"

        if self.__ignore_callback is True:
            rospy.logdebug("Callback registering was Ignored.")
            self.__ignore_callback = False
            return


        # Determine something changed compare to cache
        lists = self.__filter_configs(config)
        if (lists == self.__config) and not self.__callback_even_same:
            print("Dyn Callback not called due to same")
            self.__callback_even_same = False
            return    # If any of specified args is not changed

        # Find actual changed
        new_config = {}
        for k,v in lists.items():
            if not self.__config.has_key(k) or v != self.__config[k]:
                new_config[k] = v

        # Return if nothing changed
        if len(new_config) == 0:
            return

        self.__config = lists

        if self.__combine:
            
            self.__callback({self.name:new_config},not self.__has_initialized)
        else:
            self.__callback(self.name,new_config,not self.__has_initialized)

        self.__has_initialized = True
        self.__callback_even_same = False


class dynClientsManager:

    def __init__(self,callback):
        self.clients = {}
        self.__callback = callback

    def add(self,name,args):
        dynClient(name,args,self.__clients_callback,self.__client_connected_reception,False)

    def __client_connected_reception(self,name,client):
        self.clients[name] = client

    def get_configurations(self):
        configs = {}
        for name,client in self.clients.items():
            config = client.get_configurations(timeout=2)
            config_filterd = {}
            for key in client.args:
                config_filterd[key] = config.get(key)
            configs[name] = config_filterd

        return configs

    def __clients_callback(self,name,config,isInitial):
        self.__callback(self,name,config,isInitial)


#def callback(config):
    #print(config)
    #rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    rospy.loginfo("Starting")

    manager = dynClientsManager()
    manager.add("dynamic_tutorials/testServer1",["int_param"])
    manager.add("dynamic_tutorials/testServer2",["double_param"])

    r = rospy.Rate(0.5)
    x = 0
    b = 0
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        manager.clients["dynamic_tutorials/testServer2"].update({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
        r.sleep()