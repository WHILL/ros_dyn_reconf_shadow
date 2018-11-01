#!/usr/bin/env python

import rospy

from pprint import pprint

import dynamic_reconfigure.client

class dynClient:
    def __init__(self,name,args,callback,combine=True):
        # @TODO Needs type check for arguments!
        self.name       = name
        self.args       = args
        self.__callback = callback
        self.__combine  = combine
        self.__config   = {}
        self.__client   = dynamic_reconfigure.client.Client(self.name, timeout=30, config_callback=self.__going_callback)

    def update(self,config):
        self.__client.update_configuration(config)

    def __going_callback(self,config):
        lists = {}
        for key in self.args:
            lists[key] = config.get(key)

        if lists == self.__config:
            return    # If any of specified args is not changed
        
        self.__config = lists

        if self.__combine:
            self.__callback({self.name:lists})
        else:
            self.__callback(self.name,lists)


class dynClientsManager:

    def __init__(self):
        self.clients = {}

    def add(self,name,args):
        client = dynClient(name,args,self.__clients_callback)
        self.clients[name] = client

    def __clients_callback(self,config):
        pprint(config,width=40)



def callback(config):
    print(config)
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