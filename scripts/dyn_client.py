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
        self.__client   = dynamic_reconfigure.client.Client(self.name, timeout=30)
        
        self.__ignore_callback = True  # ignore the callback for timming of registering
        self.__client.set_config_callback(self.__going_callback)

        self.__initialized        = False
        self.__callback_even_same = False

    def update(self,config,must_fire_callback=False):
        self.__callback_even_same = must_fire_callback
        self.__client.update_configuration(config)

    def get_configurations(self,timeout=None):
        return self.__client.get_configuration(timeout)

    def __filter_configs(self,config):
        lists = {}
        for key in self.args:
            lists[key] = config.get(key)

        return lists

    def __going_callback(self,config):

        if config is None:
            return   # If the dynamic reconfigure contains is empty.

        if self.__ignore_callback is True:
            rospy.logdebug("Callback registering was Ignored.")
            self.__ignore_callback = False
            return

        # Check something changed or remains
        lists = self.__filter_configs(config)
        if (lists == self.__config) and not self.__callback_even_same:
            rospy.logwarn("Dyn Callback not called due to same")
            self.__callback_even_same = False
            return    # If any of specified args is not changed

        # Filter actual changed
        new_config = {}
        for k,v in lists.items():
            if not self.__config.has_key(k) or v != self.__config[k]:
                new_config[k] = v

        if len(new_config) == 0:
            return

        self.__config = lists


        if self.__combine:
            self.__callback({self.name:new_config},not self.__initialized)
        else:
            self.__callback(self.name,new_config,not self.__initialized)

        self.__initialized = True
        self.__callback_even_same = False


class dynClientsManager:

    def __init__(self,callback):
        self.clients = {}
        self.__callback = callback

    def add(self,name,args):
        client = dynClient(name,args,self.__clients_callback,False)
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