# ros_aws_iot_bridge
Sync parameters with AWS IoT ThingShadow by Dynamic Reconfigure Interface

## Build and Install

Clone this repository and place on your `catkin_ws`, and then build.

### Dependencies
```
rosdep install ros_aws_iot_bridge
```


## Run

```
roslaunch ros_aws_iot_bridge syncShadow.launch
```

## Parameters


### ~host (string, Required)
Specify your AWS IoT host. *(e.g.: **your-specific-host**.iot.**us-east-1**.amazonaws.com)*

### ~thingName (string, Required)
Specify your Thing name to sync.

### ~clientId  (string, default: ROS)

### ~certificatePath  (string, default: certs/certificate.pem.crt)
Specify your certification

### ~privateKeyPath (string, default: certs/private.pem.key)
Specify your private key

### ~rootCAPath (string, default: certs/rootCA.pem)
Seecify the rootCA of AWS.

### ~port (string, default: None)
No need to change normally.

### ~useWebsocket (Boolean, default: None)
