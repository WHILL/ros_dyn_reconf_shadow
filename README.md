# dyn_reconf_shadow
Sync Dynamic Reconfigure Parameters with AWS IoT ThingShadow

## Build and Install

Clone this repository and place on your `catkin_ws`, and then build.

### Dependencies
```
rosdep install dyn_reconf_shadow
```


## Run

```
rosrun dyn_reconf_shadow sync.py _host:=<hostname> _thingName:=<thingname> _reconf_config:=<config yaml file path>
```

## Parameters


### ~host (string, Required)
Specify your AWS IoT host. *(e.g.: **your-specific-host**.iot.**us-east-1**.amazonaws.com)*

### ~thingName (string, Required)
Thing name on AWS IoT to sync.

### ~clientId  (string, default: ROS)

### ~certificatePath  (string, default: ~/.awsiot/certificate.pem.crt)
Certification of AWS IoT

### ~privateKeyPath (string, default: ~/.awsiot/private.pem.key)
Private Key for AWS IoT

### ~rootCAPath (string, default: ~/.awsiot/AmazonRootCA1.pem)
Root Certification for Amazon

### ~port (string, default: None)
No need to change normally.

## ~reconf_config(Required, string)
Config file yaml of reconfigure server name and factors to sync.
See sample yaml: `config/sample-config.yaml`

### ~useWebsocket (Boolean, default: None)


