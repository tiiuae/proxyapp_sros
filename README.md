# Description

This is a relay application used to demonstrate SROS configuration.

It accepts service calls to /<namespace>/gps_waypoint and formwards them to navigation node.

## Run the proxyapp_ros with SROS
```
ROS_DOMAIN_ID=1 \
SOFTHSM2_CONF=<softhsm-configuration> \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
ROS_SECURITY_KEYSTORE=<sros-keystore> \
ROS_SECURITY_ENABLE=true \
ROS_SECURITY_STRATEGY=Enforce \
proxyapp_ros --ros-args --enclave /fog/proxyapp -r __node:=$DRONE_DEVICE_ID -p drone_id:=$DRONE_DEVICE_ID -p drone_namespace:=$DRONE_DEVICE_ID
```

## Run the proxyapp_ros without SROS
```
proxyapp_ros --ros-args -r __node:=$DRONE_DEVICE_ID -p drone_id:=$DRONE_DEVICE_ID -p drone_namespace:=$DRONE_DEVICE_ID
```

# Server and client test applications

This repo includes an application that can be used to test the proxyapp_ros ROS2 node.

## Run the server with SROS
```
ROS_DOMAIN_ID=1 \
SOFTHSM2_CONF=<softhsm-configuration> \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
ROS_SECURITY_KEYSTORE=<sros-keystore> \
ROS_SECURITY_ENABLE=true \
ROS_SECURITY_STRATEGY=Enforce \
proxyapp_server_client server $DRONE_DEVICE_ID $DRONE_DEVICE_ID --ros-args --enclave /<sros-enclave>
```

## Run the server without SROS
```
proxyapp_server_client server $DRONE_DEVICE_ID $DRONE_DEVICE_ID
```

## Run the client with SROS
```
ROS_DOMAIN_ID=1 \
SOFTHSM2_CONF=<softhsm-configuration> \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
ROS_SECURITY_KEYSTORE=<sros-keystore> \
ROS_SECURITY_ENABLE=true \
ROS_SECURITY_STRATEGY=Enforce \
proxyapp_server_client client $DRONE_DEVICE_ID $DRONE_DEVICE_ID --ros-args --enclave /<sros-enclave>
```

## Run the client without SROS
```
proxyapp_server_client client $DRONE_DEVICE_ID $DRONE_DEVICE_ID
```
