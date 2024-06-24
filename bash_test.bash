#!/bin/bash

# This is a comment
#mavlink UDP 14581, remote port: 14541)

# ros2 launch mavros px4.launch  namespace:=drone0 fcu_url:=udp://:14540@127.0.0.1:14557 
# ros2 launch mavros px4.launch  namespace:=drone0 fcu_url:=udp://:14ijkj541@127.0.0.1:14581
ros2 launch mavros px4.launch  namespace:=drone0 fcu_url:="udp://:14540@127.0.0.1:14557"
