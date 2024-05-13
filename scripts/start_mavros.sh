#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo $ROS_MASTER_URI
MODEL="uav_vision"

# Wait for the /clock topic to become available
while ! rostopic list | grep -q '/clock'; do
    sleep 1
done

roslaunch $SCRIPT_DIR/../launch/custom_mavros.launch \
            fcu_url:="udp://:14540@localhost:14557" \
            config_file:=$SCRIPT_DIR/../config/mavros_config.yaml \
            pluginlists_file:=$SCRIPT_DIR/../config/mavros_blacklist.yaml 