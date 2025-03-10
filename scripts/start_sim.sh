#!/bin/bash

# Setup environment
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo $ROS_MASTER_URI
MODEL="uav_vision"

# start mavros in external terminal
xterm -e "$SCRIPT_DIR/start_mavros.sh ; exec /bin/bash" &

roslaunch $SCRIPT_DIR/../launch/uav_simulator.launch \
          sdf:=$SCRIPT_DIR/../sim/models/$MODEL/$MODEL.sdf \
          world:=$SCRIPT_DIR/../sim/worlds/aruco_world.world \

