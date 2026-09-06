#!/usr/bin/env bash
# TMR ROS 2 environment for the robot host (.100)
source /home/aup/recloned_sources/source_migrated_stack.sh

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/aup/cyclonedds.xml
