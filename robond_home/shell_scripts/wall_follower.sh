#!/bin/bash

# Directory + Package Setup ...
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/setup.sh

echo 'launching world'
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find robond_home)/world/home.world" &
sleep 5

echo 'launching gmapping'
#xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
xterm -e "roslaunch robond_home gmapping.launch" &

#echo 'launching hector mapping'
#xterm -e "roslaunch robond_home hector_mapping.launch" &
sleep 2

echo 'launching rviz'
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2

echo 'launching follower'
xterm -e "rosrun robond_home wall_follower cmd_vel:=/mobile_base/commands/velocity"
