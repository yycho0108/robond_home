#!/bin/bash

# Directory + Package Setup ...
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/setup.sh

echo 'launching world'
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find robond_home)/world/home.world" &
sleep 5

echo 'launching AMCL'
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find robond_home)/world/map.yaml" &

echo 'launching RViz'
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2

echo 'launching Marker Visualization'
xterm -e "rosrun robond_home add_marker" &
sleep 2
