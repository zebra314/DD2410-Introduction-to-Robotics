#!/bin/zsh
source /opt/ros/noetic/setup.zsh
cd /root/irob_ws && catkin_make
clear
tmux
exec "$@"