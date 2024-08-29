#!/bin/zsh
source /opt/ros/melodic/setup.zsh
cd /root/irob_ws && catkin_make
clear
tmux
exec "$@"