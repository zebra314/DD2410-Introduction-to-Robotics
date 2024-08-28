#!/bin/zsh
source /opt/ros/melodic/setup.zsh
cd /root/irob_ws && catkin_make
source /root/irob_ws/devel/setup.zsh
exec "$@"