#!/bin/zsh
tmux split-window -h "sleep 10; roslaunch scan_land scan_land_py.launch"
sleep 7; source ~/group2_ws/devel/setup.bash; roslaunch scan_land scan_land.launch
