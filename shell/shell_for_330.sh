#!/bin/zsh

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n color_detect_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: location.launch
tmux split-window -h -t ros_session:0
tmux split-window -v -t ros_session:0.1

tmux send-keys -t ros_session:0.1 'sleep 7; source ~/group2_ws/devel/setup.bash; roslaunch scan_land scan_land.launch' C-m
tmux send-keys -t ros_session:0.2 'sleep 10; roslaunch scan_land scan_land_py.launch' C-m
# 整理第一个窗口布局
tmux select-layout -t ros_session:0 tiled

# 附加到会话并显示第一个窗口
tmux select-window -t ros_session:0
tmux attach-session -t ros_session:0