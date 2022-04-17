#!/bin/bash


session="demo"

# set up tmux
tmux start-server

# create a new tmux session, starting vim from a saved session in the new window
tmux new-session -d -s $session
tmux split-window -v
tmux split-window -h
tmux selectp -t 0
tmux split-window -h

# Select pane 1, set dir to frontend, run build.sh
tmux selectp -t 0
tmux send-keys "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys "source /home/l3xz/host_ws/devel/setup.bash" C-m
tmux send-keys "export ROS_MASTER_URI=http://172.17.0.2:11311" C-m
tmux send-keys "export ROS_MASTER_IP=172.17.0.2" C-m
tmux send-keys "cd /home/l3xz/host_ws" C-m
tmux send-keys "catkin_make install" C-m
tmux send-keys "roslaunch rosbridge_server rosbridge_websocket.launch" C-m

# Split pane 1 vertical by 50%
tmux selectp -t 1
tmux send-keys "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys "source /home/l3xz/host_ws/devel/setup.bash" C-m
tmux send-keys "export ROS_MASTER_URI=http://172.17.0.2:11311" C-m
tmux send-keys "export ROS_MASTER_IP=172.17.0.2" C-m
tmux send-keys "cd /home/l3xz/host_ws/src/webgui" C-m
tmux send-keys "python3 -m http.server 8000" C-m

# Split pane 1 vertical by 50%
tmux selectp -t 2
tmux send-keys "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys "source /home/l3xz/host_ws/devel/setup.bash" C-m
tmux send-keys "cd /home/l3xz" C-m
tmux send-keys "export ROS_MASTER_URI=http://172.17.0.2:11311" C-m
tmux send-keys "export ROS_MASTER_IP=172.17.0.2" C-m

# Split pane 1 vertical by 50%
tmux selectp -t 3
tmux send-keys "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys "source /home/l3xz/host_ws/devel/setup.bash" C-m
tmux send-keys "cd /home/l3xz" C-m
tmux send-keys "export ROS_MASTER_URI=http://172.17.0.2:11311" C-m
tmux send-keys "export ROS_MASTER_IP=172.17.0.2" C-m

tmux new-window -t $session:1 -n scratch

# return to main vim window
tmux select-window -t $session:0

# Finished setup, attach to the tmux session!
tmux attach-session -t $session
