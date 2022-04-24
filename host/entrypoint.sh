#!/bin/bash


session="host"

tmux start-server

tmux new-session -d -s $session
tmux split-window -v
tmux split-window -h
tmux selectp -t 0
tmux split-window -h

tmux_send_all() {
	for _pane in $(tmux list-panes -F '#P'); do
		tmux send-keys -t ${_pane} "$@"
	done
}

tmux_send_all "source /opt/ros/noetic/setup.bash" C-m
tmux_send_all "export ROS_MASTER_URI=http://172.17.0.2:11311" C-m
tmux_send_all "export ROS_MASTER_IP=172.17.0.2" C-m

tmux selectp -t 0
tmux send-keys "cd /home/l3xz/host_ws" C-m
tmux send-keys "catkin_make install" C-m

tmus_send_all "source /home/l3xz/host_ws/devel/setup.bash" C-m

tmux send-keys "roslaunch rosbridge_server rosbridge_websocket.launch" C-m

tmux selectp -t 1
tmux send-keys "cd /home/l3xz/host_ws/src/webgui" C-m
tmux send-keys "python3 -m http.server 8000" C-m

tmux selectp -t 2
tmux send-keys "cd /home/l3xz" C-m

tmux selectp -t 3
tmux send-keys "cd /home/l3xz" C-m

tmux new-window -t $session:1 -n scratch
tmux select-window -t $session:0
tmux attach-session -t $session
