#!/bin/bash

tmux_send_all() {
	for _pane in $(tmux list-panes -F '#P'); do
		tmux send-keys -t ${_pane} "$@"
	done
}

tmux_send_all "source client_ws/devel/setup.bash" C-m
tmux selectp -t 0
tmux send-keys "roslaunch l3xz_mapping realsense.launch" C-m
tmux selectp -t 1
tmux send-keys "roslaunch l3xz_mapping lidar.launch" C-m
tmux selectp -t 2
tmux send-keys "roslaunch l3xz_mapping rgb.launch" C-m
tmux selectp -t 3
tmux send-keys "roslaunch l3xz_mapping thermal.launch" C-m
