#!/bin/bash

session="client"

tmux start-server


tmux new-session -d -s $session
tmux split-window -v
tmux split-window -h
tmux selectp -t 0
tmux split-window -h

tmux setw -g mouse on

tmux_send_all() {
	for _pane in $(tmux list-panes -F '#P'); do
		tmux send-keys -t ${_pane} "$@"
	done
}

sudo /etc/init.d/chrony stop
sudo rm /etc/chrony/chrony.conf
sudo cp chrony.conf /etc/chrony
sudo echo "server $(cat master_ip.conf) iburst prefer" >> /etc/chrony/chrony.conf
sudo /etc/init.d/chrony start

tmux_send_all "export ROS_MASTER_URI=http://$(cat master_ip.conf):11311" C-m
tmux_send_all "export ROS_IP=$(cat client_ip.conf)" C-m
tmux_send_all "source /opt/ros/noetic/setup.bash" C-m
tmux selectp -t 0
tmux send-keys "cd .." C-m
tmux send-keys "git submodule update --init -f" C-m
tmux send-keys "cd client/client_ws" C-m
tmux send-keys "catkin_make install" C-m
tmux send-keys "catkin_make install" C-m
tmux send-keys "cd .." C-m
tmux send-keys "./after_start.sh" C-m

tmux new-window -t $session:1 -n scratch
tmux select-window -t $session:0
tmux attach-session -t $session

