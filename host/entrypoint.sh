#!/bin/bash


session="host"

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

/etc/init.d/chrony stop
rm /etc/chrony/chrony.conf
cp chrony.conf /etc/chrony
echo "allow $(cat client_ip.conf)/24" >> /etc/chrony/chrony.conf
/etc/init.d/chrony start

tmux_send_all "source /opt/ros/noetic/setup.bash" C-m
tmux_send_all "export ROS_MASTER_URI=http://$(cat master_ip.conf):11311" C-m
tmux_send_all "export ROS_MASTER_IP=$(cat master_ip.conf)" C-m

tmux selectp -t 0
tmux send-keys "cd /home/l3xz/host_ws" C-m
tmux send-keys "catkin_make install" C-m

tmux_send_all "source /home/l3xz/host_ws/devel/setup.bash" C-m

tmux selectp -t 2
tmux send-keys "cd /home/l3xz/host_ws/src/l3xz_mapping/scripts" C-m
tmux send-keys "/etc/init.d/chrony status" C-m

tmux selectp -t 3
tmux send-keys "cd /home/l3xz/host_ws/src/l3xz_mapping/scripts" C-m

tmux new-window -t $session:1 -n scratch
tmux select-window -t $session:0
tmux attach-session -t $session
