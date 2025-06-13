#!/bin/bash

tmux has-session -t px4 2>/dev/null && tmux kill-session -t px4
tmux new-session -d -s px4 -n main 
tmux split-window -h -t px4:0
tmux split-window -v -t px4:0.0
tmux split-window -v -t px4:0.0
tmux split-window -v -t px4:0.0
tmux split-window -v -t px4:0.0
tmux select-layout -t px4:0 main-vertical
tmux resize-pane -t px4:0.0 -x 70%

# Set pane titles
tmux select-pane -t px4:0.0 -T 'MavLink Shell'
tmux select-pane -t px4:0.1 -T 'Mocap'
tmux select-pane -t px4:0.2 -T 'MavProxy'
tmux select-pane -t px4:0.3 -T 'Forwarding Script'
tmux select-pane -t px4:0.4 -T 'Vehicle Position Listener'
tmux select-pane -t px4:0.5 -T 'Trajectory Setpoint Listener'
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format '#{pane_title}'

# Execute commands
tmux send-keys -t px4:0.1 'sleep 3; export MOCAP_HOST=128.238.39.121' C-m 'mocap' C-m
tmux send-keys -t px4:0.2 '. ~/venvs/global/bin/activate; mavproxy.py --master=tcp:192.168.8.4:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --out=udp:127.0.0.1:14554' C-m
tmux send-keys -t px4:0.3 'echo "waiting for Mocap..."; sleep 5' C-m '. .venv/bin/activate' C-m 'export MAVLINK_POSE_TOPIC="/vicon/race_jonas/pose"' C-m 'export MAVLINK_URL="udp:localhost:14551"' C-m 'python3 tools/external_position_reference/forward_ros_to_mavlink.py' C-m
tmux send-keys -t px4:0.4 'sleep 3; (while true; do sleep 2; echo "listener vehicle_local_position"; done) | python3 px4_autopilot/Tools/mavlink_shell.py localhost:14552 | grep -A2 \\sx\:' C-m
tmux send-keys -t px4:0.5 'sleep 3; (while true; do sleep 2; echo "listener trajectory_setpoint"; done) | python3 px4_autopilot/Tools/mavlink_shell.py localhost:14553 | grep -A2 position\:' C-m
tmux send-keys -t px4:0.0 'sleep 3; python3 px4_autopilot/Tools/mavlink_shell.py localhost:14554' C-m
tmux select-pane -t px4:0.0
tmux attach -t px4
