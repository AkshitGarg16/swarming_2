#!/bin/bash

SESSION_NAME="uav_swarm_grid"

# 1. Check if the session already exists. If it does, kill it.
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? == 0 ]; then
    echo "Old session found. Shutting down all terminals..."
    tmux kill-session -t $SESSION_NAME
    sleep 1 # Give it a second to close ports fully
fi

echo "Starting fresh tmux grid session: $SESSION_NAME"

# 2. Start the session with Pane 1 (MAVProxy 1)
tmux new-session -d -s $SESSION_NAME -n 'Swarm_GCS'
tmux send-keys -t $SESSION_NAME 'mavproxy.py --master=/dev/ttyUSB0 --out=udp:0.0.0.0:14555 --out=udp:0.0.0.0:14553' C-m

# Enable mouse support by default so you can click to navigate
tmux set-option -t $SESSION_NAME mouse on

# 3. Split vertically to create Pane 2 (MAVProxy 2) on the right half
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME 'mavproxy.py --master=/dev/ttyUSB1 --out=udp:0.0.0.0:14565 --out=udp:0.0.0.0:14554' C-m

# 4. Split Pane 1 (left side) horizontally for the Main Guided script
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME 'python3 main_height_diff_guided.py' C-m

# 5. Split Pane 2 (right side) horizontally for Drone 1 GCS
tmux split-window -v -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME 'python3 drone1_fullgcs.py' C-m

# 6. Split Pane 4 (bottom right) horizontally again to fit Drone 2 GCS
tmux split-window -v -t $SESSION_NAME:0.3
tmux send-keys -t $SESSION_NAME 'python3 drone2_fullgcs.py' C-m

# 7. Balance layout sizes neatly
tmux select-layout -t $SESSION_NAME tiled

# 8. Attach to view the dashboard
tmux attach-session -t $SESSION_NAME
