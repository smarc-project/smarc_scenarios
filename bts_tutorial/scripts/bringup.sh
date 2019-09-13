SESSION=bts_tutorial
# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'rviz'
tmux new-window -t $SESSION:2 -n 'biograd_coast'
tmux new-window -t $SESSION:3 -n 'auv_sim'
tmux new-window -t $SESSION:4 -n 'navigation'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "rviz" C-m

tmux select-window -t $SESSION:2
tmux send-keys "rosrun bts_tutorial source_models.bash" C-m
tmux send-keys "roslaunch bts_tutorial biograd_coast.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch bts_tutorial auv_sim.launch"

tmux select-window -t $SESSION:4
tmux send-keys "roslaunch bts_tutorial navigation.launch"

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
