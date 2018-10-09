SESSION=oceans_tutorial
# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'launch_server'
tmux new-window -t $SESSION:2 -n 'web_view'
tmux new-window -t $SESSION:3 -n 'rqt_gui'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "rosrun roslaunch_monitor launch_server.py" C-m

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch auv_sm_mission_executor web_ui.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roscd oceans_tutorial/rqt" C-m
tmux send-keys "rqt -p tutorial.perspective" C-m

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
