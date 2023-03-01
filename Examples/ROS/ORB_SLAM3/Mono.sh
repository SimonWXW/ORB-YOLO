# write a gnome-terminal command to open a new terminal
# the name of the new terminal is "roscore"
# and run the "roscore" command in this new terminal
gnome-terminal --tab --title="roscore" --command="bash -c 'gcc -v; exec bash'"

# wait 2 seconds for the roscore to start
sleep 2

# write a gnome-terminal command to open another new terminal
# the name of the new terminal is "monocular"
# and run the "usb" command in this new terminal
gnome-terminal --tab --title="monocular" --command="bash -c 'roslaunch usb_cam usb_cam-test.launch; exec bash'"

# wait 2 seconds for the camera to launch
sleep 2

# write a gnome-terminal command to open another new terminal
# the name of the new terminal is "ORB_SLAM3"
# and run the "rosrun" command in this new terminal
gnome-terminal --tab --title="ORB_SLAM3" --command="bash -c 'rosrun ORB_SLAM3 Mono ORBvoc.txt Mono.yaml; exec bash'"

# write a gnome-terminal command to open another new terminal
# the name of the new terminal is "rqt"
# and run the "rqt_graph" command in this new terminal
gnome-terminal --tab --title="rqt" --command="bash -c 'rqt_graph; exec bash'"