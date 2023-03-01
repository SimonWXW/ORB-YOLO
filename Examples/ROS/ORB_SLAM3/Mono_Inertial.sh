''' This is for offline running

gnome-terminal --title="ros master" -x bash -c "roscore"
sleep 2s;  
 
gnome-terminal --title="monocular" -x bash -c "roslaunch usb_cam usb_cam-test.launch"
sleep 2s;

# 启动IMU节点 
gnome-terminal --title="imu" -x bash -c "roslaunch razor_imu_m0_driver driver_node.launch"
sleep 2s;
  
gnome-terminal --title="ORB-SLAM3" -x bash -c "rosrun ORB_SLAM3 Mono_Inertial ORBvoc.txt Mono_Inertial.yaml"
sleep 2s;
 
gnome-terminal --title="ros communication" -x bash -c "rqt_graph"
'''

'''This is for rosbag running'''

# 启动ros
gnome-terminal --tab --title="roscore" --command="bash -c 'roscore; exec bash'"

# wait 1 seconds for the roscore to start
sleep 1

# 播放rosbag中数据，发布image和imu话题
gnome-terminal --tab --title="rosbag" --command="bash -c 'rosbag play 2023-02-16-11-10-21.bag; exec bash'"

# 运行orb-slam3程序
gnome-terminal --tab --title="ORB_SLAM3" --command="bash -c 'rosrun ORB_SLAM3 Mono_Inertial ORBvoc.txt Mono_Inertial.yaml; exec bash'"


# gnome-terminal --tab --title="rqt" --command="bash -c 'rqt_graph; exec bash'"