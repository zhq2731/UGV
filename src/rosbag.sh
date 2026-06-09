gnome-terminal --title="rosbag"   -x bash -c "roscore; exec bash" &
sleep 2
gnome-terminal --title="rosbag"   -x bash -c "rosbag play loop_large_two.bag;exec bash"
