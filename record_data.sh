gnome-terminal --title="record_data" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; roslaunch  launch_node record_data.launch ;exec bash"'
sleep 1s


gnome-terminal --title="pad" --tab  -e 'bash -c "source $(pwd)/devel/setup.bash; rosrun pad  pad_node ;exec bash"'
