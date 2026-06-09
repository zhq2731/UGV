
#!/bin/bash
function read_key(){
    key=$2
    cat $1 | while read LINE
    do 
            # 属性开始标志 vehicle_type:
        if [ "$(echo $LINE | grep "$key:")" != "" ];then
            if [ "$(echo $LINE | grep -E ' ')" != "" ];then
            	# 截取出key值
                echo "$LINE" | awk -F " " '{print $2}'
                continue
            else
            	# 如果关键词后面没有空格，则跳出继续查找
                continue
            fi
        fi
    done
}

cd ~
yaml_name=$(pwd)/vehicle_platform.yaml
echo $yaml_name

key_name="vehicle_type"
vehicle_type_name=($(read_key $yaml_name $key_name))
echo "$vehicle_type_name"

key_name="ins_type"
ins_type_name=($(read_key $yaml_name $key_name))
echo "$ins_type_name"



#gnome-terminal --title="roscore" --tab -e 'bash -c "roscore"'

#sleep 1s

if [ "$vehicle_type_name" == '"zhitong"' ]
then

	echo 'nvidia' | sudo -S  ip link set down can0
	#sudo ip link set down can0
	sudo ip link set can0 type can bitrate 500000 sample-point  0.875
	#sudo ip link set can0 type can bitrate 500000 berr-reporting on restart-ms 100

	sudo ifconfig can0 txqueuelen 1000
	sudo ip link set up can0

	sudo ip link set down can1
	sudo ip link set can1 type can bitrate 500000 sample-point  0.875
	sudo ifconfig can1 txqueuelen 1000
	sudo ip link set up can1
	sleep 1s
#	gnome-terminal --title="tztek"   -x bash -c "echo 'nvidia' | sudo -S  tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 30 1000;exec bash"
#	sleep 5s
#	gnome-terminal --title="srt"   -x bash -c "srt-live-transmit srt://:4200 srt://:4201 -v;exec bash"
#	sleep 10s
#	gnome-terminal --title="myCapture"   -x bash -c "~/myCapture/cmake-build-debug/myCapture ;exec bash"
	echo "type is zhitong"
fi

#gnome-terminal --title="ins_driver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun  ins_driver_lib ins_driver_lib_recevier;exec bash"'

#sleep 1s

gnome-terminal --title="chassis_driver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_node chassis_driver_node;exec bash"'

sleep 1s

gnome-terminal --title="pad" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun pad  pad_node ;exec bash"'

sleep 1s

gnome-terminal --title="vehicle_gate_cmd" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun vehicle_cmd_gate vehicle_cmd_gate;exec bash"'




