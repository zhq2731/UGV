
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



gnome-terminal --title="roscore" --tab -e 'bash -c "roscore"'

sleep 1s

if [ "$ins_type_name" == '"cgi610"' ]
then
	echo "instype is cgi610"
	gnome-terminal --title="ins_driver_can_cgi610_recevier" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun  ins_driver_can_cgi610 ins_driver_can_cgi610_recevier;exec bash"'
elif [ "$ins_type_name" == '"cgi1010"' ]
then
	echo "instype is cgi1010"
	gnome-terminal --title="ins_driver_cgi1010_recevier" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun  ins_driver_cgi1010 ins_driver_cgi1010_recevier ;exec bash"'
elif [ "$ins_type_name" == '"cgi430"' ]
then
	echo "instype is cgi430"
	echo 'nvidia' | sudo -S  chmod  777 /dev/ttyUSB0
	gnome-terminal --title="ins_driver_430" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun  ins_driver_430 ins_driver_430 ;exec bash"'
elif [ "$ins_type_name" == '"xinxiang"' ]
then
	echo "instype is xinxiang"
	gnome-terminal --title="ins_driver_xinxiang" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun  ins_driver_xinxiang ins_driver_xinxiang_recevier ;exec bash"'
elif [ "$ins_type_name" == '"ht33"' ]
then
    echo 'instype is ht33'
    echo '123456' | sudo -S  ifconfig enp10s0 promisc
	gnome-terminal --title="ins_driver_33_recevier" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun ins_driver_33 ins_driver_33_recevier ;exec bash"'
else
    echo "wrong vehicle_type "
fi

sleep 1s


if [ "$vehicle_type_name" == '"tank500"' ]
then
    gnome-terminal --title="chassis_driver_tank_receiver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_tank chassis_driver_tank_recevier;exec bash"'
	gnome-terminal --title="chassis_driver_tank_sender" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_tank chassis_driver_tank_sender;exec bash"'
	sleep 1s
	echo "type is 500"
elif [ "$vehicle_type_name" == '"sanzhou"' ]
then	
	gnome-terminal --title="chassis_driver_sanzhou_sender" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_szc chassis_driver_szc_sender;exec bash"'
	gnome-terminal --title="chassis_driver_sanzhou_receiver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_szc chassis_driver_szc_recevier;exec bash"'
	sleep 1s
	echo "type is sanzhou"
elif [ "$vehicle_type_name" == '"x6000"' ]
then	
	gnome-terminal --title="chassis_driver_x6000_sender" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_x6000 chassis_driver_x6000_sender;exec bash"'
	gnome-terminal --title="chassis_driver_x6000_receiver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_x6000 chassis_driver_x6000_recevier;exec bash"'
	sleep 1s
	echo "type is x6000"
elif [ "$vehicle_type_name" == '"xb"' ]
then	
	gnome-terminal --title="chassis_driver_xb_sender" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_xb chassis_driver_xb_sender;exec bash"'
	gnome-terminal --title="chassis_driver_xb_receiver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_xb chassis_driver_xb_recevier;exec bash"'
	sleep 1s
	echo "type is xb"	
	
elif [ "$vehicle_type_name" == '"zhito"' ]
then
    gnome-terminal --title="chassis_driver_zt" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_node chassis_driver_node;exec bash"'
    #gnome-terminal --title="chassis_driver_zhito_receiver" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_zhito chassis_driver_zhito_recevier;exec bash"'
    #gnome-terminal --title="chassis_driver_zhito_sender" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun  chassis_driver_zhito chassis_driver_zhito_sender;exec bash"'	
	sleep 1s
	echo "type is zhito"
elif [ "$vehicle_type_name" == '"zhitong"' ]
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
	sleep 2s
    gnome-terminal --title="chassis_driver_zt" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun chassis_driver_node chassis_driver_node;exec bash"'
	sleep 1s
#	gnome-terminal --title="tztek"   -x bash -c "echo 'nvidia' | sudo -S  tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 30 1000;exec bash"
#	sleep 5s
#	gnome-terminal --title="srt"   -x bash -c "srt-live-transmit srt://:4200 srt://:4201 -v;exec bash"
#	sleep 10s
#	gnome-terminal --title="myCapture"   -x bash -c "~/myCapture/cmake-build-debug/myCapture ;exec bash"
	echo "type is zhitong"
else
    echo "wrong vehicle_type "
fi


gnome-terminal --title="pad" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun pad  pad_node ;exec bash"'


gnome-terminal --title="vehicle_gate_cmd" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun vehicle_cmd_gate vehicle_cmd_gate;exec bash"'




