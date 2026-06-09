
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



gnome-terminal --title="control-planning1"   -x bash -c "source ~/UGV/devel/setup.bash; roslaunch  launch_node platoon-simulate_A.launch vehicle_type:=${vehicle_type_name} openrviz:=true   namespace:=A;exec bash"
sleep  3
gnome-terminal --title="control-planning2"   -x bash -c "source ~/UGV/devel/setup.bash; roslaunch  launch_node platoon-simulate_B.launch vehicle_type:=${vehicle_type_name} openrviz:=false  namespace:=B;exec bash"
sleep 3
gnome-terminal --title="control-planning3"   -x bash -c "source ~/UGV/devel/setup.bash; roslaunch  launch_node platoon-simulate_C.launch vehicle_type:=${vehicle_type_name} openrviz:=false  namespace:=C;exec bash"
sleep 3
gnome-terminal --title="control-planning4"   -x bash -c "source ~/UGV/devel/setup.bash; roslaunch  launch_node platoon-simulate_D.launch vehicle_type:=${vehicle_type_name} openrviz:=false  namespace:=D;exec bash"

#gnome-terminal --title="pad" --tab  -e 'bash -c "source ~/UGV/devel/setup.bash; rosrun pad  pad_node ;exec bash"'

cd ~/UGV