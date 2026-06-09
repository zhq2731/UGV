
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



yaml_name=~/vehicle_platform.yaml
echo $yaml_name

key_name="vehicle_type"
vehicle_type_name=($(read_key $yaml_name $key_name))
echo "$vehicle_type_name"

key_name="ins_type"
ins_type_name=($(read_key $yaml_name $key_name))
echo "$ins_type_name"

gnome-terminal --title="pad" --tab  -e 'bash -c "source $(pwd)/devel/setup.bash; rosrun pad  pad_node ;exec bash"'


gnome-terminal --title="control-planning"   -x bash -c "source $(pwd)/devel/setup.bash; roslaunch  launch_node platoon.launch vehicle_type:=${vehicle_type_name};exec bash"

cd ~/UGV