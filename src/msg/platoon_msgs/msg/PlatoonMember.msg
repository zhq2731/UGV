Header header
string id                       #车辆标识
geometry_msgs/Point position    #position.x x坐标 y坐标
geometry_msgs/Point latLon      #latLon.x纬度  latLon.y经度
float64 heading                 #车辆的航向
float64 linear_velocity         #车辆前向速度
float64 linear_acceleration     #车辆的前向加速度
float64 spacing_distance        #实际车间距(与前车)
float64 spacing_distance_error  #车间距误差，与前车的实际距离-期望距离。期望距离为1.5v+5
float64 front_overhang_m  #前悬
float64 rear_overhang_m   #后悬
float64 wheel_base_m      #轴距
float64 vehicle_length_m  #车长
float64 vehicle_width_m   #车宽
float64 vehicle_height_m  #车高
uint8 num   #车辆编号（唯一）
uint8 role  #0-跟随 1-引导
uint8 type  #见PlatoonMission中定义，表示当前车辆目前的type
uint8 gear  #0：空挡 1-6：前进档 7：倒档 8：无效
uint8 driving_mode  #0：人工 1：自动
uint8 parking_brake          #
uint8 err_type               #取值范围与type相同，标识产生当前误差的时候，车辆的type
uint8 ins_status             #0-正常  1-不正常
uint8 index                  #0-领航  1 2 3
#北理加的信息
float64   max_spacing_error  #用于给后车使用
float64[] acc_sequence       #加速度序列，用于给后车
float64[] velocity_sequence  #速度序列，用于给后车

float64 throttle_pedal
float64 steering_wheel_angle
float64 brake_pedal










