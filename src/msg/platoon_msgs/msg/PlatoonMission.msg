#NONE  = 0,  //无效
#BUILD = 1,    //构建
#JOIN  = 2,     //入队
#LEAVE = 3,    //出队
#DISSOLVE = 4, //解散
#COLUMN = 5,   //纵队
#DIAMOND = 6,  //菱形
#TRIANGLE = 7, //三角形
#REVESE = 8,   //编队倒车
#CANCEL_REVESE = 9, //取消编队倒车
#MASS = 10,           //集结
#DISTRIBUTE = 11,     //分散
#RUNNING = 12,        //开始运行
#PARKING_SIMPLE = 13, //简单泊车
#PARKING_AUTO = 14,   //泊车
#FORCED_FORMATION = 15, //强制组队
#RESERVE_CMD  =16     //保留
Header header
uint8 command_type    #见上面的定义
#1234 ,2134
PlatoonConfig config  
uint8[] vehicle_list  #编队构建的时候有效
uint8 num             #入队离队的车辆编号

geometry_msgs/Pose[] distributePoints #分散点

uint8[] mission_nums # 需要接收到任务信息的车辆列表