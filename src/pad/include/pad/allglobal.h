#ifndef ALLGLOBAL_H
#define ALLGLOBAL_H

#include "pad/alldeclare.h"

MapCorditCFST g_stMapCorditCmd;/*地图坐标系参数*/
bool g_bSetMapOrigin;/*是否选定了原点的标志*/

/*程序配置参数部分*/
UINT32 USE_VEH_SIMU;/*车辆无人控制算法仿真 1:有效 N/A:无效*/

/*车辆参数配置部分start*/
double VEHICLE_MASS;/*kg*/
double VEHICLE_WHEEL_BASE;/*轴距，单位：m*/

double VEHICLE_LENGTH;/*车长，单位：m*/
double VEHICLE_WIDTH;/*车宽，单位：m*/
double VEHICLE_HEIGHT;/*车高，单位：m*/

double CHAS_MOVE_CTRL_PERIOD;/*一个底盘运动控制周期，单位s*/

/*车辆参数配置部分end*/
double MAX_LINE_SPEED;/*最高纵向速度，单位m/s*/
double MAX_TURN_ANGLE;/*最高右转向角度，单位°*/
double MIN_TURN_ANGLE;/*最高左转向角度，单位°*/

double TURN_ANGLE_SPEED  ;/*最大角速度，单位：°/s*/

double GRAVITY_ACC;/*重力加速度，单位：m²/s*/

double ELECTRIC_FENCE; /*电子围栏路宽，单位：m*/
#endif // ALLGLOBAL_H
