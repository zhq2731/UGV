#ifndef ALLDECLARE_H
#define ALLDECLARE_H

#include "pad/alltypes.h"
#include <QMutex>

extern MapCorditCFST g_stMapCorditCmd;/*地图坐标系参数*/
extern bool g_bSetMapOrigin;/*是否选定了原点的标志*/

/*程序配置参数部分*/
extern UINT32 USE_VEH_SIMU;/*车辆无人控制算法仿真 1:有效 N/A:无效*/

/*车辆参数配置部分start*/
extern double VEHICLE_MASS;/*kg*/
extern double VEHICLE_WHEEL_BASE;/*轴距，单位：m*/

extern double VEHICLE_LENGTH;/*车长，单位：m*/
extern double VEHICLE_WIDTH;/*车宽，单位：m*/
extern double VEHICLE_HEIGHT;/*车高，单位：m*/

extern double CHAS_MOVE_CTRL_PERIOD;/*一个底盘运动控制周期，单位s*/

/*车辆参数配置部分end*/
extern double MAX_LINE_SPEED;/*最高纵向速度，单位m/s*/
extern double MAX_TURN_ANGLE;/*最高右转向角度，单位°*/
extern double MIN_TURN_ANGLE;/*最高左转向角度，单位°*/

extern double TURN_ANGLE_SPEED;/*最大角速度，单位：°/s*/

extern double GRAVITY_ACC;/*重力加速度，单位：m²/s*/

extern double ELECTRIC_FENCE; /*电子围栏路宽，单位：m*/

#endif // ALLDECLARE_H
