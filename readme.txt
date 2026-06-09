代码编译：
cd UGV
source compile.sh


实车启动程序的步骤：
cd UGV 
source  roscore_ins.sh      # 用于启动roscore 组合惯导 地盘驱动 pad界面。

source  start.sh            # 用于启动无人算法相关的模块 参考线 规划 横纵向控制。
                            # 不调试算法相关的模块可以不启动

									
电脑仿真：
source  start-simulate.sh  #具体都有哪些操作 ，可直接看start-simulate.sh脚本文件

录制轨迹：
实车启动程序的步骤之后，界面点击“录制轨迹开始” ，内部程序就会开始录制轨迹。
点击“录制轨迹停止” ，则录制完毕。
 
说明：录制轨迹目前是覆盖+追加的方式。覆盖：会覆盖掉之前的gpsData.txt
追加：在录制过程中可多次点击录制开始以及结束，所有轨迹点都会记录到同一个gpsData.txt。
录制完毕之后，重新启动start.sh，就会读取新录制的轨迹。


							
