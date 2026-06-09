#include "velocity_planner/velocity_planner_flow.h"
#include "ros/ros.h"
#include "ros/package.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_planner_node");
    ros::NodeHandle node_handle("~");
    VelocityPlannerFlow velocity_planner_flow(node_handle);
   //100ms执行一次
    ros::Rate rate(10);
    while (ros::ok()) {
        //运行RUN函数
        velocity_planner_flow.Run();
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}