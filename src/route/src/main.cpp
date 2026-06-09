#include "route/route.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"route_node");
    ros::NodeHandle nh;
    Route  route(nh);
    ros::spin();
    return 0;
}

