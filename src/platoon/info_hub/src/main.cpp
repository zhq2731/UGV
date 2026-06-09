#include "info_hub/info_hub.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"info_hub_node");
    ros::NodeHandle nh;
    InfoHub InfoHub_(nh);
    ros::spin();
    return 0;
}

