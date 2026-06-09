#include "perception_simulate/perception.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;

    ObstacleSubscriber subscriber(nh);

    ros::spin();
    return 0;
}
