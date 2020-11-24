#include<ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obiOne");
    ros::NodeHandle nh;
    ROS_INFO("simple program test ros init");
    ros::spinOnce();
    return 0;

}