#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "arm_mover");//initializng ros
  ros::NodeHandle nh;   // creating a handler

  //Creating apublisher that can publishe a std_msgs::Float64
  ros::Publisher joint1_pub =nh.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command",10);
  ros::Publisher joint2_pub =nh.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command",10);

  ros::Rate loop_rate(10);//Set loop Freaquency of 10Hz

  //ROS variables
  int start_time,elapsed;

  while (not start_time)
  {
    start_time = ros::Time::now().toSec(); //Get Ros Start Time
  }

  while (ros::ok())
  {
    elapsed = ros::Time::now().toSec() - start_time; //Get the ROS elapsed Time 

    //set the arm joint angles 
    std_msgs::Float64 joint1_angle, joint2_angle;

    joint1_angle.data =sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
    joint2_angle.data =sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

    //publish the arm joint angle 
    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    //sleep in order to met 10Hz requarment
    loop_rate.sleep();
  }

  return 0;
}




