#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned 

/*
 * function description :Callback function for the service
 * return val           :bool , true in sucsess
 * arguments            :request and response
 */
bool handle_mtr_command_rqst(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
  ROS_INFO("DriveToTarget request recieved linear_x: %1.2f , anguler_z: %1.2f",(float)req.linear_x ,(float)req.angular_z);

  //publish data to /cmd_vel
  geometry_msgs::Twist motor_command;
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z =req.angular_z;

  motor_command_publisher.publish(motor_command);

  //wait for 2 seconds 
  //ros::Duration(2).sleep();
  
  res.msg_feedback = "linear and angiler velocities are sucsessfully set";

  ROS_INFO_STREAM(res.msg_feedback);

  return true;

}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    // ros service binder
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot",handle_mtr_command_rqst);
    ROS_INFO("Ready to send SErvice command"); 


    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
