#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
  ball_chaser :: DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
   // ROS_INFO("Client : service requersted lin_x:%f , and_z:%f ",lin_x,ang_z);

    if(!client.call(srv))
    {
      ROS_ERROR("Fail to call service ");
    }

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    //local variables 
    int leftMargin = (int)img.width/3;
    int rightMargin= (int)(img.width/3)*2;
    int pix_location;
    bool ball_found ; 

    // itertate arra to find the matching pattern
    for(int i=0; i<img.data.size();i+=3)
    {
      ball_found = false;

      if((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255))
      {
        pix_location = ((i/3)%img.width);
        ball_found= true;
        
        break;
      }
    }
    
    if(ball_found)
    {
      if((pix_location <leftMargin)) 
      {
        //drive left
        drive_robot(0.25,0.5);
      }
    
      else if(pix_location>leftMargin && pix_location<rightMargin)
      {
        //drive straight
        drive_robot(0.5,0);
      }
      else if (pix_location>rightMargin && pix_location< img.data.size())
      {
        //drive left
        drive_robot(0.25,-0.5);
      }

    }

    if(!ball_found)
    {
      drive_robot(0,0);
    }
    

   
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ROS_INFO("Cleint : ros node initialized");

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
