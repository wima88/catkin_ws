#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>


//Gloabal variables
//  ROS
ros::Publisher joint1_pub,joint2_pub;

/*
 * function description :functionfunction to check provided parameters are in safe zones
 * return val           :vector of floats (clamped_j1, clampedj2)
 * argumets             :float, requested angles for j1&j2 
 */

std::vector<float> clamp_at_bounderies(float requested_j1, float requested_j2)
{
  float clamped_j1 = requested_j1;
  float clamped_j2 = requested_j2;

  float min_j1, min_j2, max_j1, max_j2; // defined min and max parameters for the defined jpoints

  //node handler 
  ros::NodeHandle _n;

  //get the node name programetically
  std::string node_name = ros::this_node::getName();

  _n.getParam(node_name+"/min_joint_1_angle", min_j1);
  _n.getParam(node_name+"/max_joint_1_angle", max_j1);
  _n.getParam(node_name+"/min_joint_2_angle", min_j2);
  _n.getParam(node_name+"/max_joint_2_angle", max_j2);

  //logic to check for safe Zone, otherwise Clamp it
  if(requested_j1 < min_j1 || requested_j1 > max_j1)
  {
    clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
    ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
  }
  
  if (requested_j2 < min_j2 || requested_j2 > max_j2) 
  {
   clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
   ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2); //ros log
  }

  std::vector<float> claped_data = {clamped_j1, clamped_j2}; //stuffing joint angles to return 

  return claped_data;
}



/*
 * function description :Callback function for the service "safe move"
 * return val           :bool , true in sucsess
 * arguments            :request and response
 */

bool handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res)
{
  ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2); //ros log
  std::vector<float> joints_angles = clamp_at_bounderies(req.joint_1, req.joint_2); // check for safe_zone


  // Publish clamped joint angles to the arm
  
  std_msgs::Float64 joint1_angle, joint2_angle;
  
  joint1_angle.data = joints_angles[0];
  joint2_angle.data = joints_angles[1];
  
  joint1_pub.publish(joint1_angle);
  joint2_pub.publish(joint2_angle);                     

 // Wait 2 seconds for arm to settle
      ros::Duration(3).sleep();

      res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
      ROS_INFO_STREAM(res.msg_feedback);

      return true;
}

int main(int argc ,char**argv)
{
  //initialize the ros 
  ros::init(argc,argv,"arm_mover");
  ros::NodeHandle nh;
  
  // defined two publishers for joint1 and 2
  joint1_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
  joint2_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

  //ros service binding

  ros::ServiceServer service =nh.advertiseService("/arm_mover/safe_move",handle_safe_move_request);
  ROS_INFO ("Ready to sned service command"); //ros log

  ros::spin();

  return 0;
}


