#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinva_arm_iksolver_node");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Rate r(30);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  //Kinematic model required for getting Joint Model Group
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  //Joint Model Group is used to call IK solver
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  //Kinematic state also used in IK call
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  //Joint values used to print out IK solution when it is found
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  //Joint names used to print out IK solution when it is found
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  visualization_msgs::Marker points;
  points.header.frame_id = "root"; 
  points.header.stamp = ros::Time::now();
  points.ns ="points_and_lines";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;    

  points.scale.x = 0.02; // points
  points.scale.y = 0.02;
  points.scale.z = 0.02;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  double timeout = 0.1;
  bool found_ik;
  geometry_msgs::Pose eef_pose;
  eef_pose.orientation.x = -0.988008788921;
  eef_pose.orientation.y = -0.0275425709658;
  eef_pose.orientation.z = -0.00145792044579;
  eef_pose.orientation.w = 0.15192109315;
  eef_pose.orientation.w = 0.15192109315;
  eef_pose.position.x = 0.0;
  eef_pose.position.y = 0.5;
  eef_pose.position.z = 0.4;

  geometry_msgs::Point p = eef_pose.position;
  for(uint32_t ik_call=0; ik_call<1;++ik_call){
    found_ik = kinematic_state->setFromIK(joint_model_group, eef_pose, timeout);
    if(found_ik) break;
  }
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i){
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    points.points.push_back(p);
    marker_pub.publish(points);
    ROS_INFO("Published points");
  }
  else
  {
    ROS_INFO_STREAM("No solution found\n");
  } 
    
  ros::shutdown();
  return 0;
}
