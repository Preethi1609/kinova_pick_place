/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Major portions of this code have been taken from MoveIt! tutorials: https://github.com/ros-planning/moveit_tutorials/tree/melodic-devel

/* Author: Ioan Sucan, Ridhwan Luthra, */
/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
/* Author: Preethi N */

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

double table1_z = 0.05;
double table2_z = 0.05;
double obj_z = table1_z + 0.1;

/* ##########################################PICK LOCATIONS(UNCOMMENT ANY ONE)###################################################*/
//Pick location 1 
// double table1_x = 0.0;
// double table1_y = 0.4;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 1;

//Pick location 2  
// double table1_x = -0.3;
// double table1_y = 0.0;
// double pre_grasp_z = 0.5;
// double to_grasp = 0.3;
// uint32_t pick_location = 2;

//Pick location 3 
// double table1_x = 0;
// double table1_y = 0.5;
// double pre_grasp_z = 0.4;
// double to_grasp = 0.2;
// uint32_t pick_location = 3;

//Pick location 4
double table1_x = 0.265849787727;
double table1_y = -0.212182493;
double pre_grasp_z = 0.4;
double to_grasp = 0.2;
uint32_t pick_location = 4;

//Pick location 5
// double table1_x = 0.25;
// double table1_y = 0.25;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 5;

//Pick location 6
// double table1_x = -0.25;
// double table1_y = 0.25;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 6;

//Pick location 7
// double table1_x = 0;
// double table1_y = 0.3;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 7;

//Pick location 8 works
// double table1_x = -0.15;
// double table1_y = 0.25;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 8;

//Pick location 9
// double table1_x = 0;
// double table1_y = 0.2;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 9;

//Pick location 10
// double table1_x = -0.25;
// double table1_y = 0.3;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 10;

//Pick location 11
// double table1_x = -0.3;
// double table1_y = 0.3;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 11;

//Pick location 12 NO MOTION PLAN FOUND... FOR ERROR HANDLING
// double table1_x = 0.25;
// double table1_y = 0;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t pick_location = 12;

//Pick location 13 ERROR HANDLING
// double table1_x = -0.3;
// double table1_y = 0.3;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.5;
// uint32_t pick_location = 13;

/* #############################PLACE LOCATIONS(UNCOMMENT ANY ONE, SHOULD BE DIFFERENT FROM PICK LOCATION)###########################*/
//Place location 1
// double table2_x = 0.0;
// double table2_y = 0.4;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 1;

//Place location 2
// double table2_x = -0.3;
// double table2_y = 0.0;
// double pre_place_z = 0.5;
// double to_place = 0.3;
// uint32_t place_location = 2;

//Place location 3
// double table2_x = 0;
// double table2_y = 0.5;
// double pre_place_z = 0.4;
// double to_place = 0.2;
// uint32_t place_location = 3;

//Place location 4
// double table2_x = 0.265849787727;
// double table2_y = -0.212182493;
// double pre_place_z = 0.4;
// double to_place = 0.2;
// uint32_t place_location = 4;

//Place location 5 
double table2_x = 0.25;
double table2_y = 0.25;
double pre_place_z = 0.6;
double to_place = 0.4;
uint32_t place_location = 5;

//Place location 6
// double table2_x = -0.25;
// double table2_y = 0.25;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 6;

//Place location 7
// double table2_x = 0;
// double table2_y = 0.3;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 7;

//Place location 8 
// double table2_x = -0.15;
// double table2_y = 0.25;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 8;

//Place location 9
// double table2_x = 0;
// double table2_y = 0.2;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 9;

//Place location 10
// double table2_x = -0.25;
// double table2_y = 0.3;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 10;

//Place location 11
// double table1_x = -0.3;
// double table1_y = 0.3;
// double pre_grasp_z = 0.6;
// double to_grasp = 0.4;
// uint32_t place_location = 11;

//Place location 12 NO MOTION PLAN FOUND... FOR ERROR HANDLING
// double table2_x = 0.25;
// double table2_y = 0;
// double pre_place_z = 0.6;
// double to_place = 0.4;
// uint32_t place_location = 12;

//Place location 13 ERROR HANDLING
// double table2_x = -0.3;
// double table2_y = 0.3;
// double pre_place_z = 0.6;
// double to_place = 0.5;
// uint32_t place_location = 13;



  /* This function is used to create the environment, it adds three collision objects: 
    Two tables and an object(sphere) that is placed on one of the tables. */
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);
  // Add the first table where the sphere will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "root";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = table1_x;
  collision_objects[0].primitive_poses[0].position.y = table1_y;
  collision_objects[0].primitive_poses[0].position.z = table1_z;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "root";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.2;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.1;

  //collision_objects[1].primitives[0].dimensions[2] = 0.3;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = table2_x;
  collision_objects[1].primitive_poses[0].position.y = table2_y;
  collision_objects[1].primitive_poses[0].position.z = table2_z;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "root";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
  collision_objects[2].primitives[0].dimensions.resize(1);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = table1_x;
  collision_objects[2].primitive_poses[0].position.y = table1_y;
  collision_objects[2].primitive_poses[0].position.z = obj_z;

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
  ROS_INFO_STREAM("Adding Collision Objects");
}

 /* This function is used move the end effector of the arm over the table
    which has the object over it. */

geometry_msgs::Pose moveToPreGraspPose(moveit::planning_interface::MoveGroupInterface& arm_group){

  ROS_INFO_STREAM("Moving towards pre-grasp pose");
  geometry_msgs::Pose eef_pose_pre_grasp;
  eef_pose_pre_grasp.orientation.x = -0.988008788921;
  eef_pose_pre_grasp.orientation.y = -0.0275425709658;
  eef_pose_pre_grasp.orientation.z = -0.00145792044579;
  eef_pose_pre_grasp.orientation.w = 0.15192109315;
  eef_pose_pre_grasp.position.x = table1_x;
  eef_pose_pre_grasp.position.y = table1_y;
  eef_pose_pre_grasp.position.z = pre_grasp_z;

  arm_group.setPoseTarget(eef_pose_pre_grasp);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_pre_grasp;
  bool success = (arm_group.plan(my_plan_pre_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success == 0){
    ROS_INFO_STREAM("No motion plan found to pre-grasp pose");
    ROS_INFO_STREAM("Shutting down...");

    ros::shutdown();
  }
  arm_group.execute(my_plan_pre_grasp);
  //arm_group.move();
  return eef_pose_pre_grasp;

}

 /* This function is used move the end effector of the arm 
  closer to the grasp position. */

void graspPose(moveit::planning_interface::MoveGroupInterface& arm_group, geometry_msgs::Pose& eef_pose_pre_grasp){

  ROS_INFO_STREAM("Moving towards the ball");
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(eef_pose_pre_grasp);
  geometry_msgs::Pose eef_pose_grasp = eef_pose_pre_grasp;
  eef_pose_grasp.position.z -= to_grasp;                            //Moving down 

  waypoints.push_back(eef_pose_grasp);
  //arm_group.setPoseTarget(eef_pose_grasp);
  arm_group.setMaxVelocityScalingFactor(0.1); //Cartesian motions needs to be slower 

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "kinova_arm_end_effector";
  ocm.header.frame_id = "root";
  ocm.orientation.x = -0.988008788921;
  ocm.orientation.y = -0.0275425709658;
  ocm.orientation.z = -0.00145792044579;
  ocm.orientation.w = 0.15192109315;
  //ocm.orientation.w = 1.0;

  ocm.absolute_x_axis_tolerance = 0.00001;
  ocm.absolute_y_axis_tolerance = 0.00001;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);

  moveit_msgs::RobotTrajectory trajectory;

  trajectory_msgs::JointTrajectory  joint_trajectory;

  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<float> positions;
  std::vector<float> joint_distances;
  joint_distances.resize(7);

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  const bool avoid_collisions=true;
  

  // Uncomment for Cartesian Path without constraints
  //double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

  //Cartesian path with constraints
  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, test_constraints, avoid_collisions);

  //Uncomment to print the whole joint trajectory
  //ROS_INFO_STREAM("The whole joint trajectory : "<< trajectory.joint_trajectory << std::endl);

  for(std::size_t j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j){

    //ROS_INFO_STREAM("Joint "<<j+1<<" cartesian point 0"<<" : "<<trajectory.joint_trajectory.points[0].positions[j]<<"\n");

    for (std::size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i){
      
      joint_distances[j] += abs(trajectory.joint_trajectory.points[i].positions[j]-trajectory.joint_trajectory.points[i-1].positions[j]);
      // ROS_INFO_STREAM("Joint "<<j+1<<" cartesian point "<<i<<" : "<<trajectory.joint_trajectory.points[i].positions[j]<<"\n");
      // ROS_INFO_STREAM("Distance travelled at this stage "<<joint_distances[j]<<"\n");

    }

    ROS_INFO_STREAM("Distance travelled by "<<trajectory.joint_trajectory.joint_names[j]<<" is "<<joint_distances[j]);
  }

  ROS_INFO("Visualizing plan to pick (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  if(fraction < 1.0){
    ROS_INFO_STREAM("No Cartesian path found to grasp pose");
    ROS_INFO_STREAM("Shutting down...");

    ros::shutdown();
  }
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_grasp;
  my_plan_grasp.trajectory_ = trajectory;
  //arm_group.move();
  arm_group.execute(my_plan_grasp);
  ROS_INFO_STREAM("Executing Cartesian path for pick");

}

/* This function is used to close the gripper. "Close" is a named target which was set using moveit setup assistant */

void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group){  
  gripper_group.setNamedTarget("Close");
  ROS_INFO_STREAM("Closing gripper");
  gripper_group.move();

}

/* This function is used to attach an object to the arm, by default it attaches it to the end-effector link */

void attachingObject(moveit::planning_interface::MoveGroupInterface& arm_group){
  arm_group.attachObject("object");
  ROS_INFO_STREAM("Attaching the object");

}
/* This function is used move the end effector of the arm above the second table
    on which the object has to be placed. */

geometry_msgs::Pose moveToPrePlacePose(moveit::planning_interface::MoveGroupInterface& arm_group){
  ROS_INFO_STREAM("Moving towards the other table");
  geometry_msgs::Pose eef_pose_pre_place;
  eef_pose_pre_place.orientation.x = -0.988008788921;
  eef_pose_pre_place.orientation.y = -0.0275425709658;
  eef_pose_pre_place.orientation.z = -0.00145792044579;
  eef_pose_pre_place.orientation.w = 0.15192109315;
  eef_pose_pre_place.position.x = table2_x;
  eef_pose_pre_place.position.y = table2_y;
  eef_pose_pre_place.position.z = pre_place_z;

  arm_group.setMaxVelocityScalingFactor(1.0);
  arm_group.setPoseTarget(eef_pose_pre_place);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_pre_place;
  bool success = (arm_group.plan(my_plan_pre_place) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success == 0){
    ROS_INFO_STREAM("No motion plan found to pre-place pose");
    ROS_INFO_STREAM("Shutting down...");

    ros::shutdown();
  }
  arm_group.execute(my_plan_pre_place);
  //arm_group.move();
  return eef_pose_pre_place;

}

/* This function is used move the end effector of the arm 
  closer to the grasp position. */
void placePose(moveit::planning_interface::MoveGroupInterface& arm_group, geometry_msgs::Pose& eef_pose_pre_place){
  ROS_INFO_STREAM("Moving towards place pose");
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(eef_pose_pre_place);
  geometry_msgs::Pose eef_pose_place = eef_pose_pre_place;
  eef_pose_place.position.z -= to_place;                            //Moving down 

  waypoints.push_back(eef_pose_place);
  arm_group.setMaxVelocityScalingFactor(0.1); //Cartesian motions needs to be slower 

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "kinova_arm_end_effector";
  ocm.header.frame_id = "root";
  ocm.orientation.x = -0.988008788921;
  ocm.orientation.y = -0.0275425709658;
  ocm.orientation.z = -0.00145792044579;
  ocm.orientation.w = 0.15192109315;
  ocm.absolute_x_axis_tolerance = 0.00001;
  ocm.absolute_y_axis_tolerance = 0.00001;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);

  moveit_msgs::RobotTrajectory trajectory;

  trajectory_msgs::JointTrajectory  joint_trajectory;

  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<float> positions;
  std::vector<float> joint_distances;
  joint_distances.resize(7);

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  const bool avoid_collisions=true;

  // Uncomment for Cartesian Path without constraints
  //double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

  //Cartesian path with constraints
  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, test_constraints, avoid_collisions);
  
  //Uncomment to print the whole joint trajectory
  //ROS_INFO_STREAM("The whole joint trajectory : "<< trajectory.joint_trajectory << std::endl);

  for(std::size_t j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j){
    
    //ROS_INFO_STREAM("Joint "<<j+1<<" cartesian point 0"<<" : "<<trajectory.joint_trajectory.points[0].positions[j]<<"\n");

    for (std::size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i){

      joint_distances[j] += abs(trajectory.joint_trajectory.points[i].positions[j]-trajectory.joint_trajectory.points[i-1].positions[j]);
      // ROS_INFO_STREAM("Joint "<<j+1<<" cartesian point "<<i<<" : "<<trajectory.joint_trajectory.points[i].positions[j]<<"\n");
      // ROS_INFO_STREAM("Distance travelled at this stage "<<joint_distances[j]<<"\n");

    }

    ROS_INFO_STREAM("Distance travelled by "<<trajectory.joint_trajectory.joint_names[j]<<" is "<<joint_distances[j]);
  }


  ROS_INFO("Visualizing plan to place (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  if(fraction < 1.0){
    ROS_INFO_STREAM("No Cartesian path found to grasp pose");
    ROS_INFO_STREAM("Shutting down...");

    ros::shutdown();
  }
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_place;
  my_plan_place.trajectory_ = trajectory;
  //arm_group.move();
  arm_group.execute(my_plan_place);

  ROS_INFO_STREAM("Executing Cartesian path for place");
}

/* This function is used to detach the object from the arm. */

void detachingObject(moveit::planning_interface::MoveGroupInterface& arm_group){
  arm_group.detachObject("object");
  ROS_INFO_STREAM("Detaching the object");

}

/* This function is used to open the gripper. "Open" is a named target which was set using
 Moveit Setup Assistant */

void openGripper(moveit::planning_interface::MoveGroupInterface& gripper_group){
  gripper_group.setNamedTarget("Open");
  ROS_INFO_STREAM("Opening gripper");
  gripper_group.move();

}

/*After placing the object the arm goes back to the initial position which is named "Vertical", set using 
  Moveit Setup Assistant. */

void moveToInitialPose(moveit::planning_interface::MoveGroupInterface& arm_group){
  arm_group.setMaxVelocityScalingFactor(1.0);
  arm_group.setNamedTarget("Vertical");
  ROS_INFO_STREAM("Moving to home position");
  arm_group.move();

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //The PlanningSceneInterface class is used to add collision object to our
  //environment, (here) 2 tables and a spherical object.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO_STREAM("Picking from pick position "<<pick_location<<" and placing to place position "<<place_location);

  //The MoveGroupInterface class is set up for the arm as well as gripper planning groups
  moveit::planning_interface::MoveGroupInterface arm_group("arm");
  
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

  ROS_INFO_STREAM("Planner id arm:"<<arm_group.getDefaultPlannerId("arm"));

  ROS_INFO_STREAM("Planner id gripper:"<<arm_group.getDefaultPlannerId("gripper"));

  //arm_group.setPlannerId("PRM");
  //gripper_group.setPlannerId("RRTstar");

  addCollisionObjects(planning_scene_interface);

  geometry_msgs::Pose eef_pose_pre_grasp = moveToPreGraspPose(arm_group);

  graspPose(arm_group, eef_pose_pre_grasp);
  
  closeGripper(gripper_group);

  attachingObject(arm_group);

  geometry_msgs::Pose eef_pose_pre_place = moveToPrePlacePose(arm_group);

  placePose(arm_group, eef_pose_pre_place);

  detachingObject(arm_group);

  openGripper(gripper_group);

  moveToInitialPose(arm_group);
  
  ros::shutdown();
  return 0;
}

