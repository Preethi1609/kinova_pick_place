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

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "root";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.265849787727;
  collision_objects[0].primitive_poses[0].position.y = -0.212182493;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "root";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
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
  collision_objects[2].primitive_poses[0].position.x = 0.0;
  collision_objects[2].primitive_poses[0].position.y = 0.5;
  collision_objects[2].primitive_poses[0].position.z = 0.352;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group("arm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  arm_group.setPlanningTime(50.0);
  gripper_group.setPlanningTime(50.0);

  addCollisionObjects(planning_scene_interface);


  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  geometry_msgs::Pose eef_pose;
  eef_pose.orientation.x = -0.988008788921;
  eef_pose.orientation.y = -0.0275425709658;
  eef_pose.orientation.z = -0.00145792044579;
  eef_pose.orientation.w = 0.15192109315;
  eef_pose.position.x = 0.0;
  eef_pose.position.y = 0.5;
  eef_pose.position.z = 0.45;
  arm_group.setPoseTarget(eef_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  arm_group.move();
  ROS_INFO_NAMED("tutorial", "Moving towards the ball");
  ros::WallDuration(1.0).sleep();

  arm_group.attachObject("object");
  ROS_INFO_NAMED("tutorial", "Attaching the object");
  ros::WallDuration(1.0).sleep();

  gripper_group.setNamedTarget("Close");
  ROS_INFO_NAMED("tutorial", "Closing gripper");
  gripper_group.move();
  ros::WallDuration(1.0).sleep();

  //arm_group.setNamedTarget("Home");
  //ROS_INFO_NAMED("tutorial", "Moving to home position");
  //arm_group.move();
  //ros::WallDuration(1.0).sleep();

  ROS_INFO_NAMED("tutorial", "Moving towards the other table");
  geometry_msgs::Pose eef_pose1;
  eef_pose1.orientation.x = -0.988008788921;
  eef_pose1.orientation.y = -0.0275425709658;
  eef_pose1.orientation.z = -0.00145792044579;
  eef_pose1.orientation.w = 0.15192109315;
  eef_pose1.position.x = 0.265849787727;
  eef_pose1.position.y = -0.212182493;
  eef_pose1.position.z = 0.45;
  arm_group.setPoseTarget(eef_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  bool success1 = (arm_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  arm_group.move();
  ros::WallDuration(1.0).sleep();

  gripper_group.setNamedTarget("Open");
  ROS_INFO_NAMED("tutorial", "Opening gripper");
  gripper_group.move();
  ros::WallDuration(1.0).sleep();

  arm_group.detachObject("object");
  ROS_INFO_NAMED("tutorial", "Detaching the object");
  ros::WallDuration(1.0).sleep();

  arm_group.setNamedTarget("Vertical");
  ROS_INFO_NAMED("tutorial", "Moving to home position");
  arm_group.move();
  ros::WallDuration(1.0).sleep();

  
  
  
  ros::waitForShutdown();
  return 0;
}

