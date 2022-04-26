

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <iostream>

#include <geometry_msgs/WrenchStamped.h>

//Initialize global Wrench
geometry_msgs::Wrench wr;

void ftCallback( const geometry_msgs::WrenchStamped& stamped_wr ) {

  wr = stamped_wr.wrench;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_movement");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  tf::TransformListener listener;
  ros::Subscriber ft_sub;
  ros::Rate rate(2000.0);

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Scale movement speed to 0.1
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.006;
  // target_pose1.position.y = 0.3;
  // target_pose1.position.z = 0.232;
  // move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  tf::StampedTransform transform_probe;
  tf::StampedTransform transform_wm;
  float tag_x = 0;
  float tag_y = 0;
  float tag_z = 0;

  while (node_handle.ok()){


    try{

      //Get WORLD to PROBE Transform
      listener.lookupTransform("world", "probe", ros::Time(0), transform_probe);

      //Broadcast TOOL to CAMERA Transform acquired from Hand-Eye Calibration
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(-0.0892, 0.071, 0.0578) );
      tf::Quaternion q(-0.0082, 0.112, -0.7103, 0.6949);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tool0", "stereo_gazebo_left_camera_optical_frame"));

      //Attempt to get WORLD to TAG transform
      listener.lookupTransform("world", "aruco_marker_frame", ros::Time(0), transform_wm);

      tag_x = transform_wm.getOrigin().x();
      tag_y = transform_wm.getOrigin().y();
      tag_z = transform_wm.getOrigin().z();

      //Print TAG Position with respect to WORLD
      std::cout << transform_wm.getOrigin().x() << std::endl;
      std::cout << transform_wm.getOrigin().y() << std::endl;
      std::cout << transform_wm.getOrigin().z() << std::endl;



      //Set first target as PROBE positioned vertically downward 5 cm above TAG
      target_pose1.orientation.x = -0.1026;
      target_pose1.orientation.y = 0.76332;
      target_pose1.orientation.z = 0.15701;
      target_pose1.orientation.w = 0.61819;
      target_pose1.position.x = tag_x;
      target_pose1.position.y = tag_y;
      target_pose1.position.z = tag_z + 0.05;


      break;
      
    }
    catch (tf::TransformException ex){
      //If TAG is not visible wait 1 second and try again
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }

  //Move PROBE to first target (above TAG)
  move_group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group.move();
  
  /*
  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  //We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  */

  //GET USER INPUT FOR REGION OF CONCERN LOCATIONS

  int region_num;
  std::cout << "Please enter the number of regions of concern: ";
  std::cin >> region_num;
  
  std::cout << std::endl << std::endl;
  
  float regions[2][region_num];
  
  for (int i = 0; i < region_num; i++) {
    std::cout << "Please enter x value for point " << (i+1) << ": ";
    std::cin >> regions[1][i];
    std::cout << std::endl;
    std::cout << "Please enter y value for point " << (i+1) << ": ";
    std::cin >> regions[2][i];
    std::cout << std::endl;
  }

  std::cout << std::endl << "Regions to be imaged are at: " << std::endl;
  for (int i = 0; i < region_num; i++) {
    std::cout << "(" << regions[1][i] << ", " << regions[2][i] << ")" << std::endl;
  }

  
  //Rotation representing PROBE directly downward
  Eigen::Quaterniond q;
  q.x() = -0.1026;
  q.y() = 0.76332;
  q.z() = 0.15701;
  q.w() = 0.61819;


  //Generate quaternion for 30 degree rotation about the y axis
  //(Lateral axis of the ultrasound probe)
  Eigen::Matrix3d rot1 = q.toRotationMatrix();
  Eigen::Matrix3d roty30;
  roty30 << 0.8660254, 0, 0.5, 0, 1, 0, -0.5, 0, 0.8660254;
  Eigen::Matrix3d wobble = rot1 * roty30;
  Eigen::Quaterniond wobble_q(wobble);
  wobble_q = wobble_q.normalized();

  //Generate quaternion for -30 degree rotation about the y axis
  //(Lateral axis of the ultrasound probe)
  Eigen::Matrix3d rotyneg30;
  rotyneg30 << 0.8660254, 0, -0.5, 0, 1, 0, 0.5, 0, 0.8660254;
  Eigen::Matrix3d wobble2 = rot1 * rotyneg30;
  Eigen::Quaterniond wobble2_q(wobble2);
  wobble2_q = wobble2_q.normalized();


  //Move PROBE to each region of concern and perform wobble

  for (int i = 0; i < region_num; i++) {

    //Move PROBE 5 cm above region of concern
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();
    target_pose1.orientation.w = q.w();
    target_pose1.position.x = tag_x + regions[1][i];
    target_pose1.position.y = tag_y + regions[2][i];
    target_pose1.position.z = tag_z + 0.05;

    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

    float z = tag_z + 0.05;
    while (wr.force.z > -1) {
      z -= 0.001;
      
      //Move PROBE to region of concern
      target_pose1.orientation.x = q.x();
      target_pose1.orientation.y = q.y();
      target_pose1.orientation.z = q.z();
      target_pose1.orientation.w = q.w();
      target_pose1.position.x = tag_x + regions[1][i];
      target_pose1.position.y = tag_y + regions[2][i];
      target_pose1.position.z = z;
      
      move_group.setPoseTarget(target_pose1);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
      move_group.move();
      
      ft_sub = node_handle.subscribe("robotiq_ft_wrench", 10, ftCallback);   
      
    }
    
    

    //Wobble 30 degrees around Lateral axis
    target_pose1.orientation.x = wobble_q.x();
    target_pose1.orientation.y = wobble_q.y();
    target_pose1.orientation.z = wobble_q.z();
    target_pose1.orientation.w = wobble_q.w();
    target_pose1.position.x = tag_x + regions[1][i];
    target_pose1.position.y = tag_y + regions[2][i];
    target_pose1.position.z = z;

    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

    //Return PROBE to vertical orientation
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();
    target_pose1.orientation.w = q.w();
    target_pose1.position.x = tag_x + regions[1][i];
    target_pose1.position.y = tag_y + regions[2][i];
    target_pose1.position.z = z;

    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

    //Wobble -30 degrees around Lateral axis
    target_pose1.orientation.x = wobble2_q.x();
    target_pose1.orientation.y = wobble2_q.y();
    target_pose1.orientation.z = wobble2_q.z();
    target_pose1.orientation.w = wobble2_q.w();
    target_pose1.position.x = tag_x + regions[1][i];
    target_pose1.position.y = tag_y + regions[2][i];
    target_pose1.position.z = z;

    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

    //Return PROBE to vertical orientation
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();
    target_pose1.orientation.w = q.w();
    target_pose1.position.x = tag_x + regions[1][i];
    target_pose1.position.y = tag_y + regions[2][i];
    target_pose1.position.z = z;

    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

    //Return PROBE 5 cm above region of concern
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();
    target_pose1.orientation.w = q.w();
    target_pose1.position.x = tag_x + regions[1][i];
    target_pose1.position.y = tag_y + regions[2][i];
    target_pose1.position.z = tag_z + 0.05;

    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
    
  }

  /*
  //Return Probe to original position
  target_pose1.orientation.x = transform_probe.getRotation().getAxis().getX();
  target_pose1.orientation.y = transform_probe.getRotation().getAxis().getY();
  target_pose1.orientation.z = transform_probe.getRotation().getAxis().getZ();
  target_pose1.orientation.w = transform_probe.getRotation().getW();
  target_pose1.position.x = transform_probe.getOrigin().x();
  target_pose1.position.y = transform_probe.getOrigin().y();
  target_pose1.position.z = transform_probe.getOrigin().z();
  
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group.move();
  */
  // END_TUTORIAL

  //ros::shutdown();
  return 0;
}
