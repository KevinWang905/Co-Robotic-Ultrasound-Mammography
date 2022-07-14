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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_movement");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  tf::TransformListener listener;
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

  
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
  tf::StampedTransform transform_wm;


  while (node_handle.ok()){
    

    try{



      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(-0.0892, 0.071, 0.0578) );
      tf::Quaternion q(-0.0082, 0.112, -0.7103, 0.6949);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tool0", "stereo_gazebo_left_camera_optical_frame"));

      listener.lookupTransform("world", "aruco_marker_frame", ros::Time(0), transform_wm);

      
      target_pose1.orientation.x = -0.1026;
      target_pose1.orientation.y = 0.76332;
      target_pose1.orientation.z = 0.15701;
      target_pose1.orientation.w = 0.61819;
      target_pose1.position.x = transform_wm.getOrigin().x();
      target_pose1.position.y = transform_wm.getOrigin().y();
      target_pose1.position.z = transform_wm.getOrigin().z();
      
      


      break;
      
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.1);


  move_group.setPoseTarget(target_pose1);
  move_group.move();

  // //Rotation representing aruco marker orientation
  Eigen::Quaternionf arucomarkerorientation;
  arucomarkerorientation.x() = transform_wm.getRotation().x();
  arucomarkerorientation.y() = transform_wm.getRotation().y();
  arucomarkerorientation.z() = transform_wm.getRotation().z();
  arucomarkerorientation.w() = transform_wm.getRotation().w();
  Eigen::Matrix3f R_marker = arucomarkerorientation.toRotationMatrix();

  // Locate Point from Aruco Marker via Pivot Calibration
  Eigen::Matrix4f T_markertopoint;
  T_markertopoint << 1,0,0, 0.0066, 0,1,0, -0.1753, 0,0,1, -0.0072, 0,0,0,1;
  Eigen::Matrix4f G_arucomarker;
  G_arucomarker << R_marker.row(0), transform_wm.getOrigin().x(), R_marker.row(1), transform_wm.getOrigin().y(), R_marker.row(2), transform_wm.getOrigin().z(), 0,0,0,1;



  Eigen::Matrix4f pointlocation = G_arucomarker * T_markertopoint;


  // target_pose1.orientation.x = -0.1026;
  // target_pose1.orientation.y = 0.76332;
  // target_pose1.orientation.z = 0.15701;
  // target_pose1.orientation.w = 0.61819;
  target_pose1.orientation.x = transform_wm.getRotation().x();
  target_pose1.orientation.y = transform_wm.getRotation().y();
  target_pose1.orientation.z = -transform_wm.getRotation().z();
  target_pose1.orientation.w = transform_wm.getRotation().w();
  target_pose1.position.x = pointlocation.coeff(0, 3);
  target_pose1.position.y = pointlocation.coeff(1, 3);
  target_pose1.position.z = pointlocation.coeff(2, 3);


  std::cout << "Press Enter to move to point";
  std::cin.ignore();


  move_group.setMaxVelocityScalingFactor(0.03);
  move_group.setMaxAccelerationScalingFactor(0.1);
  
  move_group.setPoseTarget(target_pose1);
  move_group.move();


 
  std::cout << "Press Enter to return to start";
  std::cin.ignore();


  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.1);
  target_pose1.orientation.x = 0.238037;
  target_pose1.orientation.y = 0.668161;
  target_pose1.orientation.z = -0.24411;
  target_pose1.orientation.w = 0.661294;
  target_pose1.position.x = 0.230228;
  target_pose1.position.y =0.309703;
  target_pose1.position.z = 0.369194;
  
  move_group.setPoseTarget(target_pose1);
  move_group.move();
  
  return 0;
}