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
#include <cmath>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double pi=M_PI;

void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
geometry_msgs::Pose &target_pose) {
  // .. _move_group_interface-planning-to-pose-goal:

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);

}
// for gripper handling
void  controll_gripper 
(moveit::planning_interface::MoveGroupInterface &move_group_interface,double value)

{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;

  joint_group_positions=move_group_interface.getCurrentJointValues();
  joint_group_positions[0]=0;
  joint_group_positions[1]=0;
  joint_group_positions[2]=value;
  joint_group_positions[3]=0;
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  move_group_interface.setJointValueTarget(joint_group_positions);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan);
}



geometry_msgs::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw) {

geometry_msgs::Pose target_pose;
tf2::Quaternion orientation;
orientation.setRPY(roll,pitch, yaw);
target_pose.orientation= tf2::toMsg(orientation);
target_pose.position.x=x;
target_pose.position.y=y;
target_pose.position.z=z;

return target_pose;
}


void pick_cube_1(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.3, 0, 0.18, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.3, -0.1, 0.18, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.3, -0.1, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.36);
}





void place_cube_1(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{



geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.3, 0, 0.18, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.18, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.8);
}



void pick_cube_2(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.3, 0, 0.18, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.5, 0, 0.18, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.5, 0, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.36);
}


void place_cube_2(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{



geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.5, 0, 0.18, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.18, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0, 0.09, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.8);
}


void pick_cube_3(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2,target_pose3,target_pose4;
target_pose1=list_to_pose(0.3, 0, 0.25, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.25, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0.1, 0.1, -pi, 0, -pi/2);
target_pose4=list_to_pose(0.4, 0.1, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
waypoints.push_back(target_pose4);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.36);
}

void place_cube_3(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{



geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.4, 0.1, 0.2, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.2, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0, 0.14, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.8);
}


void pick_cube_4(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2,target_pose3,target_pose4,target_pose5;
target_pose1=list_to_pose(0.3, 0, 0.28, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.28, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, -0.2, 0.1, -pi, 0, -pi/2);
target_pose4=list_to_pose(0.4, -0.1, 0.1, -pi, 0, -pi/2);
target_pose5=list_to_pose(0.4, -0.1, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
waypoints.push_back(target_pose4);
waypoints.push_back(target_pose5);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.36);
}

void place_cube_4(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{



geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.4, -0.1, 0.28, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.28, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0, 0.19, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.8);
}


void pick_cube_5(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2;
target_pose1=list_to_pose(0.3, 0, 0.28, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.3, 0, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.36);
}

void place_cube_5(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{



geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.3, 0, 0.28, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.28, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0, 0.24, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.8);
}


void pick_cube_6(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.3, 0, 0.28, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.3, 0.1, 0.28, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.3, 0.1, 0.04, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.36);
}

void place_cube_6(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{



geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.3, 0.1, 0.3, -pi, 0, -pi/2);
target_pose2=list_to_pose(0.4, 0, 0.3, -pi, 0, -pi/2);
target_pose3=list_to_pose(0.4, 0, 0.29, -pi, 0, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);
 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
controll_gripper(gripper,0.8);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(45.0);

  moveit::core::RobotStatePtr initial_pose = arm.getCurrentState();
  const std::string & planning_frame=arm.getPlanningFrame();
  


  pick_cube_1(arm,gripper);
  ros::WallDuration(1.0).sleep();
  place_cube_1(arm,gripper);
  ros::WallDuration(1.0).sleep();
  pick_cube_2(arm,gripper);
  ros::WallDuration(1.0).sleep();
  place_cube_2(arm,gripper);
  ros::WallDuration(1.0).sleep();
  pick_cube_3(arm,gripper);
  ros::WallDuration(1.0).sleep();
  place_cube_3(arm,gripper);
  ros::WallDuration(1.0).sleep();
  pick_cube_4(arm,gripper);
  ros::WallDuration(1.0).sleep();
  place_cube_4(arm,gripper);
  ros::WallDuration(1.0).sleep();
  pick_cube_5(arm,gripper);
  ros::WallDuration(1.0).sleep();
  place_cube_5(arm,gripper);
  ros::WallDuration(1.0).sleep();
  pick_cube_6(arm,gripper);
  ros::WallDuration(1.0).sleep();
  place_cube_6(arm,gripper);
  ros::WallDuration(1.0).sleep();
  return 0;
}

