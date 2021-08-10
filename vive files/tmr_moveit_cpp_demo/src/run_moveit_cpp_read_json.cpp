/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*
   Desc: A simple demo node running MoveItCpp for planning and execution
*/

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <cmath>
#include <iostream>
#include "json.hpp"
#include <fstream>
#include <string>
//#include "tm_msgs/srv/set_event.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

void quaternion_from_euler(float roll, float pitch, float yaw, float* result) {
  float cy, sy, cp, sp, cr, sr;
  cy = std::cos(yaw * 0.5);
  sy = std::sin(yaw * 0.5);
  cp = std::cos(pitch * 0.5);
  sp = std::sin(pitch * 0.5);
  cr = std::cos(roll * 0.5);
  sr = std::sin(roll * 0.5);
  result[0] = cy * cp * cr + sy * sp * sr; //w
  result[1] = cy * cp * sr - sy * sp * cr; //x
  result[2] = sy * cp * sr + cy * sp * cr; //y
  result[3] = sy * cp * cr - cy * sp * sr; //z
  return;
}

void moveToPoint(geometry_msgs::msg::PoseStamped coordinates, moveit_cpp::MoveItCppPtr moveit_cpp_) {
    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent arm("tmr_arm", moveit_cpp_);

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal PoseStamped");
    arm.setGoal(coordinates, "flange");

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to cartesian");
    auto plan_solution = arm.plan();
    if (plan_solution)
    {
      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
    }
    return;
}

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {
    // open coordinate file
    std::ifstream inFile;
    std::string coord;
    inFile.open("/home/zeon/vive/src/tmr_ros2/demo/src/positions.txt");
    if (!inFile) {
      std::cout << "File failed to read!";
      exit(1);
    }
    // using json = nlohmann::json;
    nlohmann::json co;
    inFile >> co;
    inFile.close();
    // std::cout << co["coordinates"][0].size();


    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent arm("tmr_arm", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base";
    collision_object.id = "box";



    // shape_msgs::msg::SolidPrimitive box;
    // box.type = box.BOX;
    // box.dimensions = { 1.0, 1.0, 1.1 };

    // geometry_msgs::msg::Pose box_pose;
    // box_pose.position.x = 0.0;
    // box_pose.position.y = 0.0;
    // box_pose.position.z = -0.5;

    // collision_object.primitives.push_back(box);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    // rclcpp::Client<tm_msgs::srv::SetEvent>::SharedPtr client =
    //   node_->create_client<tm_msgs::srv::SetEvent>("set_event");
    
    // auto request = std::make_shared<tm_msgs::srv::SetEvent::Request>();
    // request->func = tm_msgs::srv::SetEvent::Request::TAG;
    // request->arg0 = 3;
    // request->arg1 = 1;
    rclcpp::sleep_for(std::chrono::seconds(5));
    rclcpp::Rate r(5);
    for (long unsigned int i=0; i<co["coordinates"].size(); i++) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"timing");
      geometry_msgs::msg::PoseStamped test;

      float temp[4];
      quaternion_from_euler(co["coordinates"][i][3], co["coordinates"][i][4], co["coordinates"][i][5], temp);
      test.pose.position.x = co["coordinates"][i][0];
      test.pose.position.y = co["coordinates"][i][1];
      test.pose.position.z = co["coordinates"][i][2];
      test.pose.orientation.w = temp[0];
      test.pose.orientation.x = temp[1];
      test.pose.orientation.y = temp[2];
      test.pose.orientation.z = temp[3];
      test.header.frame_id = "base";
      test.header.stamp = node_->now();
      moveToPoint(test, moveit_cpp_);
      r.sleep();
    }
    
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
};


int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
