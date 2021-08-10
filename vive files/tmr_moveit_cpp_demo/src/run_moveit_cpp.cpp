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
#include <moveit_servo/servo.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <cmath>
#include <iostream>
#include "json.hpp"
#include <fstream>
#include <string>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
//#include "tm_msgs/srv/set_event.hpp"

using namespace std::chrono_literals;

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
    , joint_cmd_pub_(node_->create_publisher<control_msgs::msg::JointJog>("servo_demo_node/delta_joint_cmds", 10))
    , twist_cmd_pub_(node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_demo_node/delta_twist_cmds", 10))
  {
  }

  void publishCommands()
  {
    if (count_ < 100)
    {
      auto msg = std::make_unique<control_msgs::msg::JointJog>();
      msg->header.stamp = node_->now();
      msg->joint_names.push_back("panda_joint1");
      msg->velocities.push_back(0.3);
      joint_cmd_pub_->publish(std::move(msg));
      ++count_;
    }
    else
    {
      auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      msg->header.stamp = node_->now();
      msg->header.frame_id = "panda_link0";
      msg->twist.linear.x = 0.3;
      msg->twist.angular.z = 0.5;
      twist_cmd_pub_->publish(std::move(msg));
    }
    return;
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);
    moveit_cpp_->getPlanningSceneMonitor()->startSceneMonitor();

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent arm("tmr_arm", moveit_cpp_);
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub;

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    if (moveit_cpp_->getPlanningSceneMonitor()->getPlanningScene())
    {
      moveit_cpp_->getPlanningSceneMonitor()->startStateMonitor("/joint_states");
      moveit_cpp_->getPlanningSceneMonitor()->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "/moveit_servo/publish_planning_scene");
    }

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_, LOGGER);
    if (!servo_parameters)
    {
      RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
      return;
    }
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, moveit_cpp_->getPlanningSceneMonitor());
    servo->start();
    rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, [=](){
      if (count_ < 100)
      {
        auto msg = std::make_unique<control_msgs::msg::JointJog>();
        msg->header.stamp = node_->now();
        msg->joint_names.push_back("link_1");
        msg->velocities.push_back(0.3);
        joint_cmd_pub_->publish(std::move(msg));
        ++count_;
      }
      else
      {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = "link_0";
        msg->twist.linear.x = 0.3;
        msg->twist.angular.z = 0.5;
        twist_cmd_pub_->publish(std::move(msg));
      }
    });

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.5, 0.5, 1.0 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.5;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene
    
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
  size_t count_ = 0;
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
  node_options.use_intra_process_comms(true);
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
