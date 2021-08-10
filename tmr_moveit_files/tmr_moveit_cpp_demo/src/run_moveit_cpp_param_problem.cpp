#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "pickplace_msgs/srv/ask_move_it.hpp"
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
#include <fstream>
#include <string>
#include <functional>
#include <string>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("pickplace_moveit_node");

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

class Pickplace_MoveIt_Node : public rclcpp::Node
{
  public:
    explicit Pickplace_MoveIt_Node(const std::string & node_name) : Node(node_name)
    {
      RCLCPP_INFO(this->get_logger(), "%s started", node_name.c_str());
      rclcpp::NodeOptions node_options;
      node_options.automatically_declare_parameters_from_overrides(true);
      rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, "", node_options);

      this->get_parameter("ompl", parameter_string_);
      RCLCPP_INFO(LOGGER, parameter_string_);


      server_ = this->create_service<pickplace_msgs::srv::AskMoveIt>("pickplace_moveit"
        , std::bind(&Pickplace_MoveIt_Node::handle_service, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node);
      moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
      moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);
      moveit_cpp::PlanningComponent arm("tmr_arm", moveit_cpp_);
      rclcpp::sleep_for(std::chrono::seconds(5));

      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = "base";
      collision_object.id = "box";

      shape_msgs::msg::SolidPrimitive box;
      box.type = box.BOX;
      box.dimensions = { 1.0, 1.0, 1.1 };

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

    void moveToPoint(geometry_msgs::msg::PoseStamped coordinates, moveit_cpp::MoveItCppPtr moveit_cpp_) {
      moveit_cpp::PlanningComponent arm("tmr_arm", moveit_cpp_);

      // Set joint state goal
      arm.setGoal(coordinates, "flange");

      // Run actual plan
      auto plan_solution = arm.plan();
      if (plan_solution)
      {
        arm.execute();
      }
      return;
    }

  private:
    void handle_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<pickplace_msgs::srv::AskMoveIt::Request> req,
      std::shared_ptr<pickplace_msgs::srv::AskMoveIt::Response> res)
    {
      (void)request_header;
      geometry_msgs::msg::PoseStamped coordinates;
      float temp[4];
      quaternion_from_euler(req->coordinates[3], req->coordinates[4], req->coordinates[5], temp);
      coordinates.pose.position.x = req->coordinates[0];
      coordinates.pose.position.y = req->coordinates[1];
      coordinates.pose.position.z = req->coordinates[2];
      coordinates.pose.orientation.w = temp[0];
      coordinates.pose.orientation.x = temp[1];
      coordinates.pose.orientation.y = temp[2];
      coordinates.pose.orientation.z = temp[3];
      coordinates.header.frame_id = "base";
      coordinates.header.stamp = this->now();
      moveToPoint(coordinates, moveit_cpp_);
      res->done = true;
    }

    rclcpp::Service<pickplace_msgs::srv::AskMoveIt>::SharedPtr server_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
    rclcpp::Node::SharedPtr node_;
    std::string parameter_string_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(std::chrono::seconds(5));
  auto node = std::make_shared<Pickplace_MoveIt_Node>("pickplace_moveit_node");
  RCLCPP_INFO(node->get_logger(), "TEST");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}