#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ServoSubscriber : public rclcpp::Node
{
  using FollowJointTrajectoryMsg = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>;

  public:
    ServoSubscriber() : Node("servo_subscriber")
    {
      // client to send to tm driver action server
      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "tmr_arm_controller/follow_joint_trajectory");
      // bool response = client_->wait_for_action_server(std::chrono::seconds(1));
      // if (!response) 
      // {
      //   throw std::runtime_error("could not get action server");
      // }

      // subscriber to servo, the topic name is dictated in a yaml file from servo initialization
      subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "tmr_arm/joint_trajectory", 10, std::bind(&ServoSubscriber::topic_callback, this, _1));
    }

  private:
    // topic callback for subscriber
    void topic_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const
    {
      // creating FollowJointTrajectory from JointTrajectory
      control_msgs::action::FollowJointTrajectory_Goal goal_msg;
      goal_msg.trajectory.points = msg->points;
      goal_msg.trajectory.joint_names = msg->joint_names;
      goal_msg.trajectory.header = msg->header;
      goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(5);

      // creating goal options
      auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
      // send_goal_options.goal_response_callback = std::bind(&ServoSubscriber::goal_response_callback, this, _1);
      // send_goal_options.feedback_callback = std::bind(&ServoSubscriber::goal_feedback_callback, this, _1, _2);
      // send_goal_options.result_callback = std::bind(&ServoSubscriber::goal_result_callback, this, _1);


      send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandle::SharedPtr> future)
        {
          const auto goal_handle = future.get();
          if (!goal_handle) {
            RCLCPP_DEBUG(this->get_logger(), "Goal rejected");
          }
          else {
            RCLCPP_DEBUG(this->get_logger(), "Goal accepted");
          }
        };
      send_goal_options.feedback_callback = [&](
        rclcpp_action::ClientGoalHandle<FollowJointTrajectoryMsg>::SharedPtr,
        const std::shared_ptr<const FollowJointTrajectoryMsg::Feedback>)
        {
          // Don't know what to put here, blank for now
        };
      send_goal_options.result_callback = [&](const GoalHandle::WrappedResult & result)
        {
          RCLCPP_DEBUG(
            this->get_logger(), "common_result_response time: %f",
            rclcpp::Clock().now().seconds());
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_DEBUG(this->get_logger(), "Goal was aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_DEBUG(this->get_logger(), "Goal was canceled");
              return;
            default:
              RCLCPP_DEBUG(this->get_logger(), "Unknown result code");
              return;
          }
        };

      // send goal to action server
      auto goal_handle_future = this->client_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    // void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future)
    // {
    //   RCLCPP_DEBUG(
    //     this->get_logger(), "common_goal_response time: %f", rclcpp::Clock().now().seconds());
    //   const auto goal_handle = future.get();
    //   return;
    // }

    // void goal_feedback_callback() 
    // {
    //   return;
    // }

    // void goal_result_callback(const GoalHandle::WrappedResult & result) 
    // {
    //   return;
    // }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoSubscriber>());
  rclcpp::shutdown();
  return 0;
}
