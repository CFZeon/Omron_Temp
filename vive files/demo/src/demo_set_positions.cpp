#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/set_positions.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include "json.hpp"
#include <fstream>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
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

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_set_positions");
  rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr client =
    node->create_client<tm_msgs::srv::SetPositions>("set_positions");
  rclcpp::Rate r(20);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), std::to_string(co["coordinates"].size()));
  for (long unsigned int i=0; i< co["coordinates"].size(); i++) {
    auto request = std::make_shared<tm_msgs::srv::SetPositions::Request>();
    request->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
    request->positions.push_back(co["coordinates"][i][0]);
    request->positions.push_back(co["coordinates"][i][1]);
    request->positions.push_back(co["coordinates"][i][2]);
    request->positions.push_back(co["coordinates"][i][3]);
    request->positions.push_back(co["coordinates"][i][4]);
    request->positions.push_back(co["coordinates"][i][5]);
    request->velocity = 1;//rad/s
    request->acc_time = 0.2;
    request->blend_percentage = 40;
    request->fine_goal  = true;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    for (int i=0; i<6; i++)
      request->positions.pop_back();
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
      if(result.get()->ok){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"OK");
      } else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"not OK");
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
    r.sleep();
  }
  return true;
  rclcpp::shutdown();
  return 0;
}
