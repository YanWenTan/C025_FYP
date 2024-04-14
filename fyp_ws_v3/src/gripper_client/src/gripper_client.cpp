#include "rclcpp/rclcpp.hpp"
#include "robotiq_3f_gripper_ros2_interfaces/srv/robotiq3_f_gripper_output_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  using namespace std::chrono_literals;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gripper_client");
  rclcpp::Client<robotiq_3f_gripper_ros2_interfaces::srv::Robotiq3FGripperOutputService>::SharedPtr client =
    node->create_client<robotiq_3f_gripper_ros2_interfaces::srv::Robotiq3FGripperOutputService>("Robotiq3FGripper/OutputRegistersService");
    int gripper_pos = 0;
    auto request = std::make_shared<robotiq_3f_gripper_ros2_interfaces::srv::Robotiq3FGripperOutputService::Request>();
    request -> output_registers.r_act = 1;
    request -> output_registers.r_mod = 1;
    request -> output_registers.r_gto = 1;
    request -> output_registers.r_atr = 0;
    request -> output_registers.r_pra = gripper_pos; //Position of Gripper, 255: Fully Closed, 0: Fully Opened
    request -> output_registers.r_spa = 255;
    request -> output_registers.r_fra = 0;        

    while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper Movement Done");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
}