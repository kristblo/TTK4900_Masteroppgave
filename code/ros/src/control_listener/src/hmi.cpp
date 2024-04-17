#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "orca_ctrl_msgs/msg/ctrl.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class Hmi : public rclcpp::Node
{
  public:
  Hmi() : Node("hmi")
  {
    publisher_ = this->create_publisher<orca_ctrl_msgs::msg::Ctrl>("orca_ctrl", 10);

    RCLCPP_INFO(this->get_logger(), "Welcome to the ORCA human interface. For safe exit, type 'exit', then ctrl+c");
  }

  std::string lastInput;
  
  void parse_input()
  {
    std::string input;
    while(1)
    {
      std::cout << "ORCA_term: ";
      std::getline(std::cin, input);
      
      if(input == "exit")
      {
        RCLCPP_INFO(this->get_logger(), "Exiting HMI. Press ctrl+c to kill the ROS node");
        break;
      }
      
      

      auto message = orca_ctrl_msgs::msg::Ctrl();
      message.msgtype = "str";
      message.strcmd = input;

      RCLCPP_INFO(this->get_logger(), "Publishing cmd: '%s'", input.c_str());
      publisher_->publish(message);
    }
  }
  
  private:

  rclcpp::Publisher<orca_ctrl_msgs::msg::Ctrl>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  // std::string n;
  // while(1)
  // {
  //   std::cout << "ORCA terminal: ";
  //   std::cin >> n;
  //   std::cout << "Got n: " << n << "\n";
  // }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hmi>();
  node->parse_input();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}