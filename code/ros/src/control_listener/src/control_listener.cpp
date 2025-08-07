#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "orca_ctrl_msgs/msg/ctrl.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CtrlListener : public rclcpp::Node
{
  public:
  CtrlListener() : Node("control_listener")
  {
    armSubscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "orca_arm_controller/controller_state", 10, std::bind(&CtrlListener::arm_ctrl_state_callback, this, _1));
    handSubscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "orca_hand_controller/controller_state", 10, std::bind(&CtrlListener::hand_ctrl_state_callback, this, _1));
    
    publisher_ = this->create_publisher<orca_ctrl_msgs::msg::Ctrl>("orca_ctrl", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&CtrlListener::timer_callback, this));    

  }

  private:
  void arm_ctrl_state_callback(const control_msgs::msg::JointTrajectoryControllerState &msg) //const
  {
    //RCLCPP_INFO(this->get_logger(), "Got state: '%fl'", msg.reference.positions[0]);
    //this->positions = msg.reference.positions[0];
    for(int i = 0; i < 5; i++)
    {
      this->positions[i] = msg.reference.positions[i];
    }
  }

  void hand_ctrl_state_callback(const control_msgs::msg::JointTrajectoryControllerState &msg) //const
  {
    //RCLCPP_INFO(this->get_logger(), "Got state: '%fl'", msg.reference.positions[0]);
    //this->positions = msg.reference.positions[0];
    this->positions[5] = msg.reference.positions[0];
  }

  void timer_callback()
  {
    auto message = orca_ctrl_msgs::msg::Ctrl();
    message.msgtype = "num"; //Numerical setpoints
    message.strcmd = "";//Not an "str" type message -> no string command
    //message.positions = this->positions;

    //For some C++ type related reason can't assign the whole vector,
    //and even the below for loop fails, so message was rewritten with
    //explicit joint positions
    // for(int i = 0; i < 5; i++)
    // {
    //   message.positions[i] = this->positions[i];
    // }
    message.pos_rail      = this->positions[0];
    message.pos_shoulder  = this->positions[1];
    message.pos_elbow     = this->positions[2];
    message.pos_wrist     = this->positions[3];
    message.pos_twist     = this->positions[4];
    message.pos_halfpinch = (this->positions[5])*2; //Dumbassery due to bad pinch definition. RViz outputs half value
    
    RCLCPP_INFO(this->get_logger(), "Publishing message: '%s', %f, %f, %f, %f", 
      message.msgtype.c_str(), message.pos_rail, message.pos_shoulder, message.pos_twist, message.pos_halfpinch);
    publisher_->publish(message);
  }

  double positions[6];

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<orca_ctrl_msgs::msg::Ctrl>::SharedPtr publisher_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr armSubscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr handSubscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CtrlListener>());
  rclcpp::shutdown();
  return 0;
}