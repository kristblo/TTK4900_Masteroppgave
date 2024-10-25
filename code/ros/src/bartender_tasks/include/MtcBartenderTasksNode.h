#ifndef MTCBARTENDERTASKSNODE_H
#define MTCBARTENDERTASKSNODE_H

#include <cstdio>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif


namespace mtc = moveit::task_constructor;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("bartender_main");

class MtcBartenderTasksNode //: public rclcpp::Node
{
public:
  std::string objectName; //Object to manipulate
  
  MtcBartenderTasksNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setUpPlanningScene();
  void doPickTask(std::string objectName);
  void doPourTask(std::string objectName);
  void doPlaceTask();

private:
  std::string init_arg;
  mtc::Stage* attach_object_stage;

  mtc::Task createPickTask(std::string objectName);
  mtc::Task createPourTask(std::string objectName);
  mtc::Task createPlaceTask();
  mtc::Task pickTask_;
  mtc::Task pourTask_;
  mtc::Task placeTask_;

  rclcpp::Node::SharedPtr node_;

};


#endif //MTCBARTENDERTASKSNODE_H