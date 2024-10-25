//Library includes
#include <cstdio>
#include <vector>
#include <unistd.h>

//ROS/MoveIt/MTC includes
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

//Bartender includes
#include "MtcBartenderTasksNode.h"


namespace mtc = moveit::task_constructor;


/////////////////////////////////////////////////
//////////////////MAIN PROGRAM///////////////////
/////////////////////////////////////////////////

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);


  auto bartender_node_pick = std::make_shared<MtcBartenderTasksNode>(options);
  auto bartender_node_pour = std::make_shared<MtcBartenderTasksNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;


  auto spin_thread = std::make_unique<std::thread>([&executor, &bartender_node_pick, &bartender_node_pour](){
    executor.add_node(bartender_node_pick->getNodeBaseInterface());
    executor.add_node(bartender_node_pour->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(bartender_node_pick->getNodeBaseInterface());
    executor.remove_node(bartender_node_pour->getNodeBaseInterface());
  });
  bartender_node_pick->setUpPlanningScene();
  bartender_node_pick->doPickTask(bartender_node_pick->objectName);
  
  // unsigned int microsecond = 1000000;
  // usleep(1*microsecond);
  
  //bartender_node_pour->doPourTask(bartender_node_pick->objectName);


  spin_thread->join();
  rclcpp::shutdown();

  return 0;
}