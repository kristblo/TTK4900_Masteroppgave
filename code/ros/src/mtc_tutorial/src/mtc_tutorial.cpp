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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

  void setupPlanningSceneMultiObj(); //Test creating multiple objects in an array

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.15, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.35;
  pose.position.z = 0.2;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}


void MTCTaskNode::setupPlanningSceneMultiObj()
{
  std::vector<moveit_msgs::msg::CollisionObject> collisionObjects;
  collisionObjects.resize(5);

  for(int i = 0; i < (int)collisionObjects.size(); i++)
  {
    char strBuf[8];
    sprintf(strBuf, "bottle%i", i);
    collisionObjects[i].id = strBuf;
    printf("Object %i name: %s\n", i, collisionObjects[i].id.c_str());

    collisionObjects[i].header.frame_id = "world";

    collisionObjects[i].primitives.resize(1);
    collisionObjects[i].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    collisionObjects[i].primitives[0].dimensions = {0.15, 0.02}; //h,r

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5 - (i * 0.1);
    pose.position.y = -0.35;
    pose.position.z = 0.2;
    pose.orientation.w = 1.0;
    collisionObjects[i].pose = pose;

    collisionObjects[i].operation = collisionObjects[i].ADD;
  }

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(collisionObjects);

}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "orca_arm";
  const auto& hand_group_name = "orca_hand";
  const auto& hand_frame = "link_twist";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "gripper");
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  
  //First stage: open the hand
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  //Test, just for fun. Works immediately after open hand
  // auto stage_arm_ready =
  //     std::make_unique<mtc::stages::MoveTo>("arm ready", interpolation_planner);
  // stage_arm_ready->setGroup(arm_group_name);
  // stage_arm_ready->setGoal("ready");
  // task.add(std::move(stage_arm_ready));


  //Second stage: move to pick
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});

  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  //Stage object
  mtc::Stage* attach_object_stage = nullptr; //Forward attach_object_stage to place pose generator

{
  auto grasp = std::make_unique<mtc::SerialContainer>("pick bottle0");
  task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach bottle0", cartesian_planner);
    stage->properties().set("marker_ns", "approach bottle0");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.15);

    //Set hand forward direction (that going to work wrt only position control enabled?)
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    //vec.vector.z = 1.0;
    vec.vector.y = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  {
    //Sample grasp pose (probably will need changing for ORCA)
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose("open");
    stage->setObject("bottle0");
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr); //Hook into current stage

    Eigen::Isometry3d grasp_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    grasp_frame_transform.linear() = q.matrix();
    grasp_frame_transform.translation().z() = 0.07;

    //Compute IK
    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    grasp->insert(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,bottle0)");
    stage->allowCollisions("bottle0",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("closed");
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach bottle0");
    stage->attachObject("bottle0", hand_frame);
    attach_object_stage = stage.get();
    grasp->insert(std::move(stage));
  }


  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift bottle0", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");

    // Set upward direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));

}


//Attempt to rotate bottle0 (relative to its top) before placing
{
  auto stage_move_to_rotate = std::make_unique<mtc::stages::Connect>(
      "move to rotate",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
  stage_move_to_rotate->setTimeout(2.0);
  stage_move_to_rotate->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_rotate));
}


{
  auto rotate = std::make_unique<mtc::SerialContainer>("rotate bottle0");
  task.properties().exposeTo(rotate->properties(), {"eef", "group", "ik_frame"});
  rotate->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});


  {
    // Sample place pose waypoint
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate waypoint pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "waypoint_pose");
    stage->setObject("bottle0");

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "bottle0";
    target_pose_msg.pose.position.z = 0.20;
    target_pose_msg.pose.position.x = -0.3;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("waypoint pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame("bottle0");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    rotate->insert(std::move(wrapper));
  }

  {
    auto stage_connect_move_flip = std::make_unique<mtc::stages::Connect>(
        "connect movement and flip",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                  /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
    stage_connect_move_flip->setTimeout(2.0);
    stage_connect_move_flip->properties().configureInitFrom(mtc::Stage::PARENT);
    rotate->insert(std::move(stage_connect_move_flip));
  }

  {
    // The actual bottle flip
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate flip pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "flip_pose");
    stage->setObject("bottle0");

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "bottle0";
    target_pose_msg.pose.position.z = 0.28; //offset with half the bottle height
    target_pose_msg.pose.position.x = -0.3;    
    //target_pose_msg.pose.orientation.y = -0.8;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    Eigen::Isometry3d flip_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI*3/4, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    flip_frame_transform.linear() = q.matrix();
    flip_frame_transform.translation().z() = 0.08;


    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("flip pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(flip_frame_transform, "bottle0");
    //wrapper->setIKFrame(flip_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    rotate->insert(std::move(wrapper));
  }

  {
    auto stage_connect_flip_unflip = std::make_unique<mtc::stages::Connect>(
        "connect flip and unflip",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                  /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
    stage_connect_flip_unflip->setTimeout(2.0);
    stage_connect_flip_unflip->properties().configureInitFrom(mtc::Stage::PARENT);
    rotate->insert(std::move(stage_connect_flip_unflip));
  }

  {
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate unflip bottle pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns","unflip_pose");
    stage->setObject("bottle0");

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "bottle0";
    target_pose_msg.pose.position.z = 0.20;
    target_pose_msg.pose.position.x = -0.3;
    target_pose_msg.pose.orientation.y = 0;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("unflip pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame("bottle0");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    rotate->insert(std::move(wrapper));    

  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat_from_pour", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat_from_pour");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.x = 0.2;
    stage->setDirection(vec);
    rotate->insert(std::move(stage));
    //task.add(std::move(rotate));
  
  }

  task.add(std::move(rotate));


}

//Place stage
{
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
}

{
  auto place = std::make_unique<mtc::SerialContainer>("place bottle0");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  {
    // Sample place pose
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject("bottle0");

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "bottle0";
    target_pose_msg.pose.position.x = 0.0;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame("bottle0");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,bottle0)");
    stage->allowCollisions("bottle0",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach bottle0");
    stage->detachObject("bottle0", hand_frame);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 0.3;
    stage->setDirection(vec);
    place->insert(std::move(stage));
    task.add(std::move(place));
  
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("calibrated");
    task.add(std::move(stage));
  }  

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("closed", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("closed");
    task.add(std::move(stage));
  }  

}

  return task;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  //mtc_task_node->setupPlanningScene();
  mtc_task_node->setupPlanningSceneMultiObj();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}