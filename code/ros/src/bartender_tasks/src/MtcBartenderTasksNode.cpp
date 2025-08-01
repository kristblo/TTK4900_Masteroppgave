#include "MtcBartenderTasksNode.h"


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MtcBartenderTasksNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MtcBartenderTasksNode::MtcBartenderTasksNode(const rclcpp::NodeOptions& options)
  : node_{std::make_shared<rclcpp::Node>("bartender_node", options)}
{
  //node_->declare_parameter("objName", "bottle0"); //name, default val
  std::string arg_value = node_->get_parameter("objName").get_value<std::string>();
  RCLCPP_INFO(node_->get_logger(), "Received argument: %s", arg_value.c_str());
  this->objectName = arg_value;
}

void MtcBartenderTasksNode::setUpPlanningScene()
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
    pose.position.x = 0.5 - (i * 0.15);
    pose.position.y = -0.35;
    pose.position.z = 0.10;
    pose.orientation.w = 1.0;
    collisionObjects[i].pose = pose;

    collisionObjects[i].operation = collisionObjects[i].ADD;
  }

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(collisionObjects);

}


////Disable the warning until we find a use for bottleNo
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"
void MtcBartenderTasksNode::doPickTask(std::string objectName)
{  
  pickTask_ = createPickTask(objectName);

  try
  {
    pickTask_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!pickTask_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick task planning failed");
    return;
  }
  pickTask_.introspection().publishSolution(*pickTask_.solutions().front());

  auto result = pickTask_.execute(*pickTask_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick task execution failed");
    return;
  }

  return;

}
//#pragma GCC diagnostic pop

void MtcBartenderTasksNode::doPourTask(std::string objectName)
{

  pourTask_ = createPourTask(objectName);

  try
  {
    pourTask_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!pourTask_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pour task planning failed");
    return;
  }
  pourTask_.introspection().publishSolution(*pourTask_.solutions().front());

  auto result = pourTask_.execute(*pourTask_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pour task execution failed");
    return;
  }

  return;


}

void MtcBartenderTasksNode::doPlaceTask()
{

}

mtc::Task MtcBartenderTasksNode::createPickTask(std::string objectName)
{
  mtc::Task pickTask;
  pickTask.stages()->setName("Pick bottle");
  pickTask.loadRobotModel(node_);

  const auto& arm_group_name = "orca_arm";
  const auto& hand_group_name = "orca_hand";
  const auto& hand_frame = "link_twist";

  pickTask.setProperty("group", arm_group_name);
  pickTask.setProperty("eef", "gripper");
  pickTask.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr; //Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  pickTask.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  //First stage: open hand
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  pickTask.add(std::move(stage_open_hand));


  //Second stage: move to pick
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});

  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  pickTask.add(std::move(stage_move_to_pick));

  //Stage object
  this->attach_object_stage = nullptr; //Forward attach_object_stage to place pose generator


{
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  pickTask.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach object");
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
    stage->setObject(objectName);
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
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions(objectName,
                          pickTask.getRobotModel()
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
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(objectName, hand_frame);
    this->attach_object_stage = stage.get();
    grasp->insert(std::move(stage));
  }


  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
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

  pickTask.add(std::move(grasp));

}



//Attempt to rotate bottle0 (relative to its top) before placing
{
  auto stage_move_to_rotate = std::make_unique<mtc::stages::Connect>(
      "move to rotate",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
  stage_move_to_rotate->setTimeout(2.0);
  stage_move_to_rotate->properties().configureInitFrom(mtc::Stage::PARENT);
  pickTask.add(std::move(stage_move_to_rotate));
}


{
  auto rotate = std::make_unique<mtc::SerialContainer>("rotate bottle0");
  pickTask.properties().exposeTo(rotate->properties(), {"eef", "group", "ik_frame"});
  rotate->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});


  {
    // Sample place pose waypoint
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate waypoint pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "waypoint_pose");
    stage->setObject(objectName);

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = objectName;
    target_pose_msg.pose.position.z = 0.4;
    target_pose_msg.pose.position.x = -0.9;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("waypoint pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(objectName);
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
    stage->setObject(objectName);

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = objectName;
    target_pose_msg.pose.position.z = 0.08; //offset with half the bottle height
    target_pose_msg.pose.position.x = -0.9;    
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
    wrapper->setIKFrame(flip_frame_transform, objectName);
    //wrapper->setIKFrame(flip_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    rotate->insert(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("complete pour", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat_from_pour");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 0.2;
    stage->setDirection(vec);
    rotate->insert(std::move(stage));
    //task.add(std::move(rotate));
  
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
    stage->setObject(objectName);

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = objectName;
    target_pose_msg.pose.position.z = 0.20;
    target_pose_msg.pose.position.x = -0.9;
    target_pose_msg.pose.orientation.y = 0;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("unflip pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(objectName);
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

  pickTask.add(std::move(rotate));


}

/////////////////////////////PLACE///////////////////
{
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  pickTask.add(std::move(stage_move_to_place));
}





{
  auto place = std::make_unique<mtc::SerialContainer>("place bottle0");
  pickTask.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  {
    // Sample place pose waypoint
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate waypoint pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "waypoint_pose");
    stage->setObject(objectName);

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = objectName;
    target_pose_msg.pose.position.z = 0.4;
    target_pose_msg.pose.position.x = 0.0;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("waypoint pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(objectName);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(wrapper));
  }


  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place2",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
                                                  /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    place->insert(std::move(stage_move_to_place));
  }



  {
    // Sample place pose
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject(objectName);

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = objectName;
    target_pose_msg.pose.position.x = 0.0;
    target_pose_msg.pose.orientation.w = 1;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(objectName);
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
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,bottle)");
    stage->allowCollisions(objectName,
                          pickTask.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach bottle0");
    stage->detachObject(objectName, hand_frame);
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
    pickTask.add(std::move(place));
  
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("calibrated");
    pickTask.add(std::move(stage));
  }  

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("closed", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("closed");
    pickTask.add(std::move(stage));
  }  

}


  return pickTask;
}

mtc::Task MtcBartenderTasksNode::createPourTask(std::string objectName)
{
  mtc::Task pourTask;
  pourTask.stages()->setName("Pour bottle");
  pourTask.loadRobotModel(node_);

  const auto& arm_group_name = "orca_arm";
  const auto& hand_group_name = "orca_hand";
  const auto& hand_frame = "link_twist";

  pourTask.setProperty("group", arm_group_name);
  pourTask.setProperty("eef", "gripper");
  pourTask.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr; //Forward current_state on to grasp pose generator
  mtc::Stage* attach_object_stage = nullptr;

#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  pourTask.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage->allowCollisions(objectName,
                        pourTask.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        true);
  pourTask.add(std::move(stage));
}


// {
//   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("reattach bottle0");
//   stage->attachObject("bottle0", hand_frame);
//   attach_object_stage = stage.get();
//   pourTask.add(std::move(stage));
// }


{
  auto unrotate = std::make_unique<mtc::SerialContainer>("rotate bottle0");
  pourTask.properties().exposeTo(unrotate->properties(), {"eef", "group", "ik_frame"});
  unrotate->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("complete pour", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.2);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat_from_pour");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 0.1;
    stage->setDirection(vec);
    unrotate->insert(std::move(stage));
    attach_object_stage = stage.get();
    //task.add(std::move(rotate));
  
  }

  // {
  //   auto stage_connect_flip_unflip = std::make_unique<mtc::stages::Connect>(
  //       "connect flip and unflip",
  //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }//,
  //                                                 /*{ hand_group_name, sampling_planner }*/ }); //Results in complaints about not having a controller for all joints
  //   stage_connect_flip_unflip->setTimeout(2.0);
  //   stage_connect_flip_unflip->properties().configureInitFrom(mtc::Stage::PARENT);
  //   unrotate->insert(std::move(stage_connect_flip_unflip));
  // }
  
  pourTask.add(std::move(unrotate));

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  pourTask.add(std::move(stage_open_hand));


  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("calibrated");
    pourTask.add(std::move(stage));
  }  

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("closed", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("closed");
    pourTask.add(std::move(stage));
  }  

}




  return pourTask;
}

mtc::Task MtcBartenderTasksNode::createPlaceTask()
{
  mtc::Task placeTask;

  return placeTask;
}

