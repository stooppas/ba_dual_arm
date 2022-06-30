#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "niryo_one_msgs/srv/open_gripper.hpp"
#include "niryo_one_msgs/srv/close_gripper.hpp"
#include "niryo_one_msgs/srv/ping_dxl_tool.hpp"


#include <stdlib.h>
#include <thread>
#include <chrono>
#include <queue>

enum objectType {square,rectangle};

enum robot {a_bot,b_bot};

struct objectToMove{
  objectToMove(objectType type,moveit_msgs::msg::AttachedCollisionObject attached_object){
    this->type = type;
    this-> attached_object = attached_object;
  }

  objectType type;
  moveit_msgs::msg::AttachedCollisionObject attached_object;

};

void main_thread();
void asynchronousPick(robot robot, objectToMove* object);
void synchronousPick(objectToMove* object);
rclcpp::Node::SharedPtr node;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> a_bot_interface;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> b_bot_interface;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> niryo_two_interface;

bool sim;
rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;

rclcpp::Client<niryo_one_msgs::srv::OpenGripper>::SharedPtr a_bot_open_gripper;
rclcpp::Client<niryo_one_msgs::srv::CloseGripper>::SharedPtr a_bot_close_gripper;
rclcpp::Client<niryo_one_msgs::srv::OpenGripper>::SharedPtr b_bot_open_gripper;
rclcpp::Client<niryo_one_msgs::srv::CloseGripper>::SharedPtr b_bot_close_gripper;


bool a_bot_active = false;
bool b_bot_active = false;
std::queue<objectToMove*> objectQueue;

void open_gripper(bool a){
  if(!sim){
    RCLCPP_INFO(node->get_logger(),"Open Gripper");
    auto message = std::make_shared<niryo_one_msgs::srv::OpenGripper::Request>();
    message->id = 11;
    message->open_speed = 300;
    message->open_hold_torque = 128;
    message->open_position = 600;

    if(a){
      auto fut = a_bot_open_gripper->async_send_request(message);
      fut.wait();
    }else{
      auto fut = b_bot_open_gripper->async_send_request(message);
      fut.wait();
    }
  }
}

void close_gripper(bool a){
  if(!sim){
    RCLCPP_INFO(node->get_logger(),"Close Gripper");
    auto message = std::make_shared<niryo_one_msgs::srv::CloseGripper::Request>();
    message->id = 11;
    message->close_speed = 300;
    message->close_hold_torque = 128;
    message->close_max_torque = 1023;
    message->close_position = 230;

    if(a){
      auto fut = a_bot_close_gripper->async_send_request(message);
      fut.wait();
    }else{
      auto fut = b_bot_close_gripper->async_send_request(message);
      fut.wait();
    }
  }
}
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  node = std::make_shared<rclcpp::Node>("pick_and_place",node_options);

  node->get_parameter("sim",sim);

  RCLCPP_INFO(node->get_logger(),"Sim: %i",sim);

  if(!sim){
    a_bot_open_gripper = node->create_client<niryo_one_msgs::srv::OpenGripper>("/a_bot/niryo_one/tools/open_gripper");
    a_bot_close_gripper = node->create_client<niryo_one_msgs::srv::CloseGripper>("/a_bot/niryo_one/tools/close_gripper");

    b_bot_open_gripper = node->create_client<niryo_one_msgs::srv::OpenGripper>("/b_bot/niryo_one/tools/open_gripper");
    b_bot_close_gripper = node->create_client<niryo_one_msgs::srv::CloseGripper>("/b_bot/niryo_one/tools/close_gripper");

    auto a_bot_ping_dxl_client = node->create_client<niryo_one_msgs::srv::PingDxlTool>("/a_bot/niryo_one/tools/ping_and_set_dxl_tool");
    auto b_bot_ping_dxl_client = node->create_client<niryo_one_msgs::srv::PingDxlTool>("/b_bot/niryo_one/tools/ping_and_set_dxl_tool");


    while (!b_bot_open_gripper->wait_for_service(1s) && !b_bot_close_gripper->wait_for_service(1s)&& !b_bot_ping_dxl_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "b_bot services not available, waiting again...");
    }  

    while (!a_bot_open_gripper->wait_for_service(1s) && !a_bot_close_gripper->wait_for_service(1s) && !a_bot_ping_dxl_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "a_bot services not available, waiting again...");
    }  

    auto request = std::make_shared<niryo_one_msgs::srv::PingDxlTool::Request>();

    request->id = 11;
    request->name = "Gripper 1";

    auto res = a_bot_ping_dxl_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, res);

    res = b_bot_ping_dxl_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, res);
  }


  //Start planning scene publisher
  planning_scene_diff_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }


  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread()
{
  niryo_two_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"niryo_two");
  a_bot_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"a_bot");
  b_bot_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"b_bot");

  niryo_two_interface->setMaxVelocityScalingFactor(0.3);
  niryo_two_interface->setMaxAccelerationScalingFactor(0.1);
  niryo_two_interface->setPlanningTime(5.0);

  a_bot_interface->setMaxVelocityScalingFactor(0.3);
  a_bot_interface->setMaxAccelerationScalingFactor(0.1);
  a_bot_interface->setPlanningTime(5.0);

  b_bot_interface->setMaxVelocityScalingFactor(0.3);
  b_bot_interface->setMaxAccelerationScalingFactor(0.1);
  b_bot_interface->setPlanningTime(5.0);


  niryo_two_interface->setNamedTarget("resting");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  while(!success){
    success = (niryo_two_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  niryo_two_interface->execute(my_plan);
  

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;


  //Three squares on one Side

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "world";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "world";
  /* The id of the object */
  attached_object.object.id = "box1";

  attached_object.touch_links.push_back("a_bot_mors_1");
  attached_object.touch_links.push_back("a_bot_mors_2");
  attached_object.touch_links.push_back("b_bot_mors_1");
  attached_object.touch_links.push_back("b_bot_mors_2");


  /* A default pose */
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.15;
  pose.position.z = 0.0075;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.015;
  primitive.dimensions[1] = 0.015;
  primitive.dimensions[2] = 0.015;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operati

  planning_scene.world.collision_objects.push_back(attached_object.object);
  objectQueue.push(new objectToMove(square,attached_object));

  moveit_msgs::msg::AttachedCollisionObject attached_object_2;
  attached_object_2.link_name = "world";
  /* The header must contain a valid TF frame*/
  attached_object_2.object.header.frame_id = "world";
  /* The id of the object */
  attached_object_2.object.id = "box2";

  attached_object_2.touch_links.push_back("a_bot_mors_1");
  attached_object_2.touch_links.push_back("a_bot_mors_2");
  attached_object_2.touch_links.push_back("b_bot_mors_1");
  attached_object_2.touch_links.push_back("b_bot_mors_2");


  /* A default pose */
  geometry_msgs::msg::Pose pose_2;
  pose_2.position.x = 0.10;
  pose_2.position.z = 0.0075;
  pose_2.orientation.w = 1.0;

  attached_object_2.object.primitives.push_back(primitive);
  attached_object_2.object.primitive_poses.push_back(pose_2);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operati

  planning_scene.world.collision_objects.push_back(attached_object_2.object);
  objectQueue.push(new objectToMove(square,attached_object_2));

  moveit_msgs::msg::AttachedCollisionObject attached_object_3;
  attached_object_3.link_name = "world";
  /* The header must contain a valid TF frame*/
  attached_object_3.object.header.frame_id = "world";
  /* The id of the object */
  attached_object_3.object.id = "box3";

  attached_object_3.touch_links.push_back("a_bot_mors_1");
  attached_object_3.touch_links.push_back("a_bot_mors_2");
  attached_object_3.touch_links.push_back("b_bot_mors_1");
  attached_object_3.touch_links.push_back("b_bot_mors_2");


  /* A default pose */
  geometry_msgs::msg::Pose pose_3;
  pose_3.position.x = 0.05;
  pose_3.position.z = 0.0075;
  pose_3.orientation.w = 1.0;

  attached_object_3.object.primitives.push_back(primitive);
  attached_object_3.object.primitive_poses.push_back(pose_3);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operati

  planning_scene.world.collision_objects.push_back(attached_object_3.object);
  objectQueue.push(new objectToMove(square,attached_object_3));

  //One bigger rectangle on the second

  moveit_msgs::msg::AttachedCollisionObject attached_object_rect;
  attached_object_rect.link_name = "world";
  /* The header must contain a valid TF frame*/
  attached_object_rect.object.header.frame_id = "world";
  /* The id of the object */
  attached_object_rect.object.id = "rect1";

  attached_object_rect.touch_links.push_back("a_bot_mors_1");
  attached_object_rect.touch_links.push_back("a_bot_mors_2");
  attached_object_rect.touch_links.push_back("b_bot_mors_1");
  attached_object_rect.touch_links.push_back("b_bot_mors_2");
  

  /* A default pose */
  geometry_msgs::msg::Pose pose_rect;
  pose_rect.position.x = -0.15;
  pose_rect.position.z = 0.025;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive_rect;
  primitive_rect.type = primitive_rect.BOX;
  primitive_rect.dimensions.resize(3);
  primitive_rect.dimensions[0] = 0.01;
  primitive_rect.dimensions[1] = 0.2;
  primitive_rect.dimensions[2] = 0.05;


  attached_object_rect.object.primitives.push_back(primitive_rect);
  attached_object_rect.object.primitive_poses.push_back(pose_rect);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operati

  planning_scene.world.collision_objects.push_back(attached_object_rect.object);
  objectQueue.push(new objectToMove(rectangle,attached_object_rect));


  planning_scene_diff_publisher->publish(planning_scene);

  while(rclcpp::ok && !objectQueue.empty()){
    if(!a_bot_active || !b_bot_active){
      auto next = objectQueue.front();
      objectQueue.pop();
      if(next->type == square){
        //Check if reachable by robot
        if(!a_bot_active){
          new std::thread(asynchronousPick,a_bot,next);
        } 
        else if(!b_bot_active){
           new std::thread(asynchronousPick,b_bot,next);
        }
      }
      else if(next->type == rectangle){
        while(a_bot_active || b_bot_active){
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        synchronousPick(next);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  niryo_two_interface->setNamedTarget("resting");

  while(!success){
    success = (niryo_two_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  niryo_two_interface->execute(my_plan);
}

void asynchronousPick(robot robot, objectToMove* object)
{
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  std::string move_group;
  if(robot == a_bot){
    move_group = "a_bot";
    a_bot_active = true;
    move_group_interface = a_bot_interface;
  }
  else if(robot == b_bot){
    move_group = "b_bot";
    b_bot_active = true;
    move_group_interface = b_bot_interface;
  }
  
  RCLCPP_INFO(node->get_logger(),"Start Asynchronous Pick with Move group: %s",move_group.c_str());
  (node,move_group);

  moveit::core::RobotModelConstPtr kinematic_model = move_group_interface->getRobotModel();
  moveit::core::RobotStatePtr kinematic_state;
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(move_group);  
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  ///// Pre Grasp
  geometry_msgs::msg::Pose pose;

  pose.position.x = object->attached_object.object.primitive_poses[0].position.x + 0.01;
  pose.position.y = object->attached_object.object.primitive_poses[0].position.y;
  pose.position.z = 0.2;

  pose.orientation.w = 1/sqrt(2);
  pose.orientation.x = 0;
  pose.orientation.y = 1/sqrt(2);
  pose.orientation.z = 0;

  std::vector<double> joint_values;

  double timeout = 0.1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool found_ik, success;
  do{
    kinematic_state =move_group_interface->getCurrentState();
    move_group_interface->setStartStateToCurrentState();
    found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

    if(!found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(joint_names, joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  open_gripper(robot == a_bot);

  ///// Grasp

  pose.position.x = object->attached_object.object.primitive_poses[0].position.x + 0.01;
  pose.position.y = object->attached_object.object.primitive_poses[0].position.y;
  pose.position.z = 0.09;

  pose.orientation.w = 1/sqrt(2);
  pose.orientation.x = 0;
  pose.orientation.y = 1/sqrt(2);
  pose.orientation.z = 0;
  do{
    kinematic_state =move_group_interface->getCurrentState();
    move_group_interface->setStartStateToCurrentState();
    found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

    if(!found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(joint_names, joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  close_gripper(robot == a_bot);

  ////// Move to Pre-Put-Down

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = object->attached_object.object.id;
  remove_object.header.frame_id = object->attached_object.object.header.frame_id;
  remove_object.operation = remove_object.REMOVE;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);


  geometry_msgs::msg::Pose object_pose;
  object_pose.position.z = + 0.09;
  object_pose.position.y = - 0.01;

  object->attached_object.object.primitive_poses.clear();
  object->attached_object.object.primitive_poses.push_back(object_pose);

  object->attached_object.object.operation = object->attached_object.object.ADD;
  object->attached_object.link_name = move_group + "_hand_link";
  object->attached_object.object.header.frame_id = move_group + "_hand_link";

  planning_scene.robot_state.attached_collision_objects.push_back(object->attached_object);
  planning_scene_diff_publisher->publish(planning_scene);

  pose.position.x += 0.1;
  pose.position.y = robot==a_bot? 0.25 : -0.25;
  pose.position.z = 0.2;

  pose.orientation.w = 1/sqrt(2);
  pose.orientation.x = 0;
  pose.orientation.y = 1/sqrt(2);
  pose.orientation.z = 0;
  do{
    kinematic_state =move_group_interface->getCurrentState();
    move_group_interface->setStartStateToCurrentState();
    found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

    if(!found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(joint_names, joint_values);
    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  ///// Put down 
  pose.position.z = 0.09;
  do{
    kinematic_state =move_group_interface->getCurrentState();
    move_group_interface->setStartStateToCurrentState();
    found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

    if(!found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(joint_names, joint_values);
    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object->attached_object.object.id;
  detach_object.link_name = object->attached_object.link_name;
  detach_object.object.operation = object->attached_object.object.REMOVE;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;

  object_pose.position.x = pose.position.x - 0.01;
  object_pose.position.y = pose.position.y;
  object_pose.position.z = object->attached_object.object.primitives[0].dimensions[2]/2;

  object->attached_object.object.primitive_poses.clear();
  object->attached_object.object.primitive_poses.push_back(object_pose);

  object->attached_object.object.operation = object->attached_object.object.ADD;
  object->attached_object.link_name = "world";
  object->attached_object.object.header.frame_id = "world";

  planning_scene.world.collision_objects.push_back(object->attached_object.object);
  
  planning_scene_diff_publisher->publish(planning_scene);

  open_gripper(robot == a_bot);

  /// Go to post-grasp

  pose.position.z = 0.2;
  do{
    kinematic_state =move_group_interface->getCurrentState();
    move_group_interface->setStartStateToCurrentState();
    found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

    if(!found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(joint_names, joint_values);
    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  if(robot == a_bot){
    a_bot_active = false;
  }
  else if(robot == b_bot){
    b_bot_active = false;
  }
}

void synchronousPick(objectToMove* object)
{
  RCLCPP_INFO(node->get_logger(),"Start Synchronous Pick");

  auto move_group_interface = niryo_two_interface;

  moveit::core::RobotModelConstPtr kinematic_model = move_group_interface->getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = move_group_interface->getCurrentState();
  const moveit::core::JointModelGroup* a_bot_joint_model_group = kinematic_model->getJointModelGroup("a_bot");
  const moveit::core::JointModelGroup* b_bot_joint_model_group = kinematic_model->getJointModelGroup("b_bot");
  

  const std::vector<std::string>& a_bot_joint_names = a_bot_joint_model_group->getVariableNames();
  const std::vector<std::string>& b_bot_joint_names = b_bot_joint_model_group->getVariableNames();

  geometry_msgs::msg::Pose a_bot_pose;
  geometry_msgs::msg::Pose b_bot_pose;

  //// Pre Grasp

  a_bot_pose.position.x = object->attached_object.object.primitive_poses[0].position.x;
  a_bot_pose.position.y = object->attached_object.object.primitive_poses[0].position.y + object->attached_object.object.primitives[0].dimensions[1]/2 - 0.02;
  a_bot_pose.position.z = 0.2;

  a_bot_pose.orientation.w = -0.5;
  a_bot_pose.orientation.x = -0.5;
  a_bot_pose.orientation.y = -0.5;
  a_bot_pose.orientation.z = 0.5;

  b_bot_pose.position.x = object->attached_object.object.primitive_poses[0].position.x;
  b_bot_pose.position.y = object->attached_object.object.primitive_poses[0].position.y - object->attached_object.object.primitives[0].dimensions[1]/2 + 0.02;
  b_bot_pose.position.z = 0.2;

  b_bot_pose.orientation.w = 0.5;
  b_bot_pose.orientation.x = -0.5;
  b_bot_pose.orientation.y = 0.5;
  b_bot_pose.orientation.z = 0.5;

  std::vector<double> a_bot_joint_values;
  std::vector<double> b_bot_joint_values;

  double timeout = 0.1;
  bool a_bot_found_ik,b_bot_found_ik,success;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  do{
    a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

    if(!a_bot_found_ik || !b_bot_found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    move_group_interface->setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    bool success;

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  open_gripper(true);
  open_gripper(false);

  ////Grasp
  a_bot_pose.position.z = 0.11;
  b_bot_pose.position.z = 0.11;
  do{
    a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

    if(!a_bot_found_ik || !b_bot_found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    move_group_interface->setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  close_gripper(false);
  close_gripper(true);


  //Attach Object

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = object->attached_object.object.id;
  remove_object.header.frame_id = object->attached_object.object.header.frame_id;
  remove_object.operation = remove_object.REMOVE;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);


  geometry_msgs::msg::Pose object_pose;
  object_pose.position.z = 0.09;//0.11 - object->attached_object.object.primitives[0].dimensions[2]/2;
  object_pose.position.y = object->attached_object.object.primitives[0].dimensions[1]/2 - 0.02;

  object->attached_object.object.primitive_poses.clear();
  object->attached_object.object.primitive_poses.push_back(object_pose);

  object->attached_object.object.operation = object->attached_object.object.ADD;
  object->attached_object.link_name =  "a_bot_hand_link";
  object->attached_object.object.header.frame_id =  "a_bot_hand_link";

  planning_scene.robot_state.attached_collision_objects.push_back(object->attached_object);
  planning_scene_diff_publisher->publish(planning_scene);


  /// Pre-Move 
  a_bot_pose.position.z = 0.2;

  b_bot_pose.position.z = 0.2;
  do{
    a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

    if(!a_bot_found_ik || !b_bot_found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    move_group_interface->setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);
  
  // Constraints
  moveit_msgs::msg::Constraints constraints;

  moveit_msgs::msg::OrientationConstraint a_bot_orientation_constraint;
  a_bot_orientation_constraint.orientation.w = 0;
  a_bot_orientation_constraint.orientation.x = 1;
  a_bot_orientation_constraint.link_name = "a_bot_hand_link";
  a_bot_orientation_constraint.header.frame_id = "a_bot_base_link";
  a_bot_orientation_constraint.absolute_x_axis_tolerance = 0.1;
  a_bot_orientation_constraint.absolute_y_axis_tolerance = 0.1;
  a_bot_orientation_constraint.absolute_z_axis_tolerance = 0.1;
  a_bot_orientation_constraint.weight = 1.0;


  moveit_msgs::msg::OrientationConstraint b_bot_orientation_constraint;
  b_bot_orientation_constraint.orientation.w = 0;
  b_bot_orientation_constraint.orientation.y = 1;
  b_bot_orientation_constraint.link_name = "b_bot_hand_link";
  b_bot_orientation_constraint.header.frame_id = "b_bot_base_link";
  b_bot_orientation_constraint.absolute_x_axis_tolerance = 0.1;
  b_bot_orientation_constraint.absolute_y_axis_tolerance = 0.1;
  b_bot_orientation_constraint.absolute_z_axis_tolerance = 0.1;
  b_bot_orientation_constraint.weight = 1.0;

  constraints.orientation_constraints.push_back(a_bot_orientation_constraint);
  constraints.orientation_constraints.push_back(b_bot_orientation_constraint);

  //move_group_interface->setPlanningTime(15.0);
  move_group_interface->setPathConstraints(constraints);

  /// Move

  a_bot_pose.position.x = 0.2;
  a_bot_pose.position.z = 0.2;

  b_bot_pose.position.x = 0.2;
  b_bot_pose.position.z = 0.2;

  do{
    a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

    if(!a_bot_found_ik || !b_bot_found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    move_group_interface->setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  move_group_interface->clearPathConstraints();

  /// Put Down

  a_bot_pose.position.z = 0.11;
  b_bot_pose.position.z = 0.11;

  do{
    a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

    if(!a_bot_found_ik || !b_bot_found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    move_group_interface->setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    success = false;
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

  open_gripper(true);
  open_gripper(false);

  // Detach Object
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object->attached_object.object.id;
  detach_object.link_name = object->attached_object.link_name;
  detach_object.object.operation = object->attached_object.object.REMOVE;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;

  object_pose.position.x = 0.15;
  object_pose.position.y = 0;
  object_pose.position.z = object->attached_object.object.primitives[0].dimensions[2]/2;

  object->attached_object.object.primitive_poses.clear();
  object->attached_object.object.primitive_poses.push_back(object_pose);

  object->attached_object.object.operation = object->attached_object.object.ADD;
  object->attached_object.link_name = "world";
  object->attached_object.object.header.frame_id = "world";

  planning_scene.world.collision_objects.push_back(object->attached_object.object);
  
  planning_scene_diff_publisher->publish(planning_scene);


  // Post Move

  a_bot_pose.position.z = 0.2;
  b_bot_pose.position.z = 0.2;
  do{
    a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

    if(!a_bot_found_ik || !b_bot_found_ik){
      RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    // Set joint target
    move_group_interface->setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    move_group_interface->setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    while(!success){
      success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
  }
  while(move_group_interface->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS);

}
