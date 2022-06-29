#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
bool a_bot_active = false;
bool b_bot_active = false;
std::queue<objectToMove*> objectQueue;


int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("pick_and_place");

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
  moveit::planning_interface::MoveGroupInterface move_group_interface(node,"niryo_two");
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  move_group_interface.setPlanningTime(1.0);
  move_group_interface.setNumPlanningAttempts(20.0);

  move_group_interface.setNamedTarget("resting");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  move_group_interface.execute(my_plan);

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
  pose.position.x = 0.2;
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
  pose_2.position.x = 0.25;
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
  pose_3.position.x = 0.15;
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
  pose_rect.position.x = -0.2;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  move_group_interface.setNamedTarget("resting");

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  move_group_interface.execute(my_plan);
}

void asynchronousPick(robot robot, objectToMove* object)
{

  std::string move_group;
  if(robot == a_bot){
    move_group = "a_bot";
    a_bot_active = true;
  }
  else if(robot == b_bot){
    move_group = "b_bot";
    b_bot_active = true;
  }
  
  RCLCPP_INFO(node->get_logger(),"Start Asynchronous Pick with Move group: %s",move_group.c_str());
  moveit::planning_interface::MoveGroupInterface move_group_interface(node,move_group);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  move_group_interface.setPlanningTime(15.0);
  move_group_interface.setNumPlanningAttempts(20.0);

  moveit::core::RobotModelConstPtr kinematic_model = move_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = move_group_interface.getCurrentState();
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
  bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

  if(!found_ik){
    RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");

    if(robot == a_bot){
      a_bot_active = false;
    }
    else if(robot == b_bot){
      b_bot_active = false;
    }

    return;
  }

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(joint_names, joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

  ///// Grasp

  pose.position.x = object->attached_object.object.primitive_poses[0].position.x + 0.01;
  pose.position.y = object->attached_object.object.primitive_poses[0].position.y;
  pose.position.z = 0.09;

  pose.orientation.w = 1/sqrt(2);
  pose.orientation.x = 0;
  pose.orientation.y = 1/sqrt(2);
  pose.orientation.z = 0;

  found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

  if(!found_ik){
    RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");

    if(robot == a_bot){
      a_bot_active = false;
    }
    else if(robot == b_bot){
      b_bot_active = false;
    }

    return;
  }

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(joint_names, joint_values);

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

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

  
  pose.position.y = robot==a_bot? 0.25 : -0.25;
  pose.position.z = 0.2;

  pose.orientation.w = 1/sqrt(2);
  pose.orientation.x = 0;
  pose.orientation.y = 1/sqrt(2);
  pose.orientation.z = 0;

  found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

  if(!found_ik){
    RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");

    if(robot == a_bot){
      a_bot_active = false;
    }
    else if(robot == b_bot){
      b_bot_active = false;
    }

    return;
  }

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(joint_names, joint_values);
  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

  ///// Put down 
  pose.position.z = 0.09;
  found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

  if(!found_ik){
    RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");

    if(robot == a_bot){
      a_bot_active = false;
    }
    else if(robot == b_bot){
      b_bot_active = false;
    }

    return;
  }

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(joint_names, joint_values);
  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

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

  /// Go to post-grasp

  pose.position.z = 0.2;

  found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);

  if(!found_ik){
    RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");

    if(robot == a_bot){
      a_bot_active = false;
    }
    else if(robot == b_bot){
      b_bot_active = false;
    }

    return;
  }

  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(joint_names, joint_values);
  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  move_group_interface.execute(my_plan);


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

  moveit::planning_interface::MoveGroupInterface move_group_interface(node,"niryo_two");
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  move_group_interface.setPlanningTime(1.0);
  move_group_interface.setNumPlanningAttempts(20.0);

  moveit::core::RobotModelConstPtr kinematic_model = move_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = move_group_interface.getCurrentState();
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
  bool a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
  bool b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

  kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
  kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
  move_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

  ////Grasp
  a_bot_pose.position.z = 0.11;
  b_bot_pose.position.z = 0.11;

  a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
  b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

  kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
  kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
  move_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);


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

  a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
  b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

  kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
  kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
  move_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);
  
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

  //move_group_interface.setPlanningTime(15.0);
  move_group_interface.setPathConstraints(constraints);

  /// Move

  a_bot_pose.position.x = 0.2;
  a_bot_pose.position.z = 0.2;

  b_bot_pose.position.x = 0.2;
  b_bot_pose.position.z = 0.2;

  a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
  b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

  kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
  kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
  move_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

  move_group_interface.clearPathConstraints();

  /// Put Down

  a_bot_pose.position.z = 0.11;
  b_bot_pose.position.z = 0.11;

  a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
  b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

  kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
  kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
  move_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  success = false;
  move_group_interface.execute(my_plan);

  // Detach Object
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object->attached_object.object.id;
  detach_object.link_name = object->attached_object.link_name;
  detach_object.object.operation = object->attached_object.object.REMOVE;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;

  object_pose.position.x = 0.2;
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

  a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
  b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);

  kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
  kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

  // Set joint target
  move_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
  move_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

  while(!success){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  move_group_interface.execute(my_plan);

}
