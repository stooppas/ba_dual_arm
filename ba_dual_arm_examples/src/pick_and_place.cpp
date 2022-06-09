#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <thread>
#include <math.h>

#include <niryo_one_msgs/msg/tool_command.hpp>
#include <niryo_one_msgs/action/tool.hpp>

rclcpp::Node::SharedPtr node;
void main_thread();
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
void pick(moveit::planning_interface::MoveGroupInterface& move_group);
void place(moveit::planning_interface::MoveGroupInterface& move_group);
double pi = 3.14159265359;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("moveit");

  std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
  void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<niryo_one_msgs::action::Tool>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<niryo_one_msgs::action::Tool>::SharedPtr,
    const std::shared_ptr<const niryo_one_msgs::action::Tool::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    RCLCPP_INFO(node->get_logger(), ss.str().c_str());
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<niryo_one_msgs::action::Tool>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    RCLCPP_INFO(node->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
void main_thread()
{

    auto client = rclcpp_action::create_client<niryo_one_msgs::action::Tool>(node,"/niryo_one/tool_action");
    

    auto goal_msg = niryo_one_msgs::action::Tool::Goal();

    goal_msg.cmd.tool_id = 11;
    goal_msg.cmd.cmd_type = 2 ;
    goal_msg.cmd.gripper_close_speed = 300;
    goal_msg.cmd.gripper_open_speed = 300;
    goal_msg.cmd.activate = false;
    goal_msg.cmd.gpio = 0;

    auto send_goal_options = rclcpp_action::Client<niryo_one_msgs::action::Tool>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&goal_response_callback, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&feedback_callback, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&result_callback, std::placeholders::_1);


    client->async_send_goal(goal_msg,send_goal_options);


    //place(group);

    /*

  while(    ::ok){
    float x_rotation = rad_distr(gen) * 0.005;
    float y_rotation = rad_distr(gen) * 0.005;
    float z_rotation = rad_distr(gen) * 0.005;

    auto rotA = tf2::Quaternion();
    rotA.setRPY(x_rotation,y_rotation,z_rotation);

    auto rotB = tf2::Quaternion();
    rotB.setRPY(x_rotation ,y_rotation,z_rotation);

    a_bot_q =  rotA*a_bot_q;
    b_bot_q = rotB*b_bot_q;

    a_bot_pose.orientation.w = a_bot_q.w();
    a_bot_pose.orientation.x = a_bot_q.x();
    a_bot_pose.orientation.y = a_bot_q.y();
    a_bot_pose.orientation.z = a_bot_q.z();
    b_bot_pose.orientation.w = b_bot_q.w();
    b_bot_pose.orientation.x = b_bot_q.x();
    b_bot_pose.orientation.y = b_bot_q.y();
    b_bot_pose.orientation.z = b_bot_q.z();

    float random_x = ( ((float) distr(gen)) * 0.003);
    float random_y = ( ((float) distr(gen)) * 0.003);
    float random_z = ( ((float) distr(gen)) * 0.003);


    tf2::Matrix3x3 m(a_bot_q); 
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    tf2::Vector3 unitX(1,0,0);
    unitX = unitX.rotate(a_bot_q.getAxis(),a_bot_q.getAngle());

    tf2::Vector3 vec2(cos(yaw)*cos(pitch),sin(yaw)*cos(pitch),sin(pitch));
    tf2::Vector3 vec = a_bot_q.getAxis();


    a_bot_pose.position.x = 0.05 + random_x ;
    a_bot_pose.position.y = -0.05 + random_y;
    a_bot_pose.position.z = 0.46 + random_z ;
    b_bot_pose.position.x = unitX[0]*0.2 + a_bot_pose.position.x;
    b_bot_pose.position.y = unitX[1]*0.2 + a_bot_pose.position.y;
    b_bot_pose.position.z = unitX[2]*0.2 +a_bot_pose.position.z;

    RCLCPP_INFO(node->get_logger(),"a_bot: X(%f),Y(%f),Z(%f)",a_bot_pose.position.x,a_bot_pose.position.y,a_bot_pose.position.z);
    RCLCPP_INFO(node->get_logger(),"b_bot: X(%f),Y(%f),Z(%f)",b_bot_pose.position.x,b_bot_pose.position.y,b_bot_pose.position.z);

    double timeout = 0.1;
    bool a_bot_found_ik = kinematic_state->setFromIK(a_bot_joint_model_group, a_bot_pose, timeout);
    bool b_bot_found_ik = kinematic_state->setFromIK(b_bot_joint_model_group, b_bot_pose, timeout);
    
    if(!a_bot_found_ik || !b_bot_found_ik)
    {
      RCLCPP_INFO(node->get_logger(),"Did not find IK solution");
      continue;
    }

    kinematic_state->copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_values);
    kinematic_state->copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_values);

    for (std::size_t i = 0; i < a_bot_joint_names.size(); ++i)
    {
      RCLCPP_INFO(node->get_logger(),"Joint %s: %f", a_bot_joint_names[i].c_str(), a_bot_joint_values[i]);
      RCLCPP_INFO(node->get_logger(),"Joint %s: %f", b_bot_joint_names[i].c_str(), b_bot_joint_values[i]);
    }

    ab_group_interface.setJointValueTarget(a_bot_joint_names, a_bot_joint_values);
    ab_group_interface.setJointValueTarget(b_bot_joint_names, b_bot_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (ab_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success){
      RCLCPP_INFO(node->get_logger(),"Plan did not succeed");
    }

    ab_group_interface.execute(my_plan);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    
  }
  */
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
void openGripper(trajectory_msgs::msg::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::msg::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::msg::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);

  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{

  group.setSupportSurfaceName("table2");
}