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

rclcpp::Node::SharedPtr node;
void main_thread();
double pi = 3.14159265359;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("moveit");

  std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
void main_thread(){
  std::vector<double> joint_values;

  moveit::planning_interface::MoveGroupInterface a_bot_group_interface(node,"a_bot");
  RCLCPP_INFO(node->get_logger(),"reached");

  moveit::planning_interface::MoveGroupInterface ab_group_interface(node,"niryo_two");


  moveit::planning_interface::MoveGroupInterface b_bot_group_interface(node,"b_bot");

  ab_group_interface.setMaxVelocityScalingFactor(1.0);
  ab_group_interface.setMaxAccelerationScalingFactor(1.0);
  ab_group_interface.setPlanningTime(15.0);
  ab_group_interface.setNumPlanningAttempts(20.0);
  
  moveit::core::RobotModelConstPtr kinematic_model = a_bot_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = ab_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* a_bot_joint_model_group = kinematic_model->getJointModelGroup("a_bot");
  const moveit::core::JointModelGroup* b_bot_joint_model_group = kinematic_model->getJointModelGroup("b_bot");
  

  const std::vector<std::string>& a_bot_joint_names = a_bot_joint_model_group->getVariableNames();
  const std::vector<std::string>& b_bot_joint_names = b_bot_joint_model_group->getVariableNames();

  std::vector<double> a_bot_joint_values;
  std::vector<double> b_bot_joint_values;

  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  geometry_msgs::msg::Pose a_bot_pose;
  geometry_msgs::msg::Pose b_bot_pose;
 
  // x y z w
  tf2::Quaternion a_bot_q(-0.0018064082028716572, 0.714190127940822, -0.6999353940485185,0.0044319521005895665);
  tf2::Quaternion b_bot_q(-0.6959453209354478, -0.029260144371181365, -0.02361341612136324,0.7171097271676862 );

  while(rclcpp::ok){
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
}
