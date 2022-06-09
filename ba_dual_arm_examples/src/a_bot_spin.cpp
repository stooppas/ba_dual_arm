#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>

#define pi 3.14159265359

void main_thread(rclcpp::Node::SharedPtr node);

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("a_bot_spin");

  new std::thread(main_thread,node);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
void main_thread(rclcpp::Node::SharedPtr node){
  std::vector<double> joint_values;

  moveit::planning_interface::MoveGroupInterface a_bot_group_interface(node,"a_bot");

  a_bot_group_interface.setMaxVelocityScalingFactor(1.0);
  a_bot_group_interface.setMaxAccelerationScalingFactor(1.0);
  a_bot_group_interface.setPlanningTime(15.0);
  a_bot_group_interface.setNumPlanningAttempts(20.0);
  
  moveit::core::RobotModelConstPtr kinematic_model = a_bot_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = a_bot_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* a_bot_joint_model_group = kinematic_model->getJointModelGroup("a_bot");  

  const std::vector<std::string>& a_bot_joint_names = a_bot_joint_model_group->getVariableNames();

  std::vector<double> pose_1 {-11* (pi / 180),-67* (pi / 180),54* (pi / 180),16* (pi / 180),-31* (pi / 180),-11* (pi / 180)};
  std::vector<double> pose_2 {-159* (pi / 180),-88* (pi / 180),67* (pi / 180),150* (pi / 180),45* (pi / 180),42* (pi / 180)};
  int i = 0;
  while(rclcpp::ok){
    if(++i%2 == 0){
        a_bot_group_interface.setJointValueTarget(a_bot_joint_names, pose_1);
    }else{
        a_bot_group_interface.setJointValueTarget(a_bot_joint_names, pose_2);
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (a_bot_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(!success){
      RCLCPP_INFO(node->get_logger(),"Plan did not succeed");
    }else{
      moveit::core::MoveItErrorCode result = a_bot_group_interface.execute(my_plan,rclcpp::Duration::from_seconds(0));
      if(result == moveit::core::MoveItErrorCode::SUCCESS){
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
      }else{
        RCLCPP_INFO(node->get_logger(),"Replanning");
        i++;
      }
    }
  }
}
