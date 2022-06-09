#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>

#define pi 3.14159265359

void change_between_pose(const std::string& move_group, std::vector<double>pose_1,std::vector<double> pose_2);
rclcpp::Node::SharedPtr node;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("Collision_test");

  std::vector<double> pose_a_1 {-104* (pi / 180),0,24* (pi / 180),-23* (pi / 180),-21* (pi / 180),39* (pi / 180)};
  std::vector<double> pose_a_2 {-90* (pi / 180),0,-42* (pi / 180),30* (pi / 180),47* (pi / 180),5* (pi / 180)};
  std::vector<double> pose_b_1 {68* (pi / 180),5* (pi / 180),-9* (pi / 180),-2* (pi / 180),-16* (pi / 180),41* (pi / 180)};
  std::vector<double> pose_b_2 {128* (pi / 180),5* (pi / 180),-9* (pi / 180),-2* (pi / 180),-16* (pi / 180),41* (pi / 180)};
  const std::string a_bot = "a_bot";
  const std::string b_bot = "b_bot";

  new std::thread(change_between_pose,a_bot,pose_a_1,pose_a_2);
  new std::thread(change_between_pose,b_bot,pose_b_1,pose_b_2);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
void change_between_pose(const std::string& move_group, std::vector<double>pose_1,std::vector<double> pose_2)
{
  RCLCPP_INFO(node->get_logger(),"Move group: %s",move_group.c_str());
  moveit::planning_interface::MoveGroupInterface move_group_interface(node,move_group);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  move_group_interface.setPlanningTime(15.0);
  move_group_interface.setNumPlanningAttempts(20.0);


  moveit::core::RobotModelConstPtr kinematic_model = move_group_interface.getRobotModel();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(move_group);  
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  int i = 0;
  while(rclcpp::ok){
    if(++i%2 == 0){
        move_group_interface.setJointValueTarget(joint_names, pose_1);
    }else{
        move_group_interface.setJointValueTarget(joint_names,  pose_2);
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(!success){
      RCLCPP_INFO(node->get_logger(),"Plan did not succeed");
    }else{
        move_group_interface.execute(my_plan);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    }
  }
}
