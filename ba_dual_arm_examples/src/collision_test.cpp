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
std::shared_ptr<std::thread> run_thread;
void main_thread();
double pi = 3.14159265359;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("moveit");

  run_thread.reset(new std::thread(std::bind(&main_thread)));

  rclcpp::spin(node);
  rclcpp::shutdown();
}
void main_thread(){
  std::vector<double> joint_values;

 moveit::planning_interface::MoveGroupInterface a_bot_group_interface(node,"a_bot");
 moveit::planning_interface::MoveGroupInterface b_bot_group_interface(node,"b_bot");

  a_bot_group_interface.setMaxVelocityScalingFactor(1.0);
  a_bot_group_interface.setMaxAccelerationScalingFactor(1.0);
  a_bot_group_interface.setPlanningTime(15.0);
  a_bot_group_interface.setNumPlanningAttempts(20.0);
  
  b_bot_group_interface.setMaxVelocityScalingFactor(1.0);
  b_bot_group_interface.setMaxAccelerationScalingFactor(1.0);
  b_bot_group_interface.setPlanningTime(15.0);
  b_bot_group_interface.setNumPlanningAttempts(20.0);

  moveit::core::RobotModelConstPtr kinematic_model_a = a_bot_group_interface.getRobotModel();
  const moveit::core::JointModelGroup* a_bot_joint_model_group = kinematic_model_a->getJointModelGroup("a_bot");  
  moveit::core::RobotModelConstPtr kinematic_model_b = b_bot_group_interface.getRobotModel();
  const moveit::core::JointModelGroup* b_bot_joint_model_group = kinematic_model_b->getJointModelGroup("b_bot");  

  const std::vector<std::string>& a_bot_joint_names = a_bot_joint_model_group->getVariableNames();
  const std::vector<std::string>& b_bot_joint_names = b_bot_joint_model_group->getVariableNames();

std::vector<double> pose_a_1 {-104* (pi / 180),0,24* (pi / 180),-23* (pi / 180),-21* (pi / 180),39* (pi / 180)};
std::vector<double> pose_a_2 {-90* (pi / 180),0,-42* (pi / 180),30* (pi / 180),47* (pi / 180),5* (pi / 180)};
std::vector<double> pose_b_1 {68* (pi / 180),5* (pi / 180),-9* (pi / 180),-2* (pi / 180),-16* (pi / 180),41* (pi / 180)};
std::vector<double> pose_b_2 {128* (pi / 180),5* (pi / 180),-9* (pi / 180),-2* (pi / 180),-16* (pi / 180),41* (pi / 180)};
int i = 0;
  while(rclcpp::ok){
    if(++i%2 == 0){
        a_bot_group_interface.setJointValueTarget(a_bot_joint_names, pose_a_1);
        b_bot_group_interface.setJointValueTarget(b_bot_joint_names, pose_b_1);
    }else{
        a_bot_group_interface.setJointValueTarget(a_bot_joint_names,  pose_a_2);
        b_bot_group_interface.setJointValueTarget(b_bot_joint_names, pose_b_2);
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_a;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_b;
    bool success_a = (a_bot_group_interface.plan(my_plan_a) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success_b = (b_bot_group_interface.plan(my_plan_b) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success_a || !success_b){
      RCLCPP_INFO(node->get_logger(),"Plan did not succeed");
    }else{
        a_bot_group_interface.asyncExecute(my_plan_a);
        b_bot_group_interface.asyncExecute(my_plan_b);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    }
  }

}
