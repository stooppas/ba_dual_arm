#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <thread>
#include <math.h>

template<typename T>
std::vector<T> slice(std::vector<T> const &v, int m, int n)
{
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n + 1;
 
    std::vector<T> vec(first, last);
    return vec;
}

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

  moveit::planning_interface::MoveGroupInterface b_bot_group_interface(node,"b_bot");

  b_bot_group_interface.setMaxVelocityScalingFactor(1.0);
  b_bot_group_interface.setMaxAccelerationScalingFactor(1.0);
  b_bot_group_interface.setPlanningTime(15.0);
  b_bot_group_interface.setNumPlanningAttempts(20.0);
  
  moveit::core::RobotModelConstPtr kinematic_model = b_bot_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = b_bot_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* b_bot_joint_model_group = kinematic_model->getJointModelGroup("b_bot");  

  const std::vector<std::string>& b_bot_joint_names = b_bot_joint_model_group->getVariableNames();

  std::vector<double> pose_1 {-11* (pi / 180),-67* (pi / 180),54* (pi / 180),16* (pi / 180),-31* (pi / 180),-11* (pi / 180)};
  std::vector<double> pose_2 {-159* (pi / 180),-88* (pi / 180),67* (pi / 180),150* (pi / 180),45* (pi / 180),42* (pi / 180)};
  int i = 0;
  while(rclcpp::ok){
    if(++i%2 == 0){
        b_bot_group_interface.setJointValueTarget(b_bot_joint_names, pose_1);
    }else{
        b_bot_group_interface.setJointValueTarget(b_bot_joint_names, pose_2);
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (b_bot_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success){
      RCLCPP_INFO(node->get_logger(),"Plan did not succeed");
    }else{
      moveit::core::MoveItErrorCode result = b_bot_group_interface.execute(my_plan,rclcpp::Duration::from_seconds(0));
      if(result == moveit::core::MoveItErrorCode::SUCCESS){
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
      }else{
        RCLCPP_INFO(node->get_logger(),"Replanning");
        i++;
      }
    }
  }
}
