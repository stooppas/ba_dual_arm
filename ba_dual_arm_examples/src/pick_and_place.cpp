#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <thread>

rclcpp::Node::SharedPtr node;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("pick_and_place");

  RCLCPP_ERROR(node->get_logger(),"Not implemented yet!");

  //rclcpp::spin(node);
  rclcpp::shutdown();
}
