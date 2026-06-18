#include "easyarm_motion_server/motion_server_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<easyarm_motion_server::MotionServerNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  node->initializeMoveGroup();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
