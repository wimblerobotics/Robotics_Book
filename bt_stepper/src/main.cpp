#include <behaviortree_cpp/bt_factory.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "bt_stepper/SS.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_stepper_node");

  std::string xml_path;
  node->declare_parameter<std::string>("xml_path", "foo");
  node->get_parameter("xml_path", xml_path);

  node->declare_parameter<bool>("tick", false);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_stepper::SS>("SS");
  auto tree = factory.createTreeFromFile(xml_path);
  // node->declare_parameter("tick", "false");
  RCLCPP_INFO(node->get_logger(), "Starting manual BT ticks...");

  while (rclcpp::ok()) {
    bool tick = false;
    static bool say_message = true;
    if (say_message) {
      RCLCPP_INFO(node->get_logger(), "Waiting for tick...");
      say_message = false;
    }
    tick = node->get_parameter("tick").as_bool();

    if (tick) {
      say_message = true;
      auto status = tree.tickExactlyOnce();
      RCLCPP_INFO(node->get_logger(), "Tick returned: %s",
                  BT::toStr(status).c_str());
      rclcpp::spin_some(node);
      if (status != BT::NodeStatus::RUNNING) {
        break;
      }
      node->set_parameter(rclcpp::Parameter("tick", false));
      tick = false;
    } else {
      rclcpp::spin_some(node);
    }
  }

  rclcpp::shutdown();
  return 0;
}