#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "bt_stepper/SS.hpp"

class CustomSequence : public BT::ControlNode {
public:
  CustomSequence(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config), current_child_idx_(0) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    const auto& children = this->children();
    if (children.empty()) {
      return BT::NodeStatus::SUCCESS;
    }
    if (current_child_idx_ >= children.size()) {
      return BT::NodeStatus::SUCCESS;
    }
    auto child_status = children[current_child_idx_]->executeTick();
    if (child_status != BT::NodeStatus::RUNNING) {
      current_child_idx_++;
    }
    return child_status;
  }

private:
  size_t current_child_idx_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_stepper_node");

  std::string xml_path;
  node->declare_parameter<std::string>("xml_path", "foo");
  node->get_parameter("xml_path", xml_path);

  node->declare_parameter<bool>("tick", false);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_stepper::SS>("SS");
  factory.registerNodeType<CustomSequence>("CustomSequence");
  auto tree = factory.createTreeFromFile(xml_path);

  // Add this line to enable Groot monitoring
  BT::StdCoutLogger logger_cout(tree);

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
      if (status != BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Tick returned: %s",
                    BT::toStr(status).c_str());
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