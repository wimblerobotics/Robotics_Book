#include <behaviortree_cpp/behavior_tree.h>
#include <iostream>

namespace custom_behaviors {

class BlinkLight : public BT::SyncActionNode {
 public:
  /**
   * @brief A constructor for custom_behaviors::BlinkLight
   * @param xml_tag_name Name of the node in the XML description
   * @param conf BT node configuration
   */
  BlinkLight(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::SyncActionNode(xml_tag_name, conf) {}

  /**
   * @brief A destructor for custom_behaviors::BlinkLight
   */
  ~BlinkLight() = default;

  /**
   * @brief Function to perform some user-defined operation on tick
   * @return BT::NodeStatus Status of the node
   */
  BT::NodeStatus tick() override {
    int32_t times;
    getInput("times", times);
    for (int i = 0; i < times; i++) {
      std::cout << "Blinking light" << std::endl;
    }

    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int32_t>("times", "How many times to blink the light")};
  }
};

}  // namespace custom_behaviors

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<custom_behaviors::BlinkLight>("BlinkLight");
}
