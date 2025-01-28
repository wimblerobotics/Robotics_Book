# Creating a Custom BlinkLight Behavior

There are a few ways to create a custom behavior in Behavior Tree.
The most common way is to create a new class that inherits from one of the Behavior Tree nodes and then implement the behavior in the `tick()` function. This is the method we will use in this example.
I'll explore another, more complicated way to create a custom behavior later in this book, which
will provide additional benefits.

Let's begin by creating a simple behavior that pretends to blink a light some number of times
This behavior will be implemented as a behavior tree action that receives a number of times to blink the light
from the XML file for the behavior tree. Here's an example of the XML file that will use the behavior:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <BlinkLight times="3" />
    </BehaviorTree>
</root>
```

When this behavior tree is executed, the ROS log file will show the message "Blinking light" three times.

The following steps will assume  you have created a ROS 2 package called `custom_behaviors`.
The package should have a `CmakeLists.txt` file, a `package.xml` file, and a `src` directory.
I'll show what needs to be in all of these files and directories.

Let's begin with the C++ code for the behavior itself.
For this example, I won't split the code into a header file and a source file, which
you would normally do. I'll put everything in a single file for simplicity.
Here is the file `blink_light.cpp` which should be placed in the `src` directory of the `custom_behaviors` package.

```cpp
#include <iostream>
#include "behaviortree_cpp/behavior_tree.h"

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
    return {BT::InputPort<int32_t>("times", "How many times to blink the light")};
  }
};

}  // namespace custom_behaviors

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<custom_behaviors::BlinkLight>("BlinkLight");
}
```

### Explanation of the C++ code

The include directives are for the 
* \<behaviortree_cpp/behavior_tree.h\>  
  Because this is a Behavior Tree node, we need to include the Behavior Tree classes.
* \<iostream\>  
  Needed for the `std::cout` statement which will mock the behavior of blinking a light.

I put the code inside a namespace called `custom_behaviors`.
This is a good practice to avoid name conflicts with other libraries.
This implication of this is that we need to use the namespace `custom_behaviors` when we refer to the `BlinkLight` class.

This class will act as a synchronous action node in the behavior tee--meaning that
it will execute its behavior in a single tick, and there will be no asynchronous 
action server to implement the actual behavior.
In another chapter, I'll show how to create a custom behavior that uses an action server
the perform the real behavior, which will allow code other than the behavriour tree defined
by the XML file to perform the action.
So the class inherits from `BT::SyncActionNode`.

The constructor for the `BlinkLight` class must take two arguments as shown
and pass them to the constructor of the base class.
When called, the constructor will get the name of the node in the XML description and the node configuration, in case we need it, which we won't in this simple example.

The destructor is set to the default, which is fine for this simple example.

Skipping the tick function for a moment, the `providedPorts()` function is a static function that returns a list of ports that the node will use.
In this case, the node will have a single input port called `times` that will be used to pass the number of times to blink the light.
The port is of type `int32_t` and so the XML file will have to provide a
***times*** attribute with an integer value. This will be shown in the test XML file later.

The `tick()` function is where the behavior is implemented.
In this case, the behavior is to print "Blinking light" to the console a number of times.
The number of times to blink the light is passed as an input via the `times` attritbute,
as declared in the `providedPorts` method for the class.

That's it for the C++ code except for one tiny detail.
For the Behavior Tree library to know about the `BlinkLight` class, we need to register it with the library.
This is done with the `BT_REGISTER_NODES` macro.
The `BT_REGISTER_NODES` macro takes a single argument, which is the name of the factory object.
The factory object is created in the `bt_factory.h` file, which is included in the code.
The `BT_REGISTER_NODES` macro is followed by a block of code that registers the `BlinkLight` class with the factory object.
This is done with the `registerNodeType` method of the factory object.
The `registerNodeType` method is templatized on the type of the node to register, which is `custom_behaviors::BlinkLight` in this case. The name of the XML element to be used in the
behavior tree XML file is passed as an argument to the `registerNodeType` method.
To use this in a behavior tree, the XML  file might contain a line like this:
```xml
<BlinkLight times="3" />
```

