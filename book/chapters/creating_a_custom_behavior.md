# Creating a Custom BlinkLight Behavior

There are a few ways to create a custom behavior for use in behavior trees.
One way is to create a new class that inherits from one of the Behavior Tree nodes, and then implement the intended behavior in the `tick()` function. This is the method we will use in this example.
There is also another way to create a custom behavior, which has additional benefits.
I'll explore that other way later in this book.

Let's begin by creating a simple behavior called ***BlinkLight*** that pretends to blink a light some number of times
This behavior will be implemented as a behavior tree action which receives, as a parameter,
the number of times to blink the light
from the XML file for the behavior tree.

Remember that behavior tree actions are leaf nodes that typically do the interesting work in a behavior tree.
Other kinds of behavior tree nodes are condition nodes that usually decide which branch of the tree to follow,
control nodes, which manage the flow of the behavior tree, and decorator nodes, which modify the behavior of other nodes.

Here's an example of the XML file that will use the ***BlinkLight*** behavior:

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

## The C++ code for the custom behavior

Let's begin with the C++ code for the behavior itself.
For this example, I won't split the code into a header file and a source file, which
you would normally do. I'll put everything in a single file for simplicity.
Here is the file `blink_light.cpp` which should be placed in the `src` directory of the `custom_behaviors` package.

```cpp
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

The include directives are: 
* \<behaviortree_cpp/behavior_tree.h\>  
  Because this is a Behavior Tree node, we need to include the Behavior Tree classes.
* \<iostream\>  
  Needed for the `std::cout` statement which will mock the behavior of blinking a light.

I put the code inside a namespace called `custom_behaviors`.
Using a namespace is a good practice to avoid name conflicts with other libraries.

This class will act as a synchronous action node in the behavior tree
so the class inherits from `BT::SyncActionNode`.
This means that the node will execute its behavior in a single tick,
there will be no asynchronous action server to implement the actual behavior.
Later on, I'll show how to create a custom behavior that uses an action server to
the perform the real behavior, which will allow for long-running behaviors and
for code other than the XML-defined behavior tree to perform the action.


The constructor for the `BlinkLight` class must take two arguments as shown
and pass them to the constructor of the base class.
When called, the constructor will get the name of the node in the XML description and the node configuration, in case we need it, which we won't in this simple example.

The destructor is set to the default, which is fine for this simple example.

Skipping the tick function for a moment, the `providedPorts()` function is a static function that returns a list of ports that the node will use.
In this case, the node will have a single input port called `times` which will be used to pass the number of times to blink the light.
The value provided must be a 32-bit integer.
The XML file will have to provide a
***times*** attribute with an integer value, as shown in the XML example above.

The `tick()` function is where the behavior is implemented.
In this case, the behavior is to print "Blinking light" to the console a number of times.
The function gets the `times` value by calling the `getInput()` method.

That's it for the C++ code except for one tiny detail.
For the Behavior Tree library to know about the `BlinkLight` class, we need to register it with the library.
This is done with the `BT_REGISTER_NODES` macro which takes a single argument,
which can be any name you like.
The argument will be populated with a factory constructor object by the Behavior Tree framework.

The `BT_REGISTER_NODES` macro is filled in with a block of code that registers the `BlinkLight` class with the factory object.
The `registerNodeType` method is templatized on the type of the node to register, which is `custom_behaviors::BlinkLight` in this case, and the argument to the call
gives the name of the node as it will appear in the XML file.
The macro creates a bit of magic code that gets executed when the class gets loaded into memory.

To use this in a behavior tree, the XML  file might contain a line like this:
```xml
<BlinkLight times="3" />
```
## The CMakeLists.txt file

ROS 2 uses CMake to build packages, and the `CMakeLists.txt` file is where you specify how to build your package. Create a `CMakeLists.txt` file in the `custom_behaviors` package directory with the following content:

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_behaviors)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${behaviortree_cpp_INCLUDE_DIRS}
)

set(dependencies
  behaviortree_cpp
)

add_library(blink_light_plugin_node SHARED
  src/blink_light.cpp
)

list(APPEND plugin_libs blink_light_plugin_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_package()
```
The first lines are somewhat boilerplate code that sets the minimum required version of CMake, sets the C and C++ standards, and sets some compiler options.
You can play with these settings if you like, but they are fine for this example.

The `find_package` lines are used to find the dependencies of the package.
In particular, we need ament_cmake so that we can use the ament macros, and we need behaviortree_cpp so that we can use the Behavior Tree library.

The `include_directories` lines are used to include the directories where the header files are located.
In this case, we only need the directory where the Behavior Tree header files are located.

The `set(dependencies ...)` lines are used to set the dependencies of the package.
I'm using this ***set*** form to make it easier to add more dependencies later on.

The `add_library` line is used to create a shared library from the `blink_light.cpp` file.
The library will be called `blink_light_plugin_node` and later on, the
***navigation.yaml*** file will refer to this library to cause it to be
automatically loaded by the ***Nav2 Stack***--in particular, the ***bt_navigator*** ***ROS 2*** node.

The `list(APPEND plugin_libs ...)` line is used to add the `blink_light_plugin_node` library to the list of libraries that will be installed and exported by the package. I use this form to make it easier to add more libraries later on.

The `foreach(bt_plugin ...)` loop is used to add the dependencies to each plugin library and to define the `BT_PLUGIN_EXPORT` macro for each library. By using the list form, it will be easy to add more libraries later on.

The `install(TARGETS ...)` lines are used to install the libraries in the appropriate directories.

The `install(DIRECTORY ...)` lines are used to install the launch files in the appropriate directory.
The changes you will have to make to your launch file and to the `navigation.yaml` file will be covered shortly.
This assumes that your package has a `launch` directory.
If not, omit these lines from the `CMakeLists.txt` file.

The `ament_export_dependencies`, `ament_export_include_directories`, and `ament_export_libraries` lines are used to export the dependencies, include directories, and libraries of the package so that other packages can use them.

## The package.xml file

The `package.xml` file is used to specify the metadata of the package, such as the package name, version, description, and dependencies. Create a `package.xml` file in the `custom_behaviors` package directory with the following content:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_behaviors</name>
  <version>1.0.0</version>
  <description>My custom behavior trees</description>
  <maintainer email="my_email@foo.com">MyEmailName</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>behaviortree_cpp</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/behavior_plugin.xml" />
  </export>
</package>
```

This contents of this file is rather boiler plate.
You should read the ROS 2 documentation to understand the details of the file.

## Changing the navigation.yaml file

You will need to change a few lines in the `navigation.yaml` file to load the custom behavior plugin. I'll also discuss how to use your own behavior tree XML file.

Here is the `navigation.yaml` file with the changes needed to load the custom behavior plugin and to use your own behavior tree XML file:

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    ... Lines omitted here ---
    ### VVV ### The next line is one way to use a custom behavior tree XML file
    default_nav_to_pose_bt_xml: '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'

    ... More lines omitted here...
    ### VVV ### Here is where you specify the custom behavior plugin libraries.
    # For this example, we only have one library.
    plugin_lib_names:
    - blink_light_plugin_node
```

Under the `plugin_lib_names` key, you provide a list of the names of the libraries that you want to load. In this case, there is only one library, `blink_light_plugin_node`.
All the default libraries will be loaded as well, so you don't need to list them here.

The easy way to demonstrate the custom behavior is to use the `default_nav_to_pose_bt_xml` parameter to specify your own the XML file that contains the behavior tree.
For example, you should replace the path shown above with the path to your own XML file,
perhaps the one shown at the beginning of this chapter.

## Building the custom_behaviors package

You now need to build the `custom_behaviors` package.
You can do this by running the usual colcon build command:

```bash
colcon build --symlink-install
```

You don't need to used the ```---symlink-install``` option, but it is a good idea to use it.
Refer to the ROS 2 documentation for more information on the option.

## Running the custom behavior

Since you created a new package, you will need to source the setup file so that
***ROS 2*** will know about the new package.

```bash
source install/setup.bash
```

Then, run  your usual launch command to start the navigation stack.
If you've successfully built the `custom_behaviors` package, and modified the
`navigation.yaml` file used in your launch, the custom behavior should be loaded.
You can test that the custom behavior is working, assuming you are using the 
sample XML file from the beginning of this chapter, by doing the following:

* Bring up rviz2
* Click on the `Nav2 Goal` button.
* Click and on the map as if you were setting a destination goal for the robot.
* Look at the ROS log file. You should see the message "Blinking light" printed to the console three times.