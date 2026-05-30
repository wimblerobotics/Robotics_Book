# Launch Files

<details open>
<summary>Understanding and Creating ROS 2 Launch Files</summary>
</details>

Everything about robots is hard, and launching robots is no exception. When your robot has grown beyond a single node printing "Hello World," you'll quickly discover that manually starting each node, configuring parameters, setting up namespaces, and managing dependencies becomes unwieldy. This is where **launch files** become essential—they orchestrate the startup of complex robot systems in a coordinated, repeatable way.

In this chapter, we'll begin to demystify launch files that will serve you well as your robot systems grow in complexity. By the end of this chapter, you'll start to understand not just how to write launch files, but how to organize them so you don't have to hunt through hundreds of lines to find the configuration you need to change.

## What Are Launch Files and Why Do We Need Them?

A **launch file** is a Python script that describes how to start up a collection of ROS 2 nodes with their associated parameters, topic remappings, and dependencies. Think of it as a recipe that tells ROS exactly how to bring your robot system to life.

Consider a typical mobile robot that might need:

- A motor driver node to control the wheels
- A LIDAR node to provide sensor data
- A camera node for vision
- An EKF (Extended Kalman Filter) node for sensor fusion
- Navigation nodes for path planning and obstacle avoidance
- A behavior tree node for high-level decision making

Starting each of these manually would require opening multiple terminals and typing commands like:

```bash
ros2 run motor_driver motor_driver_node
ros2 run ldlidar ldlidar_node --ros-args -p serial_port:=/dev/lidar_top
ros2 run robot_localization ekf_node --ros-args --params-file /path/to/ekf.yaml
# ... and so on
```

Not only is this tedious, but it's error-prone and doesn't handle dependencies or proper shutdown. Launch files solve these problems by:

1. **Starting multiple nodes simultaneously** with their proper configurations
2. **Managing dependencies** so nodes start in the correct order
3. **Handling parameter loading** from YAML files
4. **Setting up topic remappings** to connect nodes properly
5. **Managing namespaces** to avoid naming conflicts
6. **Coordinating shutdown** so all nodes exit cleanly together

## The Anatomy of a Launch File

Let's start with a simple example to understand the basic structure. Here's a minimal launch file that starts a single node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            output='screen'
        )
    ])
```

Every ROS 2 launch file must:

1. **Import necessary modules** from the `launch` and `launch_ros` packages
2. **Define a `generate_launch_description()` function** that returns a `LaunchDescription` object
3. **Add actions** (like `Node`) to the `LaunchDescription`

The `Node` action is the most common, representing a single ROS node to start. Key parameters include:

- `package`: The ROS package containing the executable
- `executable`: The name of the executable to run
- `name`: What you want to call the node (overrides the node's internal name)
- `output`: Where to send the node's output ('screen', 'log', or a file path)
- `parameters`: Configuration parameters for the node
- `remappings`: Topic/service name remappings

## Launch Arguments: Making Launch Files Configurable

Hard-coded values make launch files inflexible. **Launch arguments** allow you to parameterize your launch files, making them reusable in different scenarios. Here's how to add arguments:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments with default values
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Use the arguments in node configuration
    robot_node = Node(
        package='my_robot',
        executable='robot_controller',
        name='robot_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        robot_node
    ])
```

Now you can launch with different configurations:

```bash
# Use default values
ros2 launch my_robot robot.launch.py

# Override arguments
ros2 launch my_robot robot.launch.py use_sim_time:=true log_level:=debug
```

***Bonus hint***: you can use the `-s` or `--show-args` option with `ros2 launch` to see all available launch arguments for a given launch file:

```bash
ros2 launch description description.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'do_rviz':
        Launch RViz if true
        (default: 'true')

    'gui':
        Flag to enable joint_state_publisher_gui
        (default: 'False')

    'publish_joints':
        Launch joint_states_publisher if true
        (default: 'True')

    'urdf_file_name':
        URDF file name
        (default: 'sigyn.urdf.xacro')

    'use_sim_time':
        Use simulation (Gazebo) clock if true
        (default: 'false')
```

The pattern I follow (and recommend) is to **declare all launch arguments at the top of the file** with descriptive names and sensible defaults. This makes it immediately clear what options are available and keeps related configuration together.

Actually, my own code differs somewhat from this example in that I prefer to group all argument declarations at the top, then define nodes below and add the node to the `LaunchDescription` right after its definition, keeping related context together. We'll explore this organizational style in more detail later.

## Loading Parameters from YAML Files

For complex configurations, embedding parameters directly in launch files becomes unwieldy. YAML files provide a clean solution for managing configuration values outside of your launch files. This approach ensures that when you need to change a node's configuration, you only need to update one YAML file rather than hunting through multiple launch files that might start the same node.

ROS 2 nodes have a built-in parameter API, and the launch system provides excellent support for loading YAML files and passing their contents to nodes. See [Understanding ROS2 Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) for more information on parameters in ROS2.

Here's how to load parameters from a YAML file:

**nav2_params.yaml:**

```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    # ... more parameters

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    # ... more parameters
```

**Launch file:**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot')
    
    # Path to parameter file
    default_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file'
    )
    
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    return LaunchDescription([
        params_file_arg,
        controller_node
    ])
```

## Parameter Substitution and use_sim_time

One of the most important concepts in robotics launch files is the `use_sim_time` parameter. This boolean parameter tells ROS whether to use the system clock (real time) or simulation time from Gazebo. **Every time-sensitive node needs this parameter set correctly** or your robot will behave unpredictably.

The challenge is that you often want to set `use_sim_time` at launch time but have it propagate to all nodes that need it. Here's where **parameter substitution** becomes essential—it allows you to modify YAML parameter files at runtime before they're loaded by nodes.

### Understanding RewrittenYaml

The `RewrittenYaml` class is a powerful tool that reads your YAML parameter file, finds specific parameter values, and replaces them with runtime values from your launch configuration. **The key insight is that the values in your YAML file are just placeholders**—they get replaced with the actual runtime values you specify in the launch file.

Here's how it works:

**Your YAML file (nav2_params.yaml):**
```yaml
controller_server:
  ros__parameters:
    use_sim_time: false  # This is just a placeholder!
    controller_frequency: 20.0
    # ... other parameters

planner_server:
  ros__parameters:
    use_sim_time: false  # This will be replaced too
    planner_plugins: ["GridBased"]
    # ... other parameters
```

**Your launch file with parameter substitution:**
```python
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # Create parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time  # Replace ALL occurrences
    }
    
    # RewrittenYaml reads the file and replaces values
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=False
        ),
        allow_substs=True
    )
    
    # Nodes will now get the runtime value of use_sim_time
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[configured_params],
        output='screen'
    )
    
    return LaunchDescription([
        # ... argument declarations ...
        controller_node
    ])
```

### What Actually Happens

When you run your launch file:

1. **RewrittenYaml reads your original YAML file**
2. **Finds every occurrence of `use_sim_time`** in the file
3. **Replaces each occurrence** with the runtime value from `LaunchConfiguration('use_sim_time')`
4. **Creates a modified version** of the YAML content in memory
5. **Passes the modified content** to your nodes

So if you launch with `use_sim_time:=true`, **every** `use_sim_time: false` in your YAML file becomes `use_sim_time: true` when the nodes actually receive their parameters.

### Best Practices for YAML Placeholder Values

Since the YAML values get replaced anyway, what should you put in the original file? I recommend using the **most common value** for your setup. For example:

- If you primarily work with real robots, set `use_sim_time: false` in the YAML
- If you primarily work in simulation, set `use_sim_time: true` in the YAML

This serves as **documentation** for other developers—they can look at your YAML file and immediately understand what the typical configuration looks like, even though the actual runtime value might be different.

### Multiple Parameter Substitutions

You can substitute multiple parameters at once:

```python
param_substitutions = {
    'use_sim_time': LaunchConfiguration('use_sim_time'),
    'autostart': LaunchConfiguration('autostart'),
    'robot_base_frame': LaunchConfiguration('robot_base_frame')
}

configured_params = ParameterFile(
    RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=False
    ),
    allow_substs=True
)
```

This technique allows you to have a single YAML file that works for multiple scenarios—simulation vs. real robot, different robot configurations, different environments—with the specific values determined at launch time rather than hard-coded in the file.

### Why This Matters

Without parameter substitution, you'd need separate YAML files for every configuration:
- `nav2_params_real_robot.yaml`
- `nav2_params_simulation.yaml`
- `nav2_params_debug.yaml`

This creates maintenance nightmares when you need to update other parameters—you have to remember to update all the files. With `RewrittenYaml`, you maintain one master YAML file and let the launch system handle the runtime variations.

## Simulation vs Real Robot: Different Configurations

When building robots, you'll typically need different configurations for simulation versus real hardware. Here are the key differences to account for:

### What to Disable in Simulation

In simulation mode (`use_sim_time:=true`), you typically **don't want to launch**:

- **Motor driver nodes** - Gazebo provides its own motor control through gazebo_ros_control
- **Hardware sensor nodes** - Gazebo publishes simulated sensor data
- **Hardware-specific nodes** - Serial communication, GPIO controllers, etc.

### What's Different in Simulation

**Topic Mapping**: Gazebo publishes sensor data on different topics than your real hardware. Use **topic remappings** to bridge this gap:

```python
# Real robot might publish lidar on /scan
# Gazebo might publish on /my_robot/scan
# Remap to standardize
robot_node = Node(
    package='my_robot',
    executable='robot_controller',
    remappings=[
        ('/scan', '/my_robot/scan'),  # Map from expected to actual
        ('/cmd_vel', '/my_robot/cmd_vel')
    ],
    condition=IfCondition(use_sim_time)  # Only in simulation
)
```

**URDF Parameters**: Gazebo needs additional URDF elements for physics simulation:

```xml
<!-- Real robot URDF focuses on geometry and joints -->
<link name="base_link">
  <visual>
    <geometry>
      <mesh filename="robot_chassis.dae"/>
    </geometry>
    <material name="blue"/>
  </visual>
</link>

<!-- Gazebo URDF adds physics properties -->
<link name="base_link">
  <visual>
    <geometry>
      <mesh filename="robot_chassis.dae"/>
    </geometry>
    <material name="Gazebo/Blue"/>  <!-- Gazebo-specific materials -->
  </visual>
  <collision>  <!-- Required for physics -->
    <geometry>
      <mesh filename="robot_chassis_collision.dae"/>
    </geometry>
  </collision>
  <inertial>  <!-- Required for dynamics -->
    <mass value="10.0"/>
    <inertia ixx="0.4" iyy="0.4" izz="0.2" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<!-- Controller definitions for Gazebo -->
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
  </plugin>
</gazebo>
```

Here's how to handle these differences in your launch file:

```python
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Real robot hardware nodes
    motor_driver_node = Node(
        package='motor_driver',
        executable='motor_driver_node',
        condition=UnlessCondition(use_sim_time)  # Only on real robot
    )
    
    lidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        parameters=[{'serial_port': '/dev/lidar_top'}],
        condition=UnlessCondition(use_sim_time)  # Only on real robot
    )
    
    # Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world_file]}.items(),
        condition=IfCondition(use_sim_time)  # Only in simulation
    )
    
    # Nodes that run in both modes (with different parameters)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[
            {'use_sim_time': use_sim_time},  # Critical!
            config_file_path
        ]
    )
    
    return LaunchDescription([
        # ... arguments ...
        motor_driver_node,
        lidar_node,
        gazebo_launch,
        ekf_node
    ])
```

