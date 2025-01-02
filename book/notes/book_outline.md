* What is ROS and why it is used to build robots.
  * Ros history <https://www.theconstruct.ai/history-ros/>
  * Ros 1 vs Ros 2
ROS is a set of software libraries and tools that help you build robot applications.
It provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management.
It is used in a wide range of applications, from small embedded systems to large-scale distributed systems. ROS is an open-source project that is maintained by the Open Source Robotics Foundation (OSRF).
It is released under the BSD license.
* What are the main components and concepts of ROS.
  * URDF
  * TF2 and frames
    * Frames
    * Transforms
    * base_link and base_footprint
    * odom
    * map
  * Time
    * Real time
    * Wall time
    * Simulated time
  * Topics
    * Quality of service
    * Custom messages
    * Latching
    * Remapping
    * Names
    * Namespaces
    * Wildcards
    * Global names
    * Local names
    * Private names
    * Relative names
    * Absolute names
    * Anonymous names
    * Static names
  * Packages
    * Package.xml
    * CMakeLists.txt
    * Manifest.xml
    * Package format
    * Package dependencies
    * Package exports
    * Package build types
    * Package build tools
    * Package build flags
    * Package build options
    * Package build targets
    * Package build scripts
    * Package build rules
    * Rosdep
  * Nodes
    * Node handles
    * Node names
    * Node namespaces
    * Node logging
    * Node parameters
    * Node services
    * Node actions
  * Actions
  * Services
  * Parameters
  * Launch files
    * Substitutions
  * Loggers
  * Bags
  * Rviz
  * Rqt
  * Gazebo
  * Rosdep

* How to install ROS and create a workspace.
* A bit about packages and nodes.
  * What is a package.
  * Overlays and Overlaying workspaces.
  * CMakelists.txt
  * package.xml
  * colcon
  * sourceing the workspace.
* Creating your first workspace and package.
* The URDF
  * A bit about frames and transforms.
  * How to create a basic URDF model.
* Visualizing the URDF model in Rviz.
* Visualizing the URDF model in Visual Studio Code.
  * Installing and using the URDF Visualizer extension.
* Launching the URDF model.
* Moving the joints of the URDF model.
* How to create a more complex URDF model.
* Visualizing the more complex URDF model in a Gazebo simulation.
* What are the main tags and attributes of the URDF.
* Sensors
  * Errors and uncertainties
  * Kalman filter
  * LIDAR
  * SONAR
  * Time of flight
  * Camera
  * IMU
  * GPS
  * Odometry
  * Wheel encoder
  * Force-torque sensor
  * Contact sensor
  * Range sensor
* Adding a LIDAR sensor to the URDF model.
* Adding a camera sensor to the URDF model.
* How to use the URDF in a real robot.
* How to switch between Gazebo simulation and the real robot.
* Useful packages in the ROS2 ecosystem.
  * Navigation2
  * MoveIt
  * Slam_toolbox
  * AMCL
  * Ros2_control
  * Micro-ROS
  * Lifecycle nodes
  * RMW
  * Motor drivers
  * Teleop
    * Keyboard
    * Joystick
  * RViz
  * RQT
  * Twist_mux


* How to create a package and a node.
  * Node topics
  * Node publishers
  * Node subscribers
  * Node clients
  * Node servers
  * Node timers
  * Node callbacks
  * Node spin
  * Node spinOnce
  * Node rate
  * Node sleep
  * Node shutdown
  * Node initialization

* How to create a publisher and a subscriber.
* How to create a service and a client.
* Launch files.
  * Debugging
  * Logging
  * Parameters
  * How to create a launch file.
    * How to create a launch file with a node.
    * How to create a launch file with a parameter.
    * How to create a launch file with a remap.
    * How to create a launch file with a namespace.
    * Node containers
    * Conditional execution
    * Substitution
    * Environment variables
* How to create a Gazebo simulation.
* How to create a Rviz visualization.
* How to record and play a bag file.
* How to use the ROS command line tools.
* How to use the ROS graphical tools.
* How to use the ROS documentation.
* How to use the ROS community.
* How to use the ROS ecosystem.
* How to use the ROS best practices.
* How to use the ROS development tools.
* How to use the ROS testing tools.
* How to use the ROS debugging tools.
* How to use the ROS simulation tools.
* How to use the ROS visualization tools.
* How to use the ROS optimization tools.
* How to use the ROS deployment tools.
* How to use the ROS monitoring tools.
* How to use the ROS security tools.
* How to use the ROS maintenance tools.
* How to use the ROS integration tools.
* How to use the ROS migration tools.
* Lifecycle nodes
* The ROS2 CLI
  * ros2
  * ros2 run
  * ros2 launch
  * ros2 node
  * ros2 topic
  * ros2 service
  * ros2 action
  * ros2 param
  * ros2 bag
  * ros2 log
  * ros2 param
  * ros2 daemon
  * ros2 doctor
  * ros2 component
  * ros2 extension
  * ros2 interface
  * ros2 pkg
  * ros2 multicast
  * ros2 lifecycle
  * ros2 security
* RMW
  * RMW implementations
  * cyclone
  * fastRTPS
  * connext
  * opensplice
  * microxrcedds
    *  
* Micro-ROS
* Sensors
* Actuators
* Motors
* Motor Drivers
* Ros2_control
  * Hardware interfaces
  * Joint state interface
  * Joint trajectory interface
  * Joint velocity interface
  * Joint effort interface
  * Joint position interface
  * Joint acceleration interface
  * Joint jerk interface
  * Joint torque interface
  * Joint impedance interface
  * Controllers
  * Joint state controller
  * Joint trajectory controller
  * Joint velocity controller
  * Joint effort controller
  * Joint position controller
  * Joint acceleration controller
  * Joint jerk controller
  * Joint torque controller
  * Joint impedance controller
* Nav2
* Slam_toolbox
* AMCL
* MoveIt
* Quaternion
* rclcpp
* rclpy
* rcljava
* rclnodejs
* rclcs
* rclrs
* 