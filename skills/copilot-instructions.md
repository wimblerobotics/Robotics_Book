# Copilot Agent Productivity Guide for Sigyn House Patroller

This guide enables AI agents to work productively in the Sigyn workspace. It summarizes architecture, workflows, conventions, and integration points unique to this codebase.

## 1. Colcon Build Instructions
Always include the command-line option `--symlink-install` when building the package to ensure that the agent can access the latest code changes without needing to rebuild the entire workspace.

```bash
colcon build --symlink-install
```

## 2. Target ROS 2 Distributions
- **Primary**: ROS 2 Jazzy Jalisco
- **Secondary**: ROS 2 Rolling
- Do NOT generate code or config targeting Humble or Iron unless explicitly requested.

## 3. Big Picture Architecture
- **sigyn_bringup** — Main bringup package. Contains `sigyn.launch.py`, SLAM launch files, navigation params, EKF config, and map files. Use `use_sim_time:=true` for simulation.
- **description** — Robot URDF files and Gazebo simulation models.
- **Designs** — Fusion 360, EasyEDA, and KiCad design artifacts.
- **Docker** — Dockerfiles for containerized environments.
- **Documentation** — Architecture diagrams, design documents. `AI/` subdirectory for AI-related docs.
- **experiments** — Experimental code for testing new features.
- **gripper** — Teensy 4.1 code for the gripper assembly.
- **ldlidar** — Customized LIDAR driver code.
- **min_max_curr_rviz_overlay** — RViz overlay for current readings.
- **oakd_detector** — OAK-D object/person detection code.
- **perimeter_roamer** — Perimeter patrol control code.
- **pi_servo1** — Teensy 4.1 code for servo control.
- **rviz** — RViz configuration files.
- **scripts** — Analysis and hardware utility scripts.
- **sigyn_behavior_trees** — Behavior tree implementation for decision-making.
- **sigyn_house_patroller** — Main house patroller: navigation, detection, notifications.
- **sigyn_interfaces** — Action and service interface definitions (.msg, .srv, .action).
- **sigyn_nav_goals** — Navigation goal definitions for patrol routes.
- **sigyn_to_elevator** — Elevator interface code.
- **sigyn_to_sensor** — Sensor interfaces (temperature, IMU, RoboClaw).
- **Teensy** — Design documents for custom Teensy 4.1 boards.
- **teensy_monitor** — Older Teensy monitoring code.
- **teensy_to_sigyn** — Teensy 4.1 main board code.
- **teleop_twist_keyboard** — Customized keyboard teleop package.
- **this_to_that** — ROS 2 message format converters.
- **twist_multiplexer** — Customized twist multiplexer for velocity priorities.
- **udev** — udev rules for device symlinks.
- **wall_finder** — Wall/obstacle detection using LIDAR data.
- **wifi_logger_visualizer** — Wi-Fi signal strength visualization.

## 4. Domain Skills Index

You have access to specialized domain knowledge files (skills) organized by category. **Do NOT answer advanced questions on these topics from memory alone.** If the user's request overlaps with any domain below, you MUST use the `read_file` tool to read the corresponding skill file BEFORE generating a response or writing code.

### ROS 2 Core (`skills/ros2_core/`)
| Skill File | Use When |
|---|---|
| `python_node_boilerplate.md` | Creating or refactoring `rclpy` nodes |
| `cpp_node_boilerplate.md` | Creating or refactoring `rclcpp` nodes |
| `lifecycle_nodes.md` | Implementing managed lifecycle nodes |
| `launch_files.md` | Writing or debugging Python launch files |
| `custom_interfaces.md` | Designing .msg, .srv, .action files |
| `qos_profiles.md` | Configuring Quality of Service settings |
| `tf2_transforms.md` | Working with TF2 transforms and frames |
| `parameter_handling.md` | Declaring/validating ROS 2 parameters |
| `action_server_client.md` | Implementing action servers or clients |
| `service_patterns.md` | Implementing service servers or clients |
| `topic_pub_sub.md` | Publisher/subscriber patterns and best practices |
| `component_composition.md` | Using composable nodes and containers |
| `executors_and_callbacks.md` | Executor types, callback groups, threading |
| `logging_and_diagnostics.md` | ROS 2 logging levels and diagnostics |
| `time_duration_rate.md` | Time, Duration, Rate, and clock handling |
| `rosbag2_recording.md` | Recording and replaying rosbag2 files |
| `message_filters.md` | Synchronizing multiple topic subscriptions |
| `ros2_testing.md` | Writing launch tests and unit tests |
| `intra_process_comms.md` | Zero-copy intra-process communication |
| `event_handlers.md` | Launch event handlers and process monitoring |

### Nav2 General (`skills/nav2_general/`)
| Skill File | Use When |
|---|---|
| `nav2_architecture_overview.md` | Understanding Nav2 system design and data flow |
| `nav2_lifecycle_management.md` | Managing Nav2 server lifecycles |
| `nav2_params_structure.md` | Understanding YAML parameter structure and namespacing |
| `velocity_smoother.md` | Configuring velocity smoothing |
| `collision_monitor.md` | Setting up collision avoidance zones |
| `waypoint_follower.md` | Multi-waypoint navigation |
| `nav2_simple_commander.md` | Using the Python API for Nav2 |
| `nav2_error_codes.md` | Interpreting Nav2 error/result codes |
| `behavior_server.md` | Configuring the behavior (recovery) server |
| `nav2_rviz_tools.md` | Using Nav2 RViz panels and tools |

### Nav2 Costmaps (`skills/nav2_costmaps/`)
| Skill File | Use When |
|---|---|
| `costmap_architecture.md` | Understanding costmap layered architecture and plugin ordering |
| `global_costmap_config.md` | Configuring the global costmap |
| `local_costmap_config.md` | Configuring the local (rolling window) costmap |
| `static_layer.md` | Configuring the static map layer |
| `obstacle_layer.md` | Configuring the 2D obstacle layer |
| `voxel_layer.md` | Configuring the 3D voxel obstacle layer |
| `inflation_layer.md` | Tuning inflation radius and cost scaling |
| `range_sensor_layer.md` | Integrating range sensors (VL53L0X, sonar, IR) |
| `denoise_layer.md` | Filtering salt-and-pepper costmap noise |
| `costmap_filters.md` | Using costmap filter infrastructure |
| `keepout_zones.md` | Defining no-go zones via map masks |
| `speed_restricted_zones.md` | Defining speed limit zones via map masks |

### Nav2 Planners (`skills/nav2_planners/`)
| Skill File | Use When |
|---|---|
| `smac_planner_2d.md` | Configuring the SMAC 2D A* planner |
| `smac_planner_hybrid_a_star.md` | Configuring the SMAC Hybrid-A* planner |
| `smac_lattice_planner.md` | Configuring the SMAC Lattice planner |
| `navfn_planner.md` | Configuring the NavFn (Dijkstra/A*) planner |
| `theta_star_planner.md` | Configuring the Theta* any-angle planner |
| `planner_comparison.md` | Choosing between available planners |
| `planner_benchmarking.md` | Measuring and comparing planner performance |
| `custom_planner_plugin.md` | Writing a custom planner plugin |

### Nav2 Controllers (`skills/nav2_controllers/`)
| Skill File | Use When |
|---|---|
| `mppi_controller.md` | Configuring MPPI controller parameters |
| `mppi_critics.md` | Understanding and tuning all MPPI critic plugins |
| `mppi_trajectory_visualization.md` | Visualizing MPPI trajectory rollouts |
| `dwb_controller.md` | Configuring DWB local planner parameters |
| `dwb_critics.md` | Understanding and tuning all DWB critic plugins |
| `regulated_pure_pursuit.md` | Configuring the RPP controller |
| `rotation_shim_controller.md` | Wrapping controllers with rotation shim |
| `controller_comparison.md` | Choosing between MPPI, DWB, and RPP |
| `custom_controller_plugin.md` | Writing a custom controller plugin |
| `controller_frequency_tuning.md` | Tuning controller loop rates and timing |
| `path_tracking_metrics.md` | Measuring path tracking quality |
| `graceful_controller.md` | Configuring the graceful motion controller |

### Nav2 Behaviors (`skills/nav2_behaviors/`)
| Skill File | Use When |
|---|---|
| `spin_behavior.md` | Configuring spin recovery behavior |
| `backup_behavior.md` | Configuring backup recovery behavior |
| `wait_behavior.md` | Configuring wait recovery behavior |
| `assisted_teleop.md` | Configuring assisted teleop behavior |
| `custom_behavior_plugin.md` | Writing a custom behavior plugin |

### Behavior Trees (`skills/behavior_trees/`)
| Skill File | Use When |
|---|---|
| `bt_xml_fundamentals.md` | Writing BehaviorTree.CPP v4 XML files |
| `bt_control_nodes.md` | Using Sequence, Fallback, Parallel, etc. |
| `bt_action_nodes_nav2.md` | Using Nav2 built-in action nodes |
| `bt_condition_nodes_nav2.md` | Using Nav2 built-in condition nodes |
| `bt_decorator_nodes.md` | Using decorator nodes (RateController, etc.) |
| `custom_bt_action_cpp.md` | Writing custom C++ BT action nodes |
| `custom_bt_condition_cpp.md` | Writing custom C++ BT condition nodes |
| `custom_bt_action_python.md` | Writing custom Python BT action nodes |
| `patrol_behavior_tree.md` | Designing patrol/waypoint-loop trees |
| `recovery_behavior_tree.md` | Designing recovery sub-trees |
| `multi_goal_navigation_bt.md` | Trees for navigating through multiple poses |
| `bt_blackboard_patterns.md` | Blackboard port patterns and data flow |
| `bt_subtree_composition.md` | Composing trees from reusable sub-trees |
| `groot2_integration.md` | Using Groot2 for visual BT editing |
| `bt_logging_and_replay.md` | Logging BT execution for debugging |

### SLAM & Mapping (`skills/slam_mapping/`)
| Skill File | Use When |
|---|---|
| `cartographer_tuning.md` | Tuning Google Cartographer parameters |
| `slam_toolbox_online.md` | Configuring SLAM Toolbox online mode |
| `slam_toolbox_lifelong.md` | Configuring SLAM Toolbox lifelong mode |
| `map_server_config.md` | Configuring the map server |
| `map_saver_config.md` | Saving maps from SLAM |
| `map_formats.md` | Understanding PGM, YAML, and serialized formats |
| `multi_floor_mapping.md` | Mapping multi-floor environments |
| `loop_closure.md` | Understanding and tuning loop closure |
| `map_merging.md` | Merging maps from multiple sessions |
| `mapping_best_practices.md` | General SLAM best practices |

### Localization (`skills/localization/`)
| Skill File | Use When |
|---|---|
| `amcl_tuning.md` | Tuning AMCL particle filter parameters |
| `ekf_sensor_fusion.md` | Configuring robot_localization EKF |
| `ukf_sensor_fusion.md` | Configuring robot_localization UKF |
| `imu_integration.md` | Integrating IMU data into the filter |
| `wheel_odometry_model.md` | Configuring wheel odometry models |
| `visual_odometry.md` | Integrating visual odometry sources |
| `lidar_odometry.md` | Integrating lidar-based odometry |
| `coordinate_frames_and_tf.md` | Understanding the full TF tree for navigation |
| `localization_recovery.md` | Recovering from localization failures |
| `multi_sensor_fusion.md` | Fusing multiple odometry sources |

### Perception (`skills/perception/`)
| Skill File | Use When |
|---|---|
| `laser_scan_processing.md` | Processing and filtering LaserScan data |
| `pointcloud_processing.md` | Processing PointCloud2 data |
| `depth_camera_pipeline.md` | Setting up depth camera processing |
| `yolo_ros2_integration.md` | Integrating YOLO object detection |
| `object_detection_pipeline.md` | End-to-end detection architecture |
| `person_tracking.md` | Tracking people across frames |
| `wall_line_extraction.md` | Extracting wall lines from laser scans |
| `image_transport_compressed.md` | Using image_transport for bandwidth |
| `camera_calibration.md` | Calibrating monocular and stereo cameras |
| `lidar_filtering.md` | Applying laser_filters to scan data |
| `spatial_ai_depthai.md` | Using DepthAI SDK and OAK-D pipelines |
| `anomaly_detection_vision.md` | Detecting visual anomalies in patrol |

### Hardware Integration (`skills/hardware/`)
| Skill File | Use When |
|---|---|
| `teensy_platformio.md` | Setting up Teensy 4.1 PlatformIO projects |
| `serial_communication_protocol.md` | Designing serial protocols between ROS and MCUs |
| `micro_ros_setup.md` | Running micro-ROS on microcontrollers |
| `encoder_odometry_math.md` | Computing odometry from wheel encoders |
| `motor_controller_interface.md` | Interfacing with motor controllers (RoboClaw, etc.) |
| `battery_monitoring.md` | ADC-based battery voltage monitoring |
| `servo_control.md` | PWM servo control patterns |
| `hardware_watchdog.md` | Implementing hardware safety watchdogs |
| `udev_rules.md` | Writing udev rules for persistent device names |
| `i2c_sensor_interface.md` | I2C sensor integration patterns |
| `imu_driver_config.md` | Configuring IMU drivers for ROS 2 |
| `lidar_driver_config.md` | Configuring LIDAR drivers for ROS 2 |
| `power_management.md` | Robot power distribution and management |
| `emergency_stop.md` | Implementing emergency stop systems |
| `hardware_abstraction_layer.md` | Abstracting hardware behind ROS interfaces |

### Simulation (`skills/simulation/`)
| Skill File | Use When |
|---|---|
| `gz_sim_setup.md` | Setting up Gazebo (Ignition/Ionic) simulation |
| `urdf_gz_plugins.md` | Adding Gazebo plugins to URDF/SDF |
| `simulated_sensors.md` | Configuring simulated LIDAR, camera, IMU |
| `world_building.md` | Creating Gazebo world files |
| `sim_time_management.md` | Managing use_sim_time across the stack |
| `gz_ros2_bridge.md` | Configuring the Gazebo-ROS 2 bridge |
| `physics_tuning.md` | Tuning physics engine parameters |
| `simulation_testing.md` | Using simulation for automated testing |

### URDF & Robot Description (`skills/urdf/`)
| Skill File | Use When |
|---|---|
| `urdf_fundamentals.md` | Writing URDF files from scratch |
| `xacro_macros.md` | Using xacro for modular URDF |
| `joint_types_and_limits.md` | Configuring joint types and limits |
| `inertia_calculation.md` | Computing correct inertia tensors |
| `collision_geometry.md` | Defining collision geometries |
| `sensor_frame_mounting.md` | Mounting sensor frames in URDF |
| `differential_drive_model.md` | Modeling differential drive kinematics |
| `robot_state_publisher.md` | Configuring robot_state_publisher |

### DevOps & Build (`skills/devops/`)
| Skill File | Use When |
|---|---|
| `colcon_workspace.md` | Managing colcon workspaces and builds |
| `package_xml_cmake.md` | Writing package.xml and CMakeLists.txt |
| `docker_ros2.md` | Dockerizing ROS 2 applications |
| `systemd_autostart.md` | Creating systemd services for robot boot |
| `github_actions_ros2.md` | CI/CD with GitHub Actions for ROS 2 |
| `rosdep_dependencies.md` | Managing dependencies with rosdep |
| `workspace_overlays.md` | Understanding and using workspace overlays |
| `cross_compilation.md` | Cross-compiling ROS 2 for ARM targets |

### Data & Visualization (`skills/data_visualization/`)
| Skill File | Use When |
|---|---|
| `rosbag2_analysis.md` | Analyzing recorded rosbag2 data |
| `battery_data_analysis.md` | Battery discharge curve analysis |
| `rviz2_config.md` | Creating and managing RViz2 configurations |
| `custom_rviz_overlay.md` | Writing custom RViz2 overlay panels |
| `performance_profiling.md` | Profiling ROS 2 node performance |
