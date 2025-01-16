Your launch file launches the nav2 stack with the following nodes:
* bt_navigator::bt_navigator
  This reads an amo file describing which navigator plugins to use.
  It includes a NavigateToPose plugin.
  It instantiates a NavigateToPoseActionServer.
  * Gathers
    * global_frame_
    * robot_frame_
    * transform_tolerance_
    * odom_topic_
  * get list list of predefined plugins
  * gets the list of user defined plugins
  * gets the odom smoother
  * gets the two default navigators:
    * navigate_to_pose
    * navigate_through_poses
  * gets the user defined navigators
  * Gets and configures the navigators
* The NavigateToPoseActionServer
  * Reads the behavior tree to use from the nav2_behavior_tree.xml file.
  *  listens for goals.
  *  call goalReceived, then startNavigating
  It sends the goal to the BT Navigator.
* rviz2 creates a "Nav2 Goal" plugin
  This plugin allows you to set a goal in the map frame.
  It sends the goal to the NavigateToPoseActionServer.
  The NavigateToPoseActionServer sets the goal into the blackboard.

  goalReceived calls initializeGoalPose(goal) which sets the transformed goal into the blackboard.

  The `onLoop()` function of the BT Navigator reads the goal from the blackboard
  gets the current pose from the blackboard,
  finds the closest point on the path to the goal,
  calculates the angle to that point, estimates the time and distance remaining for the feedback message,
* nav2_rviz_plugins/src 
  * [45] nav2_rviz_plugins/src/goal_tool.cpp onPoseSet(x, y, theta)
    * GoalUpdate.setGoal(x, y, theta, context->fetFixedFrame())
  * [949] onNewGoal(x, y, theta, frame)
    * Create a pose
    * If accumulating, then push into accumulated_poses
    * Else call startNavigation(pose)
  * [1300] startNavigation(pose)
    * Wait for navigate_to_pose_action_server or error message if not ready.
    * navigation_goal_.pose = pose
    * "NavigateToPose will be called using the BT Navigator's default behavior tree"
    * navigation_action_client->async_send_goal(navigation_goal_, send_goal_options)
* nav2_msgs::action::NavigateToPose