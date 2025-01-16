# What Happens When You Navigate Using rviz2

## Using rviz2 to Move Your Robot to a Pose

When you fire up ***rviz2*** after having launched your robot and the ***nav2
stack***, you can cause your robot to move towards a pose that you establish
by clicking and dragging your mouse. You select the ***Nav2 Goal*** tab
in the ***rviz2*** menu, then click on the location you want your robot to
move to, and, while the mouse is still down, drag your mouse to establish the
direction you want your robot to point when it gets to that position.
You see the result as something like the thick, green arrow shown in
this partial screenshot of the ***rviz2*** window.

![Setting a navigation goal in rviz2](../media/setting_a_nav_goal.png)

From there, the various components of the ***nav2 stack*** will take over and
send movement commands to your robot to put it at the position and orientation
you chose.

So, what happens under the hood?

When you launch the ***nav2 stack***, one of the nodes launched is the
***bt_navigator*** node. This, among other things:

* Reads the ***navigation.yaml*** file (or whatever other file name you gave it in your 
  launch file) to find the names of navigators to be used, one for navigating to a 
  single pose, as is being described now, and another to be used to navigate to a 
  list of poses, like if you'd want to lay out a series of waypoints to be followed 
  along a complex path.

  ```code
  navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    ```

* Those two navigators are then loaded into memory and started as pieces of code.
  Each navigator is an ***action server*** (which is described elsewhere
  in this book).
  Any bit of ***ros 2*** code, such as ***rviz2*** can request that an action server perform the service it was
  designed for.
  This is, in fact, how ***rviz2*** will communicate between your
  mouse click and the ***nav2 stack***.

* Reads the ***navigation.yaml*** file to get the list of any custom
  ***behavior tree*** nodes (discussed elsewhere in this book) that you
  or someone else wrote, outside of the default set of ***behavior tree**
  nodes provided by the ***nav2 stack***. Those custom nodes are also loaded
  into memory so that they can extend the set of things you can do with
  a ***behavior tree*** beyond the default set provided.
  This next code snippet is used in one of my experiments for my robot to
  load in two custom ***behavior tree*** bits of code that I wrote;
  most people when they begin building a robot using ***ros 2*** will
  not have lines like this in their ***navigation.yaml*** file.

  ```code
      plugin_lib_names:
      - sigyn_say_something_plugin_node
      - sigyn_move_a_short_distance_ahead_plugin_node
    ```

When ***rviz2*** starts up, it forms a connection to the two navigators
just described. 
That is, if forms an ***action server client*** connection
(again, to be described later) to the two default navigators provided by
the ***nav2 stack***. 
Each of those navigators needs a ***behavior tree*** XML file to describe
the actions needed to actually move the robot and how to deal with
problems that come up while moving, and they also need the actual ***goal***
or list of goals that you gave, such as when you did the click and drag in
***rviz2*** to specify where the robot should move.

When you click and drag as above, the goal pose (position and orientation) is
captured and sent, via the ***action server client*** connection to the
appropriate navigator.

For our example, the default navigator is a module written in the C++ computer language 
with a class name of ***NavigateToPoseActionServer***.
Knowing the name of that class is how the ***navigation.yaml*** file tells the ***bt_navigator***
***ros 2*** node to find the needed code to run.
As with most things we are talking about here, this is provided by default
with the ***nav2 stack*** code that comes with the ***ros 2*** software package.

When it started up, ***NavigateToPoseActionServer*** also read the ***navigation.yaml*** file to find out
what behavior tree it should use to move the robot.
The following code snippet show how you would put lines in the ***navigation.yaml***
file to specify the ***behavior tree*** to be used.
Until you understand ***behavior trees*** well enough to create your own
custom one, you won't have any lines like this in your ***navigation.yaml*** file.

```code
    default_nav_to_pose_bt_xml: '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
```

When the ***NavigateToPoseActionServer*** gets called via the
***action server client*** and gets the associated ***goal*** (or list of goals)
you gave, it sets up the starting environment and begins interpreting those instructions for the given ***behavior tree***
to be followed .

The finer internal details of how this all works can be skipped for now so
we can concentrate on a general overview of what ***behavior trees*** are and
how they cause the robot to move.

## Introduction to Behavior Trees as Used in ros 2
For this discussion, a ***tree*** is a collection of ***nodes***
(think visually of boxes) and lines that connect boxes together.
The connections always go from a single box, called the ***parent node***,
to one or more boxes called ***child nodes***.
All the nodes are connected together this way such that there is only a single
node, called the ***root node***, which has no parent.
Hence, the whole thing looks like a tree which starts out as a single node which
branches out to one or more other nodes, and each of those nodes can branch
out to other nodes.
This also implies that there aren't any loops in the connection between the
node boxes, since any node can only have one and only one parent.
And, again, importantly, there is exactly one node that has no parent, and
that is the ***root node***.

![Default behavior tree for navigating to a pose](../media/default_bt.xml.txt)