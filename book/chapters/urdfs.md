# URDFS or Uniform Resource Definition Files

## Setup

If you haven't done so yet, you need to create a ROS 2 workspace to hold the book code.

```bash
# Do the following if you haven't already created a workspace
mkdir -p ~/wr_book_ws/src && cd ~/wr_book_ws/src # Change this if you need a different name or location
git clone git@github.com:wimblerobotics/Robotics_Book.git
cd .. # Should be back at ~/wr_book_ws
colcon build --symlink-install
source install/setup.bash
```

## What is a URDF?

A URDF is a text file that describes a robot's pyhsical structure. It can be used, for instance, to create a simulated image of what the robot looks like. It does more than just that, such a provide kinematic properties so that a full, physics-base simulator can simulate the robot acting and moving in a simulated world.

For now, think of a URDF as a list of parts, called links and a list of descriptions about how those links are connected, called joints. In this section, we will create a URDF that describes a two wheeled robot with a square body, a caster wheel, and a LIDAR sensor. We will use the ROS tool called rviz2 to visualize the robot. We will do this in a few steps, starting with simple shapes and then adding more complexity and eventually add in the use of a macro processor to simpllfy how you write the description.

## Step 1: Creating your first URDF file

A URDF file is an XML file. You can create one using a text editor. Let's begin with this simple URDF file which is in the [1.urdf](../../description/urdf/1.urdf) file.

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.25"/>
      </geometry>
      <pose>0 0 0.125 0 0 0</pose>
    </visual>
  </link>
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 -0.29 -0.13" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 0.29 -0.13" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

</robot>
  ```
Here is what the UDR file looks like using the rviz2 tool:

![alt text](../media/1.png)*

To see the visualization yourself, you can run the following command:

```bash
cd ~/wr_book_ws # Change this if you have a different workspace name
colcon build --symlink-install
source install/setup.bash
ros2 launch description description.launch.py urdf_file_name:="1.urdf"
```

## Explaining 1.urdf
