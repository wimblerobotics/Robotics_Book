
# The Structure of a URDF
<details open>
<summary>Explains the allowed elements  of a URDF and how they interrelate</summary>

Like all XML files, a URDF file is made up of elements.
These elements are the building blocks of the URDF file.
The main elements of a URDF file are ***links*** which represent the physical parts of the robot, and ***joints*** which represent the connections between the links.

The URDF must create a tree struction of links, that is one link is the _root_ of the tree with no other
links connected to it, and all other links are connected to the root link or to other links in the tree.
Every link in the tree has exactly one parent link, except for the root link which has no parent.
The think that connects the links in the tree are the joints.

A link describes all the physical properties of a rigid body, principly the visual aspects, such as its shape and color,
but also its mass, inertia, and collision properties. 
A joint describes the physical connection between two links, principly which link is the parent,
which is the child, how they are physically positioned relative to each other,
and how the child link can move relative to the parent link, if, indeed, any movement is allowed.

The URDF file creeates a tree structure, with the root element being the `robot` element.
The `robot` element contains all the other elements in the URDF file. The `robot` element is the root of the URDF tree.

A good start to a URDF file would be the following:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
</robot>
```
The elemenets inside the [`robot`] element are any number of the following elements:
- `link`  - A link element describes a rigid body with an inertia, visual, and collision properties.
- `joint` - A joint element describes a joint between two links.
- `material` - A material element describes the visual properties of a link.  
Materials can be defined globally and then referenced by name within a [`link`] element.
Also, ***gabebo*** elements have their own [`material`] elements and don't recognized the [`material`] element from the URDF.
- [`transmission`] - A transmission element describes the transmission of a joint.
- [`gazebo`] - A gazebo element describes the gazebo properties of a link or joint.  
Note that gazebo element is not actuall part of the URDF schema, but is used by the Gazebo simulator.
</details>
