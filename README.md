### my_rb1_robot

Repository for Checkpoint 1 of The Construct Robotics Masterclass. Creating a simple replica of the Robotnik RB-1 Base warehouse robot. The blueprint image below is copyright of Robotnik.

![RB-1 Base blueprint](assets/Robotnik-RB-1-BASE-Blueprints-web-2.jpg)


### Notes _and Questions_

1. `tf` in ROS and RViz refers to **t**ransform between coordinate **f**rames. There is a world frame and each robot has at least one frame of its own and multiple if it has multiple components. The [`tf`](http://wiki.ros.org/tf) package (deprecated and superceded by [`tf2`](http://wiki.ros.org/tf2)) keeps track of and maintains the relation (i.e. transform) between the multiple frames in a world. The frames can be visualized in RViz as well as in a tree view. The [tutorial](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) illustrates this.
2. The moments of inertia are expressed as the terms of [3D inertia tensors](https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors). Note that the 3x3 inertia matrix is symmetric, so only 6 numbers are specified (see below). Instead of precomputing them, they can be expressed with their formulas and declared parameters. However, see note below on URDF and Xacro.
   ```xml
   <inertial>
        <mass value="10" />
        <origin
            xyz="0 0 0.3"
            rpy="0 0 0" />
        <inertia
            ixx="1.5417"
            ixy="0"
            ixz="0"
            iyy="3.467"
            iyz="0"
            izz="4.742" />
   </inertial>
   ```
3. The `inertial` tag has an origin. _What is this the origin of and how does it relate to the joint and link origins?_
4. For the purposes of inertia:
   1. The robot is a cylinder.
   2. The two wheels are cylinders.
   3. The caster wheels are (approximately) spheres. (In actuallity, a caster wheel is a sphere and an off-center nested sphere.)
   4. The scanner is a cylinder.
5. The project asks for a URDF file `my_rb1_robot.urdf` and not an Xacro file. Xacro allows definition of properies with `<xacro:property name="chassis_mas" value="10"/>` which can then be used in formulae in, for example `<mass value="${2 * chassis_mas}"/>`. In a way, XACRO serves as a _preprocessor_ for URDF, among other things. This means that all numbers have to be hardcoded (though their computation can be documented in comments) and `gazebo` tags should be in the one URDF file.
6. This is the first time that a "fooprint" links is used apart from the base link. It is usually a reference for the rest of the robot links and is defined to be co-planar with the ground plane. It helps with the simulation of a lot of robot behavior. `base_footprint` is a projection of `base_link` on the ground plane (floor) and there may be a parent-child dependence between the two. _What tags does the `base_footprint` have as a link since it is just a projection?_ _Is there a joint between `base_footprint` and `base_link` and, if yes, what type?_
7. A [link](https://wiki.ros.org/urdf/XML/link) has 4 reference frames associated with it (origin (in the joint), inertial, collision, visual) and specified in the corresponding `<origin />` tags.
8. On the "link frame":
   1. The most basic frame of the link (that is, the robot component defined in a `<link>` tag). It is _basic_ in the sense that all subtags, in particular the `<origin>` subtags, are _relative to it_.
   2. There doesn't seem to be a way to define the origin of the link frame itself. For a link called, say, `base_link`, that defines a simple shape, when "Fixed Frame" in RViz is set to `base_link`, the shape appears in the middle of the plane centered at the shape's center of mass (CoM). This would imply that the the link frame initially and by default (that is, when `xyz="0 0 0"` and `rpy="0 0 0") has its origin at the CoM of the link shape. What happens when the shape is more complex, it remains to be seen.
   3. Setting `xyz="1 1 1"` in `<visual>` and `<colision>`, for example, makes the shape appear at a distance of 1 along each of the 3 axes from its frame. What is the utility of that is not clear. For proper inertial calculations, it seem that if the shape is moved like this away from the origin of its frame, the `<inertial>` origin should be modified the same way.
   4. Since the link frame cannot be set explicitly, the way to place a link "above ground" (that is, its bottom is level with the ground plane) is to put half of its height in the `z` dimension of `<collision>` (none for footprint) and `<visual>`. Since `<inertial>-<origin>` is also relative to the link frame, it has to be corrected to again coincide with the link shape's CoM.
9. Cannot have two links that are not connected directly or indirectly, showing that there can be only one link in a URDF file.
10. The `<origin>` in `<joint>` places the link frame of the child relative to the link frame of the parent! The `<origin>` tags in the link `<inertial>`, `<collision>`, and `<visual>` tags are unnecessary if the link doesn't have to translate or rotate relative to its own frame (i.e. child link frame). If `base_footprint` is elevated above the ground, it's elevated relative to its own link frame, so that height + half of its thickness have to be added to the joint `<origin>` `z` dimension when placing the origin of the child link frame (for example, half a height of a cylinder).
11. The root link does not have to be in any way the most "downward". It can be a spatially "central" component, to which the rest of the components are attached through joints, in all directions. To spawn "above the ground" specify an argument for the vertical offset for the spawner. 
12. Probably the most confusing and nonsensical feature of URDF is that the shape can be offset from its frame of reference (the link frame). If that is done, all 3 subtags (inertial, collision, visual) have matching offsets. This becomes hard to track and get right for a complex robot. The more natural approach is to only specify the `<offset>` in joints, which moves the link frame for the child relative to the parent link frame. If the parent link frame is also defaulted, no parent offsets have to be added to the joint `<origin>`. Importantly, the `<inertial>-<origin>` can be defaulted.
13. In a joint with an axis of rotation, the rotation is interpreted relative to the parent link frame!


### References

1. [tf](http://wiki.ros.org/tf) package.
2. [tf2](http://wiki.ros.org/tf2) package.
3. [tf tutorial](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf).
4. [Xacro](http://wiki.ros.org/xacro).
5. [`gazebo`](https://classic.gazebosim.org/tutorials?tut=ros_urdf) tag in URDF.
6. `base_footprint`:
   1. [What is the purpose of `base_footprint`?](https://answers.ros.org/question/208051/what-is-the-purpose-of-base_footprint/) on ROS Answers.
   2. [`base_link` to `base_footprint` transform?](https://answers.ros.org/question/12770/base_link-to-base_footprint-transform/) on ROS Answers.
   3. ROS [REP-120 # `base_footprint`](https://www.ros.org/reps/rep-0120.html#base-footprint) for a _humanoid_ robot.
7. [List of ROS enhancement proposals (REPs)](https://ros.org/reps/rep-0000.html). 
8. URDF [link](https://wiki.ros.org/urdf/XML/link).
