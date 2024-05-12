### checkpoint2

#### Overview

Creates a node providing a service `/rotate_robot` to rotate the RB1 robot in place a specified number of degrees clockwise or counteclockwise. The robot is rotated by publishing to the `/cmd_vel` topic. The service is called with a custom message, containing an integer request variable `degrees` and a boolean response variable `result`. The service is implemented in a package `my_rb1_ros`.

#### Submission notes

_Notes of interest to the reviewer!_

`cd ~/catkin_ws/src/my_rb1_robot`
`git clone https://github.com/ivogeorg/my_rb1_robot.git` (unnecessary if already working in my workspace)
`git checkout checkpoint2`
`roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch`
`roslaunch my_rb1_ros rotate_service.launch`
`rosservice call /rotate_robot "degrees: -90"`

1. The branch `checkpoint2` has not been merged.
2. The `#include` section of the source file [`src/rotate_service.cpp`](src/rotate_service_cpp) is messy. No attempt at minimality has been made.
3. When the service is launched, the service works per the requirements, but `TF_REPEATED_DATA` warnings are issued at a high rate. Removing them seems to require a more in-depth knowledge of the `tf` infrastructure than I posses right now.
4. The following constants have been `#define`d, for pi, the angular velocity of the robot, and the ROS rate (in Hz):
   ```c++
   #define __PI 3.14159265359
   #define __VELOCITY 0.25
   #define __RATE 10.0
   ```
5. The significant delay of this assignment was mainly due to spending a lot of time trying to wrap my head around the complexities and ambiguities of `tf`. The design is overly complex and the documentation is quite lacking. The relationship of `tf` and `tf2` is poorly described ("`tf2` is a _second generation_ implementation of `tf`" is just not enough). A much deeper dive would be necessary at some point.
6. I could not get the rotation to display along with the frame. The only way I could think of displaying the frame was to select the robot in translation mode. But then the visual seems to be frozen to the programmatic rotation and the service hangs. The moment I switch back to the selector mode, the robot shows in the end state of the rotation.

#### TODO: Items/features to modify and/or fix for re-evaluation

1. ~Instead of `TransformListener` use the `/odom` topic directly to rotate the robot. This will also remove the warning spam.~
2. Faster rotation (3 sec for 90 degrees, 6 sec for 180, 12 sec for 360).
3. ~`result` variable of custom message has to be `string`.~
4. Status messages like `/rotate_robot service: success/failed: +/- xx degrees`:
   1. `ROS_INFO("/rotate_robot service: Called: +/- %d degrees", req.degrees).
   2. `ROS_INFO("/rotate_robot service: Working...").
   3. `ROS_INFO("/rotate_robot service: Success: +/- %d degrees", degrees_after) + `res.result`.
   4. `ROS_INFO("/rotate_robot service: Failed: +/- %d degrees", req.degrees) + `res.result`.



#### Implementation notes

1. The node:
   1. Will be called `rotate_service_node`.
   2. Will advertise a service called `/rotate_robot`. It will be called from the command line, not through a programmatic client.
   3. ~Will subscribe to topic `/odom`.~ Does not have to subscribe to `/odom` because it gets the transform between the `/odom` and `base_footlink` frames from [`tf2_ros::Buffer::lookupTransform`](https://docs.ros.org/en/jade/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html#acabbd72cae8f49fb3b6ede3be7e34c55). The robot is rotated _in its own frame_, so this topic will be used to read its current position to rotate the robot the specified number of degrees. Note that the service request message contains _degrees_ but the actual calculation is in _radians_.
   4. Will publish to topic `/cmd_vel`. The topic takes `geometry_msgs::Twist` messages.
   5. Will work with a custom service message `Rotate.srv`.
      ```
      int32 degrees
      ---
      bool result
      ```
   6. The `CMakeLists.txt` and `package.xml` files in the [`my_cusom_srv_msg_pkg`](https://github.com/ivogeorg/my_custom_srv_msg_pkg) contain the right settings for the generation and usage of custom service messages.
      1. `CMakeLists.txt`:
         1. ```
            find_package(catkin REQUIRED COMPONENTS
                roscpp
                message_generation  # <--
            )
            ```
         2. ```
            add_service_files(  # <--
                FILES
                Rotate.srv  # <--
            )
            ```
         3. ```
            generate_messages(  # <--
                DEPENDENCIES
            )
            ```
         4. (possibly not strictly necessary if messages in the same package)
            ```
            catkin_package(
                CATKIN_DEPENDS roscpp
            )
            ```
         5. For package compilation:
            `add_executable` 
            `add_dependencies`
            `target_link_libraries` 
      2. `package.xml`:
         1. ```
            <package>
                <build_depend>message_generation</build_depend>  # <--
                <build_export_depend>message_runtime</build_export_depend>  # <--
                <exec_depend>message_runtime</exec_depend>  # <--
            </package>
            ```





   7. The Python file [`robot_control_class.py`](https://github.com/ivogeorg/robot_control/blob/441c9c5170ed0f31f4a457d1f1c2077638845141/robot_control_class.py#L37) contains good code to implement in [`rotate_service.cpp`](src/rotate_service.cpp). Note that `get_odom` assumes `base_link` to be the root, while the RB1 has `base_fooprint` as the root link. `get_odom` gets the transform between `/odom` and the root link, which gives `rotate` the ability to rotate from any starting position (i.e. in the frame of the robot). 
   8. The Python class mentioned uses some advanced `tf` functionality, specifically `waitForTransform` and `lookupTransform`. Resources for learning how to use them:
      1. `tf` and `Time` [tutorials](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29).
      2. [`tf2` time travel (C++)](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29).
      3. [`tf2` time travel (Python)](http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28Python%29).
      4. YouTube video [All you need to know about TF and TF2 in ROS | Tutorial](https://www.youtube.com/watch?v=_t4HZ8r_qFM) to understand which one to use. `tf2` is a new version of `tf` and supercedes it. In fact, `tf` is **explicitly deprecated** in favor of `tf2` and `tf2` is de facto the _implementation under the hood_. `tf2` or, actually, `tf2_ros` is likely easier to use to implement a tranform listener, possibly rendering unnecessary the geometric primitive manipulation overhead of the `PyKDL` library. `tf2` provides those manipulations. `tf2` is also strongly preferred over `tf` for starting out.
      5. [`tf2` starting page](https://wiki.ros.org/tf2). [`tf2` Tutorials](http://wiki.ros.org/tf2/Tutorials) with [Introduction](http://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2) and branches for C++ and Python.
      6. The `tf` and `tf_static` topics are implented by `tf2` and are of type `tf2_msgs/TFMessage`.
      7. The [`tf2` listener tutorial](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29) is pretty clear about the implementation that is needed.
         1. The first two arguments to `lookupTranform` are the _target_ and _source_ frames. In what order should `odom` and `base_footprint` be specified for the purpose of rotating the robot around its own frame? Two discussions on this issue:
            1. [Confusing target and source frames](https://answers.ros.org/question/296844/time-travel-with-tf-tutorial-confusing-target-and-source-frames/).
            2. [The problem with `lookupTranform`](https://answers.ros.org/question/194046/the-problem-of-transformerlookuptransform/).
         2. [`tf2_ros::Buffer::lookupTranform`](https://docs.ros.org/en/jade/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html#acabbd72cae8f49fb3b6ede3be7e34c55) returns a message [`geometry_msgs::TransformStamped`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html), which contains a [`geometry_msgs::Transform`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Transform.html), which contains a [`geometry_msgs::Vector3`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html) for translation and a [`geometry_msgs::Quaternion`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html) for rotation. The z-axis (aka body-z) rotation angle can be extracted from the latter by converting the quaternion to a [`tf2::Matrix3x3`](https://docs.ros.org/en/jade/api/tf2/html/classtf2_1_1Matrix3x3.html) and calling [`getRPY`](https://docs.ros.org/en/jade/api/tf2/html/classtf2_1_1Matrix3x3.html#aa3d893c7af63ad0496bcc1922daad108) on it ([example](https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac)).
            
   9. The following contain supporting code for working with frame transformations:
      1. The [`orocos_kdl`](https://www.orocos.org/kdl.html) library. 
         1. On [Github](https://github.com/orocos/orocos_kinematics_dynamics).
         2. Installation:
            1. Installation [manual](https://www.orocos.org/wiki/Installation_Manual.html). Note the dependency on [`eigen2`](https://eigen.tuxfamily.org/index.php?title=Main_Page).
            2. [INSTALL.md](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md) on Github. The dependency on Eigen states `libeigen3-dev`.
      2. The [`<cmath>`](https://cplusplus.com/reference/cmath/) library.
         1. The constant `M_PI` might or might not be defined. If not, use a `#define` with a sufficiently precise value.
      3. There is very little needed to manipulate the geometric primitives for the `tf` code, for which installing the KD library may be an overkill:
         1. The `tf` function `lookupTransform` fills a `Transform` object parameter reference with the resulting _translation_ and _rotation_. The latter is returned as either a 3x3 matrix or a quaternion, so has to be converted to the corresponding _yaw_ angle (in the robot's frame) to monitor the progress of the robot's rotation. The [functionality of the PyKDL library](https://docs.ros.org/en/diamondback/api/kdl/html/python/geometric_primitives.html) (which is just a Python binding for `orocos_kdl` (see above)) used on this [`line`](https://github.com/ivogeorg/robot_control/blob/441c9c5170ed0f31f4a457d1f1c2077638845141/robot_control_class.py#L225) can be implemented locally.
         2. The method [`lookupTransform`](https://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b).
         3. The reference for the [`Tranform`](https://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transform.html) class.
         4. The [`tranform_datatypes.h`](https://docs.ros.org/en/indigo/api/tf/html/c++/transform__datatypes_8h_source.html) header.
         5. `#include <tf.h>`.
   8. The file [`bb8_move_circle_class.cpp`](https://github.com/ivogeorg/my_cpp_class/blob/main/src/bb8_move_circle_class.cpp) file contains a decent template for a node class.

2. The file `rotate_service.cpp`:
   1. Will have a definition of class `RB1RotateService` and `main` function instantiating it.
   2. Will initialize the topic pub and sub in the constructor. Remember to have an explicit destructor.
   3. Will advertise the service in the constructor.
   4. May have a `get_odom` and `rotate` methods, depending on the functionality `tf2_ros` provides.

3. The file `rotate_service.launch` will launch the service.

4. The include `#include "my_rb1_ros/Rotate.h"` statement resolves to `/home/user/catkin_ws/devel/include/my_rb1_ros/Rotate.h`, where the custom service message infrastructure files are stored. 

5. ~**Question:** Will a _`tf2` broadcaster_ need to be started in `my_rb1_robot_warehouse.launch` so that a _`tf2` listener_ can be used to compute the transform from the world frame to the RB1 frame? Is the `robot_state_publisher` doing that already? Btw, how is broadcasting of the world frame started? Is it started by default?~
   1. After launch, `/robot_state_publisher` publishes `/tf` and `/tf_static`, both of type `tf2_msgs/TFMessage`.
   2. After launch, `/gazebo` publishes `/tf` and `/odom`, the latter of type `nav_msgs/Odometry`.

6. The service works but there are the following warnings all the time:
   ```
   Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame right_wheel at time 606.152000 according to authority unknown_publisher
         at line 278 in /tmp/binarydeb/ros-noetic-tf2-0.7.5/src/buffer_core.cpp
   ```
   There seems to be an ongoing discussion about this issue:
   1. Accepted answer [here](https://answers.ros.org/question/377796/tf_repeated_data-ignoring-data-with-redundant-timestamp-for-frame-link_left_wheel-at-time-618268000-according-to-authority-unknown_publisher/).
   2. [This](https://github.com/ros/geometry2/issues/414) issue in `ros/geometry2`.
   3. [This](https://github.com/ros/geometry2/issues/467) issue in `ros/geometry2`.
   4. [This](https://github.com/ros/geometry2/pull/475) pull request in `ros/geometry2`.

7. The syntax for the initialization of the class members in the constructor init lists is shown in the following:
   1. The [constructor](https://en.cppreference.com/w/cpp/language/constructor) article in the C++ reference. See the last example on the page.
   2. This tf2 [tutorial](https://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2::MessageFilter).

8. After the source file compiled there were linker messages about undeclared `tf2` entitites. Adding the missing package dependencies, as follows, fixed the issue:
   ```
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     tf2_ros
     tf
     tf2
     message_generation
   )
   ```
   Interestingly, no dependencies were added to `package.xml`. _Should they?_

9. If a service server node depends on the timely update of a topic it subscribes to, what are the options to get the timely update?
   1. [`ros::AsyncSpinner`](https://docs.ros.org/en/noetic/api/roscpp/html/classros_1_1AsyncSpinner.html), in our case with 2 threads.
   2. Multithreaded node with a `std::mutex` on the shared variable.
   3. Timer-based update, based on [`ros::Timer`](https://docs.ros.org/en/indigo/api/roscpp/html/classros_1_1Timer.html).