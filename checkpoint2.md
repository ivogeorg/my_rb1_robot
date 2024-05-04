### checkpoint2

#### Overview

Creates a node providing a service `/rotate_robot` to rotate the RB1 robot in place a specified number of degrees clockwise or counteclockwise. The robot is rotated by publishing to the `/cmd_vel` topic. The service is called with a custom message, containing an integer request variable `degrees` and a boolean response variable `result`. The service is implemented in a package `my_rb1_ros`.

#### Implementation notes

1. The node:
   1. Will be called `rotate_service_node`.
   2. Will advertise a service called `/rotate_robot`. It will be called from the command line, not through a programmatic client.
   3. Will subscribe to topic `/odom`. The robot is rotated _in its own frame_, so this topic will be used to read its current position to rotate the robot the specified number of degrees. Note _degrees_ not _radians_.
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

