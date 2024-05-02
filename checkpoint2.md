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
                **message_generation**
            )
            ```
         2. ```
            **add_service_files**(
                FILES
                **Rotate.srv**
            )
            ```
         3. ```
            **generate_messages**(
                DEPENDENCIES
            )
            ```
         4. (possibly not strictly necessary if messages in same package)
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
                <build_depend>message_generation</build_depend>
                <build_export_depend>message_runtime</build_export_depend>
                <exec_depend>message_runtime</exec_depend>
            </package>
            ```





   7. The Python file [`robot_control_class.py`](https://github.com/ivogeorg/robot_control/blob/441c9c5170ed0f31f4a457d1f1c2077638845141/robot_control_class.py#L37) contains good code to implement in [`rotate_service.cpp`](src/rotate_service.cpp). Note that `get_odom` assumes `base_link` to be the root, while the RB1 has `base_fooprint` as the root link. `get_odom` gets the transform between `/odom` and the root link, which gives `rotate` the ability to rotate from any starting position (i.e. in the frame of the robot). 
   8. The Python class mentioned uses some advanced `tf` functionality, specifically `waitForTransform` and `lookupTransform`. Resources for learning how to use them:
      1. `tf` and `Time` [tutorials](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29).
      2. [`tf2` time travel (C++)](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29).
      3. [`tf2` time travel (Python)](http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28Python%29).
      4. YouTube video [All you need to know about TF and TF2 in ROS | Tutorial](https://www.youtube.com/watch?v=_t4HZ8r_qFM) to understand which one to use.
   8. The file [`bb8_move_circle_class.cpp`](https://github.com/ivogeorg/my_cpp_class/blob/main/src/bb8_move_circle_class.cpp) file contains a decent template for a node class.
2. The file `rotate_service.cpp`:
   1. Will have a definition of class `RB1RotateService` and `main` function instantiating it.
   2. Will initialize the topic pub and sub in the constructor. Remember to have an explicit destructor.
   3. Will advertise the service in the constructor.
   2. Will have a `get_odom` and `rotate` methods.
3. The file `rotate_service.launch` will launch the service.
4. The include `#include "my_rb1_ros/Rotate.h"` statement resolves to `/home/user/catkin_ws/devel/include/my_rb1_ros/Rotate.h`, where the custom service message infrastructure files are stored. 
