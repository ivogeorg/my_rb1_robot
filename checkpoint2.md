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
   7. The Python file [`robot_control_class.py`](https://github.com/ivogeorg/robot_control/blob/441c9c5170ed0f31f4a457d1f1c2077638845141/robot_control_class.py#L37) contains good code to implement in [`rotate_service.cpp`](src/rotate_service.cpp).
2. The file `rotate_service.cpp`:
   1. Will have a definition of class `RB1RotateService` and `main` function instantiating it.
   2. Will initialize the topic pub and sub in the constructor. Remember to have an explicit destructor.
   3. Will advertise the service in the constructor.
   2. Will have a `get_odom` and `rotate` methods.
3. The file `rotate_service.launch` will launch the service.
