### checkpoint2

#### Overview

Creates a node providing a service `/rotate_robot` to rotate the RB1 robot in place a specified number of degrees clockwise or counteclockwise. The robot is rotated by publishing to the `/cmd_vel` topic. The service is called with a custom message, containing an integer request variable `degrees` and a boolean response variable `result`. The service is implemented in a package `my_rb1_ros`.

#### Implementation notes

1. The node:
   1. Will be called `rotate_service_call`.
   2. Will advertise a service called `/rotate_robot`. It will be called from the command line, not through a programmatic client.
   3. Will subscribe to topic `/odom`. The robot is rotated _in its own frame_, so this topic will be used to read its current position to rotate the robot the specified number of degrees. Note _degrees_ not _radians_.
   4. Will publish to topic `/cmd_vel`. The topic takes `geometry_msgs::Twist` messages.
   5. Will work with a custom service message `Rotate.srv`.
      ```
      int32 degrees
      ---
      bool result
      ```
   6. 

