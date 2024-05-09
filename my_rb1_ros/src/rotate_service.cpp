#include "my_rb1_ros/Rotate.h"
#include "ros/duration.h"
#include "ros/subscriber.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // May be unnecessary!
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#define __PI 3.14159265359

class RB1RotateService {
private:
  // NOTE: Private member names start with double underscore ("__")
  // ROS Objects
  ros::NodeHandle __nh;
  ros::Rate __rate;

  // ROS Services
  ros::ServiceServer __rot_svc;

  // ROS Publishers
  ros::Publisher __vel_pub;

  // ROS Subscribers
  // NOTE: Not necessary to subscribe to /odom.
  //       tf2_ros::Buffer::lookupTransform() returns
  //       tf b/n the /odom and base_footprint frames.
  //   ros::Subscriber __odom_sub;

  // ROS Messages
  geometry_msgs::Twist __vel_msg;
  geometry_msgs::TransformStamped __tf_stamp;

  // TF2
  tf2_ros::Buffer __tf_buf;
  tf2_ros::TransformListener __tf_listen;

  // Other
  float32 __angular_tolerance;

public:
  RB1RotateService()
      : __rate(10.0),
        __rot_svc(__nh.advertiseService(
            "/rotate_robot", &RB1RotateService::serviceCallback, this)),
        __vel_pub(__nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1)),
        __tf_listen(__tf_buf), __angular_tolerance(__deg2rad(2)) {
    ROS_INFO("Service /rotate_robot: READY");
  }

  ~RB1RotateService() {}

  // NOTE: Public functions in camel_case.

  bool serviceCallback(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {
    // 1. get_odom: get tf from /odom to base_footprint
    __get_odom();
    // 2. rotate: rotate the number of degrees in req
    __rotate();
    res.result = true;
    ROS_INFO("ROS service to rotate RB1 %d degrees: FINISHED", req.degrees);
  }

private:
  // NOTE: Private functions in __snake_case.
  void __get_odom() {
    try {
      __tf_stamp =
          __tf_buf.lookupTransform('odom', 'base_footprint', ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  void __rotate(int32 degrees) {
    // TODO
    // 1. get yaw from transform rotation quaternion
    // 2. convert degrees parameter to radians
    float32 rad = __deg2rad(degrees);
    // 3. perform rotation to within angular_tolerance (in radians)
  }

  float32 __deg2rad(int32 deg) { return (float32) deg * __PI / 180.0; }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");

  ROS_INFO("TODO: Implement service node class RB1RotateService");

  return 0;
}