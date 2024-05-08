#include "my_rb1_ros/Rotate.h"
#include "ros/subscriber.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // May be unnecessary!
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#define __PI 3.14159265359

class RB1RotateService {
private:
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
  float __angular_tolerance;

public:
  RB1RotateService()
      : __rate(10.0),
        __rot_svc(__nh.advertiseService(
            "/rotate_robot", &RB1RotateService::serviceCallback, this)),
        __vel_pub(__nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1)),
        __tf_listen(__tf_buf), __angular_tolerance(deg_to_rad(2)) {
    ROS_INFO("Service /rotate_robot: READY");
  }

  ~RB1RotateService() {}

  // NOTE: Public functions in camel_case.

  bool serviceCallback(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {
    // TODO
  }

private:
  // NOTE: Private functions in snake_case.
  void get_odom() {
    // TODO: It's not void
  }

  void rotate(int32 degrees) {
    // TODO
  }

  float deg_to_rad(float deg) { return deg * __PI / 180.0; }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");

  ROS_INFO("TODO: Implement service node class RB1RotateService");

  return 0;
}