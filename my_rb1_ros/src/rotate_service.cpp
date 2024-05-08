#include "my_rb1_ros/Rotate.h"
#include "ros/subscriber.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // May be unnecessary!
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class RB1RotateService {
private:
  // ROS Objects
  ros::NodeHandle __nh;
  tf2_ros::Buffer __tf_buf;

  // ROS Services
  ros::ServiceServer __rot_svc;

  // ROS Publishers
  ros::Publisher __vel_pub;

  // ROS Subscribers
  ros::Subscriber __odom_sub;
  tf2_ros::TransformListener __tf_listen;

  // ROS Messages
  geometry_msgs::Twist __vel_msg;
  geometry_msgs::TransformStamped __tf_stamp;

public:
  RB1RotateService()
      : {__rot_svc(__nh.advertiseService(
             "/rotate_robot", &RB1RotateService::serviceCallback, this)),
         __vel_pub(__nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1)),
         __odom_sub(__nh.subscribe("/odom", 1000, )),
         __tf_listen(__tf_buf) {
    ROS_INFO("Service /rotate_robot: READY");
}

~RB1RotateService() {
}

bool serviceCallback(my_rb1_ros::Rotate::Request &req,
                     my_rb1_ros::Rotate::Response &res) {
  // TODO
}

void odometryCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
  // TODO
}

private:
void get_odom() {
  // TODO
}

void rotate(int32 degrees) {
  // TODO
}
}
;

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");

  ROS_INFO("TODO: Implement service node class RB1RotateService");

  return 0;
}