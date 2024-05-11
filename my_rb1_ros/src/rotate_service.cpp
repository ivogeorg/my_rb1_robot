#include "my_rb1_ros/Rotate.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/tf.h>

#define __PI 3.14159265359
#define __VELOCITY 0.25
#define __RATE 10.0
#define __ANGULAR_TOLERANCE_DEG 2

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
  ros::Subscriber __odom_sub;

  // ROS Messages
  geometry_msgs::Twist __vel_msg;

  // Other
  double __angular_tolerance;
  double __yaw_rad;

public:
  RB1RotateService()
      : __nh{}, __rate{__RATE}, __rot_svc{__nh.advertiseService(
                                    "/rotate_robot",
                                    &RB1RotateService::serviceCallback, this)},
        __vel_pub{__nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1)},
        __odom_sub{__nh.subscribe("odom", 1, &RB1RotateService::odometryCallback, this)},
        __angular_tolerance{__deg2rad(__ANGULAR_TOLERANCE_DEG)} {
    ROS_INFO("/rotate_robot service: READY");
  }

  ~RB1RotateService() {}

  // NOTE: Public functions in camelCase.
  bool serviceCallback(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {
    double yaw_start = __yaw_rad;
    ROS_INFO("/rotate_robot service: CALLED");
    ROS_INFO("              Current yaw: %d degrees", (int) __rad2deg(yaw_start));
    ROS_INFO("       Requested rotation: %d degrees", req.degrees);
    ROS_INFO("        Angular tolerance: %d degrees", __ANGULAR_TOLERANCE_DEG);

    __rotate(req.degrees);

    double yaw_result = __yaw_rad;

    // TODO: report actual degrees turned
    if (abs(yaw_start + yaw_result) < __angular_tolerance) {
        res.result = "/rotate_robot service: SUCCESS"; // TODO
        ROS_INFO("/rotate_robot service: SUCCESS");
    } else {
        res.result = "/rotate_robot service: FAILED";  // TODO
        ROS_INFO("/rotate_robot service: FAILED");
    }
    ROS_INFO("              Current yaw: %d degrees", (int) __rad2deg(yaw_start));
    ROS_INFO("       Requested rotation: %d degrees", req.degrees);
    ROS_INFO("        Actual rotatedion: %d degrees", req.degrees);  // TODO

    ROS_INFO("/rotate_robot: FINISHED");

    return true;
  }

  void odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
    ROS_INFO("RB1 yaw updated by /odom: %f", odom->twist.twist.angular.z);
    __yaw_rad = odom->twist.twist.angular.z;
  }

private:
  // NOTE: Private functions in __snake_case.
  void __rotate(int degrees) {
    if (degrees > 0)
      __vel_msg.angular.z = __VELOCITY;
    else
      __vel_msg.angular.z = -__VELOCITY;

    double last_angle = __yaw_rad;
    double turn_angle = 0;
    double goal_angle = __deg2rad(degrees);

    while (ros::ok() &&
           (abs(turn_angle + __angular_tolerance) < abs(goal_angle))) {
      // Turn robot
      __vel_pub.publish(__vel_msg);
      ROS_INFO("/rotate_robot: Working...");
      __rate.sleep();

      double delta_angle = __norm_angle(__yaw_rad - last_angle);

      turn_angle += delta_angle;
      last_angle = __yaw_rad;
    }

    // Stop robot
    __vel_msg.angular.z = 0.0;
    __vel_pub.publish(__vel_msg);
  }

  double __deg2rad(int deg) { return (float)deg * __PI / 180.0; }
  double __rad2deg(double rad) { return rad * 180 / __PI; }

  double __norm_angle(double angle) {
    double res = angle;
    while (res > __PI)
      res -= 2.0 * __PI;
    while (res < -__PI)
      res += 2.0 * __PI;
    return res;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");

  RB1RotateService robotRotateService;

  ros::spin();

  return 0;
}