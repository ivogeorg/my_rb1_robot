#include "my_rb1_ros/Rotate.h"
#include "ros/init.h"
#include "ros/spinner.h"
#include "tf/transform_datatypes.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <string>
#include <tf/tf.h>

#define __PI 3.14159265359
#define __VELOCITY 0.5
#define __RATE 10.0
#define __ANGULAR_TOLERANCE_DEG 1

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
  double __actual_rotation;

public:
  RB1RotateService()
      : __nh{}, __rate{__RATE}, __rot_svc{__nh.advertiseService(
                                    "/rotate_robot",
                                    &RB1RotateService::serviceCallback, this)},
        __vel_pub{__nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1)},
        __odom_sub{__nh.subscribe("odom", 1,
                                  &RB1RotateService::odometryCallback, this)},
        __angular_tolerance{__deg2rad(__ANGULAR_TOLERANCE_DEG)} {
    ROS_INFO("/rotate_robot service: READY");
  }

  ~RB1RotateService() {}

  // NOTE: Public functions in camelCase.
  bool serviceCallback(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {
    double yaw_start = __yaw_rad;
    ROS_INFO("/rotate_robot service: CALLED");
    ROS_INFO("          Current yaw: %f degrees", __rad2deg(yaw_start));
    ROS_INFO("   Requested rotation: %d degrees", req.degrees);
    ROS_INFO("    Angular tolerance: %d degrees", __ANGULAR_TOLERANCE_DEG);

    double req_rot = __deg2rad(req.degrees);

    double sec_start = ros::Time::now().toSec();
    __rotate(req.degrees);
    double sec_stop = ros::Time::now().toSec();
    ROS_INFO("/rotate_robot service: rotation performed in %f seconds", sec_stop - sec_start);

    double yaw_result = __yaw_rad;
    ROS_INFO("            Final yaw: %f degrees", __rad2deg(yaw_result));

    // Normalize requested degrees to handle angles above pi
    if (abs(yaw_result - yaw_start - __norm_angle(req_rot)) <
        __angular_tolerance) {
      res.result = "/rotate_robot service: SUCCESS (actual rotation " +
                   std::to_string(__rad2deg(__actual_rotation)) +
                   " degrees)";
      ROS_INFO("/rotate_robot service: SUCCESS");
    } else {
      res.result = "/rotate_robot service: FAILED (actual rotation " +
                   std::to_string(__rad2deg(__actual_rotation)) +
                   " degrees)";
      ROS_INFO("/rotate_robot service: FAILED");
    }
    ROS_INFO("          Current yaw: %f degrees", __rad2deg(yaw_result));
    ROS_INFO("   Requested rotation: %d degrees", req.degrees);
    // Reports actual rotation, excluding multiples of 360
    ROS_INFO("      Actual rotation: %f degrees",
             __rad2deg(yaw_result - yaw_start));

    ROS_INFO("/rotate_robot service: FINISHED");

    return true;
  }

  void odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
    __yaw_rad = tf::getYaw(odom->pose.pose.orientation);
  }

private:
  // NOTE: Private functions in __snake_case.
  void __rotate(int degrees) {
    ROS_INFO("/rotate_robot service: Working...");
    if (degrees > 0)
      __vel_msg.angular.z = __VELOCITY;
    else
      __vel_msg.angular.z = -__VELOCITY;

    double last_angle = __yaw_rad;
    double turn_angle = 0;
    double goal_angle = __deg2rad(degrees);

    if (goal_angle > 0) {
      while (ros::ok() &&
             (abs(turn_angle + __angular_tolerance) < abs(goal_angle))) {
        // Turn robot
        __vel_pub.publish(__vel_msg);
        __rate.sleep();

        double temp_yaw = __yaw_rad; // __yaw_rad may change!
        double delta_angle = __norm_angle(temp_yaw - last_angle);

        turn_angle += delta_angle;
        last_angle = temp_yaw;
      }
    } else {
      while (ros::ok() &&
             (abs(turn_angle - __angular_tolerance) < abs(goal_angle))) {
        // Turn robot
        __vel_pub.publish(__vel_msg);
        __rate.sleep();

        double temp_yaw = __yaw_rad; // __yaw_rad may change!
        double delta_angle = __norm_angle(temp_yaw - last_angle);

        turn_angle += delta_angle;
        last_angle = temp_yaw;
      }
    }

    // Stop robot
    __vel_msg.angular.z = 0.0;
    __vel_pub.publish(__vel_msg);

    __actual_rotation = turn_angle;
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

  ros::AsyncSpinner spinner(2);

  RB1RotateService robotRotateService;

  spinner.start();

  ros::waitForShutdown();

  return 0;
}