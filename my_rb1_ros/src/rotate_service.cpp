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

#define __PI 3.14159265358979323846
#define __VELOCITY 0.5
#define __RATE 10.0
#define __ANGULAR_TOLERANCE_DEG 1.5

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
    ROS_INFO("/rotate_robot service: CALLED");
    ROS_INFO("   Requested rotation: %d degrees", req.degrees);
    ROS_INFO("    Angular tolerance: %.2f degrees", __ANGULAR_TOLERANCE_DEG);

    double req_rot = __deg2rad(req.degrees);

    double sec_start = ros::Time::now().toSec();
    __rotate(req.degrees);
    double sec_stop = ros::Time::now().toSec();

    if (abs(__actual_rotation - req_rot) < __angular_tolerance) {
      res.result = "/rotate_robot service: SUCCESS (actual rotation " +
                   std::to_string(__rad2deg(__actual_rotation)) + " degrees)";
      ROS_INFO("/rotate_robot service: SUCCESS");
    } else {
      res.result = "/rotate_robot service: FAILED (actual rotation " +
                   std::to_string(__rad2deg(__actual_rotation)) + " degrees)";
      ROS_INFO("/rotate_robot service: FAILED");
    }
    ROS_INFO("/rotate_robot service: performed in %.2f seconds",
             sec_stop - sec_start);
    ROS_INFO("      Actual rotation: %f degrees", __rad2deg(__actual_rotation));

    ROS_INFO("/rotate_robot service: FINISHED");
    ROS_INFO(" ");

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

    // Necessary code duplication to avoid inaccuracy
    // depending on the direction of rotation.
    // Notice the condition in the while loops.
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

  double __deg2rad(int deg) { return (double)deg * __PI / 180.0; }
  double __deg2rad(double deg) { return deg * __PI / 180.0; }
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

  // spinner has to be initilized BEFORE
  // the registration of the callbacks,
  // which happens in the constructor of
  // the RB1RotateService class, and
  // started after that. This allows the
  // service and topic callbacks to
  // interleave, providing fresh updates
  // of the robots yaw to the rotation
  // code.
  ros::AsyncSpinner spinner(2);

  RB1RotateService robotRotateService;

  spinner.start();

  ros::waitForShutdown();

  return 0;
}