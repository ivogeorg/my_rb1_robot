// TODO
// THIS INCLUDE SECTION IS A F**ING MESS
// No idea if the right thing is getting
// included or not.

#include "my_rb1_ros/Rotate.h"
#include <ros/ros.h>
#include "ros/duration.h"
#include "ros/subscriber.h"
#include <tf/tf.h>
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Scalar.h"
#include <tf/LinearMath/Matrix3x3.h>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

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

  // ROS Messages
  geometry_msgs::Twist __vel_msg;
  geometry_msgs::TransformStamped __tf_stamp;

  // TF2
  tf2_ros::Buffer __tf_buf;
  tf2_ros::TransformListener __tf_listen;

  // Other
  double __angular_tolerance;
  double __yaw_rad;

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
    __get_yaw_rad();
    // 2. rotate: rotate the number of degrees in req
    __rotate(req.degrees);
    res.result = true;
    ROS_INFO("ROS service to rotate RB1 %d degrees: FINISHED", req.degrees);
    return true;
  }

private:
  // NOTE: Private functions in __snake_case.
  void __get_yaw_rad() {
    try {
      __tf_stamp =
          __tf_buf.lookupTransform("odom", "base_footprint", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Quaternion q(
        __tf_stamp.transform.rotation.x,
        __tf_stamp.transform.rotation.y,
        __tf_stamp.transform.rotation.z,
        __tf_stamp.transform.rotation.w
        );
    tf::Matrix3x3 m3x3(q);
    double roll, pitch, yaw;
    m3x3.getRPY(roll, pitch, yaw);
    __yaw_rad = yaw;
  }

  void __rotate(int degrees) {
    double rad = __deg2rad(degrees);


    // TODO
  }

  double __deg2rad(int deg) { return (float)deg * __PI / 180.0; }

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

  ROS_INFO("TODO: Implement service node class RB1RotateService");

  return 0;
}