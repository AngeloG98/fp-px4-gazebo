/**
 * @file offb_node.hpp
 * @brief Head file for offb_node.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "quadrotor_msgs/PositionCommand.h"

void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode);
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
geometry_msgs::Point limit_velocity(float vx, float vy,float maximum);

ros::Subscriber state_sub;
ros::Subscriber planWaypointsub;
ros::Subscriber odometrysub;
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

mavros_msgs::State current_state;

bool planned_update_flag = false;
geometry_msgs::Point planned_point;
geometry_msgs::Point planned_velocity;
float planned_yaw_angle;
float planned_yaw_dot;

bool current_update_flag = false;
geometry_msgs::Point current_point;
geometry_msgs::Quaternion current_angle;
geometry_msgs::Vector3 curr_angle;
float current_yaw_angle;

float error_yaw_last = 0.0, error_yaw_integrated = 0.0;
geometry_msgs::Point error_pos_last, error_pos_integrated, attitude_expect, velocity_expected;

geometry_msgs::TwistStamped msgtwist;
geometry_msgs::Vector3 linear;
geometry_msgs::Vector3 angular;

geometry_msgs::PoseStamped msgpose;
geometry_msgs::Point pos;
geometry_msgs::Quaternion ori;

bool IFPLANNER = false;
const float deg2rad = 3.1415926535798/180.0;
const float rad2deg = 180.0/3.1415926535798;


//位置控制pid
const float P_pos = 1.5;
const float I_pos = 0.0;
const float D_pos = 0.0;

const float I_VEL_LIMIT = 0.025; //the intergrate item limit of position control, limit the expected velocity
const float D_VEL_LIMIT = 0.015;
const float YAW_I_LIMIT = 2.0;
const float YAW_RATE_LIMIT = 20.0;
const float control_alpha = 0.0;

bool dead_zone_flag = false;

const float P_yaw = 1.00;
const float I_yaw = 0.00;
const float D_yaw = 0.00;
const float P_z = 1.00;

#define DEAD_ZONE 0.01 //dead zone, unit: m
#define POS_I_DEAD_ZONE 0.04 //in dead zone, don't intergrate
#define YAW_DEAD_ZONE 2
#define HEIGHT_DEAD_ZONE 0.0001

template <typename T>
T limit(T a, T max)
{
  if(a > max)
    a = max;
  if(a < -max)
    a = -max;
  return a;
}