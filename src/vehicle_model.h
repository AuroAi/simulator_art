/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: 170169953038383a9a3e2f7094765477263d3270 $
 */

/**  \file

 Model speed and turn rate of the ART autonomous vehicle.

 \author Jack O'Quin

 */

#ifndef _VEHICLE_MODEL_H_
#define _VEHICLE_MODEL_H_ 1

#include <string>
#include <math.h>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

// libstage
#include <stage.hh>

// ROS interfaces
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <auro_vehicle_msgs/VehicleInfo.h>

// Corresponding ROS relative names
#define BRAKE_STATE "brake/cmd"
#define SHIFTER_STATE "shifter/cmd"
#define STEERING_STATE "steering/cmd"
#define THROTTLE_STATE "throttle/cmd"

struct state_info
{
  ros::Time sim_time;
  double steering_vel;
  double steering_angle;
  nav_msgs::Odometry odom;
  nav_msgs::Odometry groundTruth;
  sensor_msgs::Imu imu;
};
class ArtVehicleModel
{
public:
  // Constructor
  ArtVehicleModel(Stg::ModelPosition *stgPos, tf::TransformBroadcaster *tfBroad, std::string ns_prefix)
  {
    stgp_ = stgPos;                // Stage position model
    tf_ = tfBroad;                 // ROS transform broadcaster
    ns_prefix_ = ns_prefix;        // namespace prefix
    tf_prefix_ = ns_prefix + "/";  // transform ID prefix

    // servo control status
    brake_position_ = 1.0;
    //shifter_gear_ = art_msgs::Shifter::Drive;
    shifter_gear_ = 1;
    steering_angle_ = 0.0;
    throttle_position_ = 0.0;
    show_pose_ = false;
    ros::param::get("show_pose", show_pose_);
  }
  ~ArtVehicleModel(){};

  void update(ros::Time sim_time);  // update vehicle model
  void setup(void);                 // set up ROS topics
  void setup(std::string ns_prefix);

private:
  void ModelAcceleration(geometry_msgs::Twist *odomVel, sensor_msgs::Imu *imu_msg, ros::Time sim_time);
  void ackermannCmdControl(geometry_msgs::Twist *odomVel, sensor_msgs::Imu *imuMsg, ros::Time sim_time);

  // Stage interfaces
  Stg::ModelPosition *stgp_;

  // ROS interfaces
  ros::NodeHandle node_;          // simulation node handle
  tf::TransformBroadcaster *tf_;  // ROS transform broadcaster
  std::string ns_prefix_;         // vehicle namespace
  std::string tf_prefix_;         // transform ID prefix

  std_msgs::Float32 steer_angle_msg_,steer_vel_msg_;
  auro_vehicle_msgs::VehicleInfo veh_info_msg_;
  //nav_msgs::Odometry odomMsg_;
  ros::Publisher odom_pub_;
  ros::Publisher ground_truth_pub_;
  ros::Time last_update_time_;
  ros::Time last_acker_cmd_time_;

  ros::Publisher imu_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher utm_pub_;
  ros::Publisher steer_angle_pub_;
  ros::Publisher steer_vel_pub_;
  ros::Publisher veh_info_pub_;

  // servo device interfaces
  ros::Subscriber brake_sub_;
  ros::Subscriber shifter_sub_;
  ros::Subscriber steering_sub_;
  ros::Subscriber throttle_sub_;
  ros::Subscriber ackermann_sub_;

  // servo message callbacks
  void brakeReceived(const std_msgs::Int32::ConstPtr &msg);
  void shifterReceived(const std_msgs::Int8::ConstPtr &msg);
  void steeringReceived(const std_msgs::Float32::ConstPtr &msg);
  void throttleReceived(const std_msgs::Int32::ConstPtr &msg);
  void ackermannCmdReceived(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);

  // servo control status
  //
  // The mutex serializes access to these fields, which are set by
  // message callbacks running in a separate thread.
  boost::mutex msg_lock_;
  double brake_position_;
  uint8_t shifter_gear_;
  double steering_angle_;
  double steering_vel_;
  double throttle_position_;
  bool cmd_mode_ackermann;
  double ack_steering_angle, ack_speed, ack_acc, ack_steering_angle_velocity;
  double prev_speed_,prev_steering_angle_;
  void publishGPS(ros::Time sim_time,nav_msgs::Odometry odom);
  void publishUpdate(ros::Time time_latest);
  void publishSetUpdate(ros::Time sim_time,geometry_msgs::Twist twist,sensor_msgs::Imu imu);

  double origin_lat_;
  double origin_long_;
  double origin_elev_;
  double origin_easting_;
  double origin_northing_;
  char origin_zone_[20];
  double map_origin_x_;
  double map_origin_y_;
  double max_accel_;
  double max_decl_;
  double max_speed_;
  double weight_;
  double turning_radius_;
  double wheelbase_;
  double height_;
  double width_;
  double length_;
  double max_steer_degrees_;
  double max_steer_radians_;
  double max_steering_vel_;
  int delay_location_steps_;


  boost::circular_buffer<state_info> state_info_buffer_;


  bool show_pose_;
  bool broadcast_utm_odom_;
};

#endif  // _VEHICLE_MODEL_H_
