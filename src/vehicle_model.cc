/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: d5987f2902fa5c61d475348a3bf95e203b11fc84 $
 */

/**  \file

 Model speed and turn rate of the ART autonomous vehicle.

 \author Jack O'Quin

 */

#include <angles/angles.h>
#include <art/conversions.h>
#include <art/frames.h>
#include <art/UTM.h>
#include <art/steering.h>
#include "vehicle_model.h"

void ArtVehicleModel::setup(void)
{
  int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  odom_pub_ = node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "odom", qDepth);
  ground_truth_pub_ = node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "ground_truth", qDepth);
  imu_pub_ = node_.advertise<sensor_msgs::Imu>(ns_prefix_ + "imu", qDepth);
  gps_pub_ = node_.advertise<sensor_msgs::NavSatFix>(ns_prefix_ + "gps", qDepth);
  utm_pub_ = node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "utm", qDepth);
  steer_angle_pub_ = node_.advertise<std_msgs::Float32>(ns_prefix_ + "steer_angle", 1);
  steer_vel_pub_ = node_.advertise<std_msgs::Float32>(ns_prefix_ + "steer_vel", 1);

  ros::NodeHandle private_nh("~");
  private_nh.param("cmd_mode_ackermann", cmd_mode_ackermann, false);
  private_nh.param("latitude", origin_lat_, 29.446018);
  private_nh.param("longitude", origin_long_, -98.607024);
  private_nh.param("elevation", origin_elev_, 100.0);
  private_nh.param("broadcast_utm_odom", broadcast_utm_odom_, true);
  private_nh.param("max_accel", max_accel_, 1.2);
  private_nh.param("max_decl", max_decl_, 3.2);
  private_nh.param("max_speed", max_speed_, 5.0);
  private_nh.param("weight", weight_, 700.0);
  private_nh.param("turning_radius", turning_radius_, 4.6);//4.21999225977
  private_nh.param("wheel_base", wheelbase_, 2.59);
  private_nh.param("height", height_, 1.778);
  private_nh.param("width", width_, 1.397);
  private_nh.param("length", length_, 3.25);
  private_nh.param("max_steer_degrees", max_steer_degrees_, 29.0);
  private_nh.param("max_steer_radians", max_steer_radians_, 0.5061455);
  private_nh.param("max_steering_vel", max_steering_vel_, 0.6 );

  prev_speed_ = 0;
  ack_steering_angle = 0;
  ack_steering_angle_velocity = 0;
  ack_speed = 0;
  ack_acc = 0;
  if (cmd_mode_ackermann == false)
  {
    // servo state topics
    brake_sub_ = node_.subscribe(ns_prefix_ + "brake/state", qDepth, &ArtVehicleModel::brakeReceived, this, noDelay);
    shifter_sub_ = node_.subscribe(ns_prefix_ + "shifter/state", qDepth, &ArtVehicleModel::shifterReceived, this,
                                   noDelay);
    steering_sub_ = node_.subscribe(ns_prefix_ + "steering/state", qDepth, &ArtVehicleModel::steeringReceived, this,
                                    noDelay);
    throttle_sub_ = node_.subscribe(ns_prefix_ + "throttle/state", qDepth, &ArtVehicleModel::throttleReceived, this,
                                    noDelay);
  }
  else
  {
    ROS_INFO("cmd_mode_ackermann mode subscribing to  ackermann_cmd");
    ackermann_sub_ = node_.subscribe(ns_prefix_ + "ackermann_cmd", qDepth, &ArtVehicleModel::ackermannCmdReceived, this,
                                     noDelay);
  }
  // set default GPS origin, from SwRI site visit in San Antonio

  ROS_INFO("map GPS origin: latitude %.6f, longitude %.6f, elevation %.1f m", origin_lat_, origin_long_, origin_elev_);

  // Convert latitude and longitude of map origin to UTM.
  UTM::LLtoUTM(origin_lat_, origin_long_, origin_northing_, origin_easting_, origin_zone_);
  ROS_INFO("Map origin UTM: %s, %.2f, %.2f", origin_zone_, origin_easting_, origin_northing_);

  // Round UTM origin of map to nearest UTM grid intersection.
  double grid_x =0; //(rint(origin_easting_ / UTM::grid_size) * UTM::grid_size);
  double grid_y =0;// (rint(origin_northing_ / UTM::grid_size) * UTM::grid_size);
  ROS_INFO("UTM grid origin: (%.f, %.f)", grid_x, grid_y);

  // Report stage-generated odometry relative to that location.
  map_origin_x_ = origin_easting_ - grid_x;
  map_origin_y_ = origin_northing_ - grid_y;
  ROS_INFO("MapXY origin: (%.f, %.f)", map_origin_x_, map_origin_y_);
}

// Servo device interfaces.
//
// IMPORTANT: These callbacks run in a separate thread, so all class
// data updates must be done while holding the msg_lock_.
//
void ArtVehicleModel::brakeReceived(const std_msgs::Int32::ConstPtr &msg)
{
  //ROS_DEBUG("brake state received: position %.3f", msg->position);
  boost::mutex::scoped_lock lock(msg_lock_);
  brake_position_ = msg->data;
}

void ArtVehicleModel::shifterReceived(const std_msgs::Int8::ConstPtr &msg)
{
  ROS_DEBUG("shifter state received: gear %u", msg->data);
  boost::mutex::scoped_lock lock(msg_lock_);
  shifter_gear_ = msg->data;
}

void ArtVehicleModel::steeringReceived(const std_msgs::Float32::ConstPtr &msg)
{
  //ROS_DEBUG("steering state received: %.1f (degrees)", msg->data);
  boost::mutex::scoped_lock lock(msg_lock_);
  steering_angle_ = msg->data;
}

void ArtVehicleModel::throttleReceived(const std_msgs::Int32::ConstPtr &msg)
{
  //ROS_DEBUG("throttle state received: position %.3f", msg->data);
  boost::mutex::scoped_lock lock(msg_lock_);
  throttle_position_ = msg->data;
}
void ArtVehicleModel::ackermannCmdReceived(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(msg_lock_);
  ack_steering_angle = msg->drive.steering_angle;
  ack_steering_angle_velocity = msg->drive.steering_angle_velocity;
  ack_speed = msg->drive.speed;
  ack_acc = msg->drive.acceleration;
  last_acker_cmd_time_ = msg->header.stamp;
  //ROS_INFO_STREAM("ackermannCmdReceived:ack_speed"<<ack_speed<<" ack_steering_angle"<<ack_steering_angle<<" ack_acc"<<ack_acc<<" ack_steering_angle_velocity"<<ack_steering_angle_velocity);
}

void ArtVehicleModel::ackermannCmdControl(geometry_msgs::Twist *odomVel, sensor_msgs::Imu *imuMsg, ros::Time sim_time)
{

  // MUST serialize with updates from incoming messages
  boost::mutex::scoped_lock lock(msg_lock_);
  //float prev_speed = fabs(odomVel->linear.x);
  double delta_cmd_T = ros::Duration(sim_time - last_acker_cmd_time_).toSec();
  if(delta_cmd_T>1.0)
  {
    ack_speed=0;
    ack_steering_angle=0;
    ack_steering_angle_velocity=0;
    ack_acc=0;
    ROS_ERROR_STREAM_THROTTLE(1,"STAGE ArtVehicle: no ackermannCmd for last Model "<<delta_cmd_T<<" secs resetting commands");
  }
  //ROS_INFO_STREAM_THROTTLE(1,"STAGE ArtVehicle:  ackermannCmd  last  "<<delta_cmd_T<<" secs ");
  double max_angle = asinf(wheelbase_ / turning_radius_);

  // compute seconds since last update (probably zero first time)
  double deltaT = ros::Duration(sim_time - last_update_time_).toSec();

  double accel = (ack_speed - prev_speed_) / deltaT;
  double speed = ack_speed;
  //ROS_ERROR_STREAM("ack_speed "<<ack_speed<<" prev_speed "<<prev_speed_<<" deltaT " <<deltaT<<" accel "<<accel<<std::endl);
  //ROS_ERROR_STREAM("accel "<<accel<<" ack_acc "<<ack_acc<<" max_accel_ " <<max_accel_<<" max_decl_"<<max_decl_<<std::endl);

  if (ack_acc > 0)  //ackermann command acceleration limit is set
  {
    if (accel > 0)  //accelerating
    {
      accel = std::min(ack_acc, accel);
    }
    else //decelerating
    {
      accel = std::max(-ack_acc, accel);
    }

  }

  //ROS_ERROR_STREAM(" ackermann command limit check: accel "<<accel);

  if(speed>=0) //forward
  {
    if (accel >= 0) //accelerating
    {
      accel = std::min(max_accel_, accel);
    }
    else //decelerating
    {
      accel = std::max(-max_decl_, accel);
    }
  }else //backward
  {
    if (accel >= 0) //decelerating
    {
      accel = std::min(max_decl_, accel);
    }
    else //accelerating
    {
      accel = std::max(-max_accel_, accel);
    }
  }

  speed = prev_speed_ + accel * deltaT; //*0.999;              // adjust speed by set ack_acc limit
  //ROS_ERROR_STREAM(" Final accel "<<accel<<" speed "<<speed<<" prev_speed " <<prev_speed_<<std::endl);
  if (speed >= 0) //forward
  {
    speed = std::min(max_speed_, speed);
  }
  else //backward
  {
    speed = std::max(-max_speed_, speed);
  }
  prev_speed_ = speed;
  //ROS_ERROR_STREAM("speed "<<speed<<" art_msgs::ArtVehicle::max_speed " <<art_msgs::ArtVehicle::max_speed<<std::endl);





  if (ack_steering_angle > 0)
  {
    steering_angle_ = std::min(max_angle, ack_steering_angle);
  }
  else
  {
    steering_angle_ = std::max(-max_angle, ack_steering_angle);
  }

  double steering_vel = (steering_angle_ - prev_steering_angle_) / deltaT;

  if (ack_steering_angle_velocity > 0)  //ackermann command ack_steering_angle_velocity limit is set
    {
      if (steering_vel > 0)
      {
        steering_vel = std::min(ack_steering_angle_velocity, steering_vel);
      }
      else
      {
        steering_vel = std::max(-ack_steering_angle_velocity, steering_vel);
      }

    }

    //ROS_ERROR_STREAM(" ack_steering_angle_velocity command limit check: steering_vel "<<steering_vel);

    if (steering_vel > 0)
    {
      steering_vel = std::min(max_steering_vel_, steering_vel);
    }
    else
    {
      steering_vel = std::max(-max_steering_vel_, steering_vel);
    }

  steering_angle_ = prev_steering_angle_ + steering_vel * deltaT;
  prev_steering_angle_ = steering_angle_;

  // set simulated vehicle velocity using the "car" steering model,
  // which uses steering angle in radians instead of yaw rate.
  // set yaw rate (radians/second) from velocity and steering angle
  odomVel->linear.x = speed;
  odomVel->angular.z = speed * tan(steering_angle_) / wheelbase_;
  imuMsg->linear_acceleration.x = accel;
  imuMsg->angular_velocity.z = odomVel->angular.z;

  //publish steering data
  steer_angle_msg_.data=steering_angle_;
  steer_angle_pub_.publish(steer_angle_msg_);
  steer_vel_msg_.data=fabs(steering_vel);
  steer_vel_pub_.publish(steer_vel_msg_);



  ROS_DEBUG("Stage SetSpeed(%.3f, %.3f, %.3f)", odomVel->linear.x, odomVel->linear.y, ack_steering_angle);
  stgp_->SetSpeed(odomVel->linear.x, odomVel->linear.y, ack_steering_angle);

  //Int Stage position modle implementation for  drive_mode = DRIVE_CAR
  //vel.x = goal.x * cos(goal.a);
  //vel.y = 0;
  //vel.a = goal.x * sin(goal.a)/wheelbase;

}
/** Model vehicle acceleration
 *
 *  @pre last_update_time_ = time of previous update.
 *  @param odomVel[out] -> the Odometry message Twist component
 *  @param imuMsg[out] -> the IMU message
 *  @param sim_time[in] current simulation time
 *
 *  @note This simple model does not account for sideways slippage, so
 *        odomVel->linear.y is always zero.  Similarly, roll, pitch
 *        and odomVel->linear.z are always zero, because Stage is a 2D
 *        simulation.
 *
 *  @todo introduce some small random fluctuations
 */

void ArtVehicleModel::ModelAcceleration(geometry_msgs::Twist *odomVel, sensor_msgs::Imu *imuMsg, ros::Time sim_time)
{
  // MUST serialize with updates from incoming messages
  boost::mutex::scoped_lock lock(msg_lock_);

  double speed = fabs(odomVel->linear.x);

  // assume full brake or throttle produces 1g (9.81 m/s/s)
  // TODO: tune these coefficients using actual vehicle measurements
  static const double g = 9.81;         // acceleration due to gravity
  static const double throttle_accel = g;
  static const double brake_decel = g;
  static const double rolling_resistance = 0.01 * g;
  static const double drag_coeff = 0.01;

  // the vehicle idles at 7 MPH (3.1 m/s) with no brake or throttle
  //static const double idle_accel = (rolling_resistance
  //                                  + drag_coeff * 3.1 * 3.1);
  //static const double idle_accel = 0;  //Golf cart actually 0
 // double wind_resistance = drag_coeff * speed * speed;

  //double accel = (idle_accel + throttle_position_ * throttle_accel - brake_position_ * brake_decel - rolling_resistance - wind_resistance);

  double accel = ( throttle_position_ * throttle_accel - brake_position_ * brake_decel - rolling_resistance );
  // compute seconds since last update (probably zero first time)
  double deltaT = ros::Duration(sim_time - last_update_time_).toSec();
  speed += accel * deltaT;              // adjust speed
  imuMsg->linear_acceleration.x = accel;

  // Brake and throttle (by themselves) never cause reverse motion.
  // Only shifting into Reverse can do that.
  if (speed < 0.0)
  {
    speed = 0.0;
    imuMsg->linear_acceleration.x = 0.0;
  }

  // Set velocity sign based on gear.
  odomVel->linear.x = speed;            // forward movement
  if (shifter_gear_ == -1)
    odomVel->linear.x = -speed;         // reverse movement

  // set yaw rate (radians/second) from velocity and steering angle
  odomVel->angular.z = Steering::angle_to_yaw(odomVel->linear.x, steering_angle_);
  imuMsg->angular_velocity.z = odomVel->angular.z;

  // set simulated vehicle velocity using the "car" steering model,
  // which uses steering angle in radians instead of yaw rate.
  double angleRadians = angles::from_degrees(steering_angle_);
  ROS_DEBUG("Stage SetSpeed(%.3f, %.3f, %.3f)", odomVel->linear.x, odomVel->linear.y, angleRadians);
  stgp_->SetSpeed(odomVel->linear.x, odomVel->linear.y, angleRadians);

}

// update vehicle dynamics model
void ArtVehicleModel::update(ros::Time sim_time)
{
  sensor_msgs::Imu imu_msg;

  // model vehicle acceleration from servo actuators
  if (cmd_mode_ackermann == true)
  {
    ackermannCmdControl(&odomMsg_.twist.twist, &imu_msg, sim_time);
  }
  else
  {
    ModelAcceleration(&odomMsg_.twist.twist, &imu_msg, sim_time);
  }

  // Get latest position data from Stage
  // Translate into ROS message format and publish
  odomMsg_.pose.pose.position.x = stgp_->est_pose.x;//stgp_->est_pose.x + map_origin_x_;
  odomMsg_.pose.pose.position.y = stgp_->est_pose.y;//stgp_->est_pose.y + map_origin_y_;
  odomMsg_.pose.pose.position.z = 0;//origin_elev_;
  odomMsg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(stgp_->est_pose.a);

  odomMsg_.header.stamp = sim_time;
  odomMsg_.header.frame_id = "odom";
  odomMsg_.child_frame_id = "base_link";
  odom_pub_.publish(odomMsg_);

  // publish simulated IMU data
  imu_msg.header.stamp = sim_time;
  imu_msg.header.frame_id = "base_link";
  imu_msg.orientation = odomMsg_.pose.pose.orientation;
  imu_pub_.publish(imu_msg);

  // broadcast /earth transform relative to sea level
  tf::Quaternion vehicleQ;
  tf::quaternionMsgToTF(odomMsg_.pose.pose.orientation, vehicleQ);
  //ROS_INFO(" sendTransform txEarth");
  //tf::Transform txEarth(
  //    vehicleQ, tf::Point(odomMsg_.pose.pose.position.x, odomMsg_.pose.pose.position.y, odomMsg_.pose.pose.position.z));
  tf::Transform txEarth(
      vehicleQ, tf::Point( odomMsg_.pose.pose.position.x  , odomMsg_.pose.pose.position.y ,0));
  //    vehicleQ, tf::Point(odomMsg_.pose.pose.position.x- map_origin_x_ + origin_easting_, odomMsg_.pose.pose.position.y - map_origin_y_ + origin_northing_, 0));

  tf_->sendTransform(
      tf::StampedTransform(txEarth, sim_time, "odom", "base_link"));

  // Also publish /odom frame with same elevation as /vehicle and same
  // orientation as /earth
 tf::Transform txOdom(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Point( origin_easting_,origin_northing_, 0));
  //ROS_INFO(" sendTransform txOdom earth");
  if(broadcast_utm_odom_)
    tf_->sendTransform(tf::StampedTransform(txOdom, sim_time, "utm", "odom"));

  // Also publish the ground truth pose and velocity, correcting for
  // Stage's screwed-up coord system.
  Stg::Pose gpose = stgp_->GetGlobalPose();
  if(show_pose_) ROS_INFO_STREAM("Robot Global Position: ("<<gpose.x<<", "<<gpose.y<<")");
  Stg::Velocity gvel = stgp_->GetVelocity();
  tf::Quaternion gposeQ;
  gposeQ.setRPY(0.0, 0.0, gpose.a - M_PI / 2.0);
  tf::Transform gt(gposeQ, tf::Point(gpose.y, -gpose.x, 0.0));
  tf::Quaternion gvelQ;
  gvelQ.setRPY(0.0, 0.0, gvel.a - M_PI / 2.0);
  tf::Transform gv(gvelQ, tf::Point(gvel.y, -gvel.x, 0.0));

  groundTruthMsg_.pose.pose.position.x = gt.getOrigin().x();
  groundTruthMsg_.pose.pose.position.y = gt.getOrigin().y();
  groundTruthMsg_.pose.pose.position.z = gt.getOrigin().z();
  groundTruthMsg_.pose.pose.orientation.x = gt.getRotation().x();
  groundTruthMsg_.pose.pose.orientation.y = gt.getRotation().y();
  groundTruthMsg_.pose.pose.orientation.z = gt.getRotation().z();
  groundTruthMsg_.pose.pose.orientation.w = gt.getRotation().w();
  groundTruthMsg_.twist.twist.linear.x = gv.getOrigin().x();
  groundTruthMsg_.twist.twist.linear.y = gv.getOrigin().y();
  groundTruthMsg_.twist.twist.angular.z = gvel.a;
  groundTruthMsg_.header.stamp = sim_time;
  groundTruthMsg_.header.frame_id = "odom_truth";
  groundTruthMsg_.child_frame_id = "base_link";
  ground_truth_pub_.publish(groundTruthMsg_);

  publishGPS(sim_time);

  last_update_time_ = sim_time;
}

void ArtVehicleModel::publishGPS(ros::Time sim_time)
{
  nav_msgs::Odometry utm_odom_msg;
  sensor_msgs::NavSatFix gps_msgs;
  double utm_northing = odomMsg_.pose.pose.position.x  + origin_northing_;
  double utm_easting = odomMsg_.pose.pose.position.y  + origin_easting_;

  utm_odom_msg.header.stamp = sim_time;
  utm_odom_msg.header.frame_id =  "utm";
  utm_odom_msg.child_frame_id = "base_link";
  utm_odom_msg.pose.pose.position.x = utm_easting;
  utm_odom_msg.pose.pose.position.y = utm_northing;

  gps_msgs.header.stamp = sim_time;

  UTM::UTMtoLL(utm_northing, utm_easting, origin_zone_, gps_msgs.latitude, gps_msgs.longitude);


  utm_pub_.publish(utm_odom_msg);
  gps_pub_.publish(gps_msgs);

}
