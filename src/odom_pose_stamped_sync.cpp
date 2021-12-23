// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include "mocap_vislam_sync/odom_pose_stamped_sync.hpp"

#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

namespace sync_topics {

OdomPoseStampedSync::OdomPoseStampedSync(ros::NodeHandle *nh) {
  nh_= *nh;
  std::string output_position_drift_topic;
  nh_.getParam("in_pose_stamped_topic", in_pose_stamped_topic_);
  nh_.getParam("in_odometry_topic", in_odometry_topic_);
  nh_.getParam("output_path", output_file_path_);
  nh_.getParam("output_position_drift_topic", output_position_drift_topic);
  start_new_estimation_ = false;
  is_estimating_ = false;
  node_name_ = "odom_pose_stamped_sync";

  // Create output publisher
  output_position_drift_pub_ = nh_.advertise<geometry_msgs::Point>(output_position_drift_topic, 10);

  ROS_INFO("[%s]: Synchronizing topics '%s' and '%s'",
           node_name_.c_str(), in_pose_stamped_topic_.c_str(), in_odometry_topic_.c_str());

  pose_stamped_sub_  = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, in_pose_stamped_topic_, 10);
  odometry_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, in_odometry_topic_, 10);
  sync_.reset(new Sync(SyncPolicyApprox(10), *pose_stamped_sub_, *odometry_sub_));
  sync_->registerCallback(boost::bind(&OdomPoseStampedSync::PoseCallback, this, _1, _2));

  start_sync_srv_ = nh->advertiseService(
      "/odom_posestamped_sync/start_sync", &OdomPoseStampedSync::StartSyncSrv, this);
  stop_sync_srv_  = nh->advertiseService(
      "/odom_posestamped_sync/stop_sync", &OdomPoseStampedSync::StopSyncSrv, this);

  // Check if sync should start immediately
  bool sync_at_start;
  nh_.getParam("sync_at_start", sync_at_start);
  if (sync_at_start) {
    start_new_estimation_ = true;
    is_estimating_ = true;
    position_pair_vec_.clear();
    yaw_pair_vec_.clear();
  }
}

// Subscribe to synchronized messages from mocap localization and vislam
void OdomPoseStampedSync::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_stamped_msg,
                                       const nav_msgs::Odometry::ConstPtr& odom_msg) {
  if (!is_estimating_) {
    return;
  }

  if (start_new_estimation_) {
    start_new_estimation_ = false;
    initial_time_ = pose_stamped_msg->header.stamp;
  }

  const Eigen::Vector3d pos_pose_stamped =
      msg_conversions::ros_point_to_eigen_vector(pose_stamped_msg->pose.position);
  const Eigen::Vector3d pos_odometry =
      msg_conversions::ros_point_to_eigen_vector(odom_msg->pose.pose.position);
  double yaw_mocap = msg_conversions::GetYawFromQuat(pose_stamped_msg->pose.orientation);
  double yaw_vislam = msg_conversions::GetYawFromQuat(odom_msg->pose.pose.orientation);

  // Compute raw offset between pose_stamped and odometry
  const Eigen::Vector3d offset = pos_pose_stamped - pos_odometry;
  output_position_drift_pub_.publish(msg_conversions::eigen_vector_to_ros_point(offset));

  std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_pair(pos_pose_stamped, pos_odometry);
  std::pair<double, double> yaw_pair(yaw_mocap, yaw_vislam);
  position_pair_vec_.push_back(pos_pair);
  yaw_pair_vec_.push_back(yaw_pair);
  times_.push_back((pose_stamped_msg->header.stamp - initial_time_).toSec());

  // Debug printing
  printf("\r[%s]: Number of mocap/vislam pairs = %zd", node_name_.c_str(), position_pair_vec_.size());
  std::fflush(stdout);
}

bool OdomPoseStampedSync::StartSyncSrv(std_srvs::Trigger::Request  &req,
                                       std_srvs::Trigger::Response &res) {
  ROS_INFO("[%s]: Starting Sync!", node_name_.c_str());
  start_new_estimation_ = true;
  is_estimating_ = true;
  position_pair_vec_.clear();
  yaw_pair_vec_.clear();

  res.success = true;
  res.message = "Starting Sync";
  return true;
}

bool OdomPoseStampedSync::StopSyncSrv(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res) {
  if (output_file_path_.length() == 0) {
    ROS_WARN("No output path for Odometry/PoseStamped synchronized data. Will not save to file");
    return true;
  }

  is_estimating_ = false;
  ROS_INFO("\nPrinting synced measurements to files in path %s", output_file_path_.c_str());

  // Create path if it doesn't exist already
  boost::filesystem::create_directories(output_file_path_);

  // Print measurements to file
  std::ofstream x_file, y_file, z_file, yaw_file;
  x_file.open((output_file_path_ + "x_file.txt").c_str());
  y_file.open((output_file_path_ + "y_file.txt").c_str());
  z_file.open((output_file_path_ + "z_file.txt").c_str());
  yaw_file.open((output_file_path_ + "yaw_file.txt").c_str());
  for (uint i = 0; i < position_pair_vec_.size(); i++) {
    const Eigen::Vector3d mocap_pos  = position_pair_vec_[i].first;
    const Eigen::Vector3d vislam_pos = position_pair_vec_[i].second;
    const double mocap_yaw = yaw_pair_vec_[i].first;
    const double vislam_yaw = yaw_pair_vec_[i].second;
    x_file << mocap_pos[0] << ", " << vislam_pos[0] << ", " << times_[i] << std::endl;
    y_file << mocap_pos[1] << ", " << vislam_pos[1] << ", " << times_[i] << std::endl;
    z_file << mocap_pos[2] << ", " << vislam_pos[2] << ", " << times_[i] << std::endl;
    yaw_file << mocap_yaw  << ", " << vislam_yaw    << ", " << times_[i] << std::endl;
  }
  x_file.close();
  y_file.close();
  z_file.close();
  yaw_file.close();

  return true;
}

}