// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include "mocap_vislam_sync/hector_vislam_sync_class.hpp"

#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

namespace sync_topics {

HectorVislamSync::HectorVislamSync(ros::NodeHandle *nh) {
    nh_= *nh;
    nh_.getParam("in_lidar_topic", in_lidar_topic_);
    nh_.getParam("in_vislam_topic", in_vislam_topic_);
    nh_.getParam("output_path", output_file_path_);
    start_new_estimation_ = false;
    is_estimating_ = false;
    node_name_ = "lidar_vislam_sync";

    ROS_INFO("[%s]: Synchronizing topics '%s' and '%s'",
             node_name_.c_str(), in_lidar_topic_.c_str(), in_vislam_topic_.c_str());

    lidar_sub_  = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, in_lidar_topic_, 10);
    vislam_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, in_vislam_topic_, 10);
    sync_.reset(new Sync(SyncPolicyApprox(10), *lidar_sub_, *vislam_sub_));
    sync_->registerCallback(boost::bind(&HectorVislamSync::PoseCallback, this, _1, _2));

    start_scale_estimation_srv_ = nh->advertiseService(
        "/hector_vislam_sync_class/start_sync", &HectorVislamSync::StartSyncSrv, this);
    stop_scale_estimation_srv_  = nh->advertiseService(
        "/hector_vislam_sync_class/stop_sync", &HectorVislamSync::StopSyncSrv, this);
}

// Subscribe to synchronized messages from lidar localization and vislam
void HectorVislamSync::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& lidar_msg,
                                        const geometry_msgs::PoseStamped::ConstPtr& vislam_msg) {
    if (!is_estimating_) {
        return;
    }

    if (start_new_estimation_) {
        start_new_estimation_ = false;
        first_pos_lidar_  = msg_conversions::ros_point_to_eigen_vector(lidar_msg->pose.position);
        first_pos_vislam_ = msg_conversions::ros_point_to_eigen_vector(vislam_msg->pose.position);
    }

    Eigen::Vector3d pos_lidar = 
        msg_conversions::ros_point_to_eigen_vector(lidar_msg->pose.position) - first_pos_lidar_;
    Eigen::Vector3d pos_vislam = 
        msg_conversions::ros_point_to_eigen_vector(vislam_msg->pose.position) - first_pos_vislam_;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_pair(pos_lidar, pos_vislam);
    position_pair_vec_.push_back(pos_pair);

    // Debug printing
    printf("\r[%s]: Number of lidar/vislam pairs = %zd", node_name_.c_str(), position_pair_vec_.size());
    std::fflush(stdout);
}

bool HectorVislamSync::StartSyncSrv(std_srvs::Trigger::Request  &req,
                                   std_srvs::Trigger::Response &res) {
    ROS_INFO("[%s]: Starting Sync!", node_name_.c_str());
    start_new_estimation_ = true;
    is_estimating_ = true;
    position_pair_vec_.clear();

    res.success = true;
    res.message = "Starting Sync";
    return true;
}

bool HectorVislamSync::StopSyncSrv(std_srvs::Trigger::Request  &req,
                                  std_srvs::Trigger::Response &res) {
    double scale, singular_value_ratio;
    uint n_meas;
    is_estimating_ = false;
    ROS_INFO("\nPrinting synced measurements to files in path %s", output_file_path_.c_str());

    // Print measurements to file
    std::ofstream x_file, y_file, z_file;
    x_file.open((output_file_path_ + "x_file.txt").c_str());
    y_file.open((output_file_path_ + "y_file.txt").c_str());
    z_file.open((output_file_path_ + "z_file.txt").c_str());
    for (uint i = 0; i < position_pair_vec_.size(); i++) {
        Eigen::Vector3d lidar_pos  = position_pair_vec_[i].first;
        Eigen::Vector3d vislam_pos = position_pair_vec_[i].second;
       x_file << lidar_pos[0] << ", " << vislam_pos[0] << std::endl;
       y_file << lidar_pos[1] << ", " << vislam_pos[1] << std::endl;
       z_file << lidar_pos[2] << ", " << vislam_pos[2] << std::endl;
    }
    x_file.close();
    y_file.close();
    z_file.close();

    return true;
}

}