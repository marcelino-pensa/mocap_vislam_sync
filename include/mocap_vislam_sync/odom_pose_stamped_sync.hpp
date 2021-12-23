// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include "ros/ros.h"

// ROS types
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

// Eigen
#include <Eigen/Core>

// Locally-defined libraries
#include "mocap_vislam_sync/msg_conversions.hpp"

namespace sync_topics {

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry> SyncPolicyApprox;
typedef message_filters::Synchronizer<SyncPolicyApprox> Sync;

class OdomPoseStampedSync {
 public:
    OdomPoseStampedSync(ros::NodeHandle *nh);

 private:
    ros::NodeHandle nh_;
    ros::Publisher output_position_drift_pub_;
    std::string in_pose_stamped_topic_, in_odometry_topic_;
    std::string output_file_path_;
    boost::shared_ptr<Sync> sync_;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* pose_stamped_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odometry_sub_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > position_pair_vec_;
    std::vector<std::pair<double, double> > yaw_pair_vec_;
    std::vector<double> times_;
    ros::Time initial_time_;
    bool start_new_estimation_, is_estimating_;
    ros::ServiceServer start_sync_srv_, stop_sync_srv_;
    std::string node_name_;

    // Callback that subscribes to both mocap and vislam measurements
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& mocap_msg,
                      const nav_msgs::Odometry::ConstPtr& vislam_msg);

    // Service to trigger the computation of scale
    bool StartSyncSrv(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);
    bool StopSyncSrv(std_srvs::Trigger::Request  &req,
                     std_srvs::Trigger::Response &res);
};

}  // namespace sync_topics