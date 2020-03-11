// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include "ros/ros.h"

// ROS types
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

// Eigen
#include <Eigen/Core>

// Locally-defined libraries
#include "mocap_vislam_sync/msg_conversions.hpp"

namespace sync_topics {

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> SyncPolicyApprox;
typedef message_filters::Synchronizer<SyncPolicyApprox> Sync;

class MocapVislamSync {
 public:
    MocapVislamSync(ros::NodeHandle *nh);

 private:
    ros::NodeHandle nh_;
    std::string in_mocap_topic_, in_vislam_topic_;
    std::string output_file_path_;
    boost::shared_ptr<Sync> sync_;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* mocap_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* vislam_sub_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > position_pair_vec_;
    std::vector<double> times_;
    ros::Time initial_time_;
    bool start_new_estimation_, is_estimating_;
    Eigen::Vector3d first_pos_mocap_, first_pos_vislam_;
    ros::ServiceServer start_scale_estimation_srv_, stop_scale_estimation_srv_;
    std::string node_name_;

    // Callback that subscribes to both mocap and vislam measurements
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& mocap_msg,
                      const geometry_msgs::PoseStamped::ConstPtr& vislam_msg);

    // Service to trigger the computation of scale
    bool StartSyncSrv(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);
    bool StopSyncSrv(std_srvs::Trigger::Request  &req,
                     std_srvs::Trigger::Response &res);
};

}  // namespace sync_topics