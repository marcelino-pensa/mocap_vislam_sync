// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include "ros/ros.h"
#include "mocap_vislam_sync/odom_pose_stamped_sync.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_pose_stamped_sync");
  ros::NodeHandle node("~");

  sync_topics::OdomPoseStampedSync sync1(&node);

  ros::spin();

  return 0;
}