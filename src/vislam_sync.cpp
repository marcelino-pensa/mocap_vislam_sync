// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include "ros/ros.h"
#include "mocap_vislam_sync/mocap_vislam_sync_class.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mocap_vislam_sync");
  ros::NodeHandle node("~");

  sync_topics::MocapVislamSync sync1(&node);

  ros::spin();

  return 0;
}