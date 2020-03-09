// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include <Eigen/Core>
#include <geometry_msgs/Point.h>

namespace msg_conversions {

  Eigen::Vector3d ros_point_to_eigen_vector(const geometry_msgs::Point & p);

 }  // namespace msg_conversions