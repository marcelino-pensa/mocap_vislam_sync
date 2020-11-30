// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>

namespace msg_conversions {

  Eigen::Vector3d ros_point_to_eigen_vector(const geometry_msgs::Point & p);

  double GetYawFromQuat(const geometry_msgs::Quaternion &quat);

 }  // namespace msg_conversions