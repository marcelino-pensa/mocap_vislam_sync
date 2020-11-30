// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include <mocap_vislam_sync/msg_conversions.hpp>

namespace msg_conversions {

Eigen::Vector3d ros_point_to_eigen_vector(const geometry_msgs::Point & p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

double GetYawFromQuat(const geometry_msgs::Quaternion &quat) {
  double yaw, pitch, roll;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace msg_conversions