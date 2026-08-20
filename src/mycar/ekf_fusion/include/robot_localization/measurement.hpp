/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * Copyright (c) 2017, Locus Robotics, Inc.
 * Copyright (c) 2019, Steve Macenski
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOT_LOCALIZATION__MEASUREMENT_HPP_
#define ROBOT_LOCALIZATION__MEASUREMENT_HPP_

#include <Eigen/Dense>

#include <limits>
#include <string>
#include <vector>
#include <memory>

#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

namespace robot_localization
{

/**
 * @brief Structure used for storing and comparing measurements
 * (for priority queues)
 *
 * Measurement units are assumed to be in meters and radians. Times are
 * real-valued and measured in seconds.
 */
struct Measurement
{
  Measurement()
  : time_(0), mahalanobis_thresh_(std::numeric_limits<double>::max()),
    max_position_update_(std::numeric_limits<double>::max()),
    max_position_innovation_(std::numeric_limits<double>::max()),
    max_yaw_update_(std::numeric_limits<double>::max()),
    process_noise_scale_(1.0), correction_attempted_(false), correction_accepted_(false),
    correction_reject_reason_("none"), position_innovation_(0.0), yaw_innovation_(0.0),
    nis_(0.0), requested_position_update_(0.0), applied_position_update_(0.0),
    requested_yaw_update_(0.0), applied_yaw_update_(0.0),
    position_correction_clamped_(false), yaw_correction_clamped_(false),
    latest_control_time_(0), topic_name_(""), latest_control_()
  {
  }

  // The real-valued time, in seconds, since some epoch (presumably the start of
  // execution, but any will do)
  rclcpp::Time time_;

  // The Mahalanobis distance threshold in number of sigmas
  double mahalanobis_thresh_;

  // Optional safety limit for the XY component of one correction. This is
  // normally infinite and is set only for sparse absolute pose sources such
  // as the HIGH-quality LiDAR odometry update.
  double max_position_update_;

  // Optional safety values for sparse absolute pose sources. Innovation is
  // evaluated after prediction to this measurement's timestamp.
  double max_position_innovation_;
  double max_yaw_update_;

  // Stage 6 rare-event Q scale captured when this measurement is queued.
  double process_noise_scale_;

  // Machine-readable correction outcome, filled by Ekf::correct().
  mutable bool correction_attempted_;
  mutable bool correction_accepted_;
  mutable std::string correction_reject_reason_;
  mutable double position_innovation_;
  mutable double yaw_innovation_;
  mutable double nis_;
  mutable double requested_position_update_;
  mutable double applied_position_update_;
  mutable double requested_yaw_update_;
  mutable double applied_yaw_update_;
  mutable bool position_correction_clamped_;
  mutable bool yaw_correction_clamped_;

  // The time stamp of the most recent control term (needed for lagged data)
  rclcpp::Time latest_control_time_;

  // The topic name for this measurement. Needed for capturing previous state
  // values for new measurements.
  std::string topic_name_;

  // This defines which variables within this measurement actually get passed
  // into the filter. std::vector<bool> is generally frowned upon, so we use
  // ints.
  std::vector<bool> update_vector_;

  // The most recent control vector (needed for lagged data)
  Eigen::VectorXd latest_control_;

  // The measurement and its associated covariance
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;

  // We want earlier times to have greater priority
  bool operator()(
    const std::shared_ptr<Measurement> & a,
    const std::shared_ptr<Measurement> & b)
  {
    return (*this)(*(a.get()), *(b.get()));
  }

  bool operator()(const Measurement & a, const Measurement & b)
  {
    return a.time_ > b.time_;
  }
};
using MeasurementPtr = std::shared_ptr<Measurement>;

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__MEASUREMENT_HPP_
