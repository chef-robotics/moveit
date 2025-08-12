/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Re:Build AppliedLogix, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Andy Zelenak */
/* Description: Time-parameterize a trajectory with Time Optimal Trajectory Generation, then iterate until torque limits are obeyed. */

#pragma once

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <boost/optional.hpp>
#include <unordered_map>

namespace trajectory_processing
{
class IterativeTorqueLimitParameterization
{
public:
  // TimeOptimalTrajectoryGeneration defaults will be used for any parameters that are not specified.
  IterativeTorqueLimitParameterization(boost::optional<double> path_tolerance = boost::none,
                                       boost::optional<double> resample_dt = boost::none,
                                       boost::optional<double> min_angle_change = boost::none);

  /**
   * \brief See computeTimeStampsWithTorqueLimits(trajectory, velocity_limits, acceleration_limits, ...) for docs.
   */
  bool computeTimeStampsWithTorqueLimits(
      robot_trajectory::RobotTrajectory& trajectory, const std::vector<double>& torque_limits,
      const geometry_msgs::Vector3& gravity_vector,
      const std::vector<geometry_msgs::Wrench>& external_link_wrenches,
      const double max_velocity_scaling_factor, const double max_acceleration_scaling_factor,
      boost::optional<double> accel_limit_decrement_factor = boost::none,
      boost::optional<size_t> max_iterations = boost::none,
      boost::optional<bool> reset_trajectory_after_max_iterations = boost::none) const
  {
    // Delegate to the overload that accepts custom velocity and acceleration limits, using empty maps
    // to indicate that RobotModel limits should be used.
    std::unordered_map<std::string, double> empty_velocity_limits;
    std::unordered_map<std::string, double> empty_acceleration_limits;
    return computeTimeStampsWithTorqueLimits(trajectory, empty_velocity_limits, empty_acceleration_limits,
                                             torque_limits, gravity_vector, external_link_wrenches,
                                             max_velocity_scaling_factor, max_acceleration_scaling_factor,
                                             accel_limit_decrement_factor, max_iterations,
                                             reset_trajectory_after_max_iterations);
  }

  /**
   * \brief Compute a trajectory with waypoints spaced equally in time (according to resample_dt_).
   * \param[in,out] trajectory A path which needs time-parameterization. It's OK if this path has already been
   * time-parameterized; this function will re-time-parameterize it.
   * \param velocity_limits Joint names and velocity limits [rad/s, m/s].
   * \param acceleration_limits Joint names and acceleration limits [rad/s^2, m/s^2].
   * \param torque_limits Torque limits for each joint (all should be > 0) [N*m].
   * \param gravity_vector Orientation of the gravity vector w.r.t. the RobotModel's base frame [m/s^2].
   * \param external_link_wrenches Externally-applied wrenches on each link. TODO(andyz): what frame is this in?
   * \param max_velocity_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   * \param max_acceleration_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   * \param accel_limit_decrement_factor (Optional) How much to change the acceleration limits every iteration.
   *   Time-optimality of the output is accurate to approximately (100*accel_limit_decrement_factor)%. For example, if
   *   accel_limit_decrement_factor is 0.1, the output should be within 10% of time-optimal. Must be > 0.01. If not
   *   specified, default is 0.1.
   * \param max_iterations (Optional) Maximum number of times to do the TOTG->torque check->accel change loop. If not
   *   specified, default is 10.
   * \param reset_traj_after_max_iterations (Optional) Whether to reset `trajectory` to its initial state when more
   *   than `max_iterations` iterations is needed to get all torques under their limits. If not specified, default is
   *   false.
   */
  bool computeTimeStampsWithTorqueLimits(
      robot_trajectory::RobotTrajectory& trajectory,
      const std::unordered_map<std::string, double>& velocity_limits,
      const std::unordered_map<std::string, double>& acceleration_limits,
      const std::vector<double>& torque_limits,
      const geometry_msgs::Vector3& gravity_vector,
      const std::vector<geometry_msgs::Wrench>& external_link_wrenches,
      const double max_velocity_scaling_factor, const double max_acceleration_scaling_factor,
      boost::optional<double> accel_limit_decrement_factor = boost::none,
      boost::optional<size_t> max_iterations = boost::none,
      boost::optional<bool> reset_trajectory_after_max_iterations = boost::none) const;

private:
  TimeOptimalTrajectoryGeneration totg_;
};
}  // namespace trajectory_processing
