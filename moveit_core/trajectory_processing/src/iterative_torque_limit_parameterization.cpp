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

#include <moveit/dynamics_solver/dynamics_solver.h>
#include "moveit/trajectory_processing/iterative_torque_limit_parameterization.h"

namespace trajectory_processing
{
namespace
{
const std::string LOGNAME = "trajectory_processing.iterative_torque_limit_parameterization";
constexpr double DEFAULT_VELOCITY_LIMIT = 1.0;
constexpr double DEFAULT_ACCELERATION_LIMIT = 1.0;
}

IterativeTorqueLimitParameterization::IterativeTorqueLimitParameterization(boost::optional<double> path_tolerance,
                                                                           boost::optional<double> resample_dt,
                                                                           boost::optional<double> min_angle_change)
  : totg_(path_tolerance, resample_dt, min_angle_change)
{
}

bool IterativeTorqueLimitParameterization::computeTimeStampsWithTorqueLimits(
    robot_trajectory::RobotTrajectory& trajectory, const geometry_msgs::Vector3& gravity_vector,
    const std::vector<geometry_msgs::Wrench>& external_link_wrenches, const std::vector<double>& joint_torque_limits,
    const double max_velocity_scaling_factor, const double max_acceleration_scaling_factor,
    boost::optional<double> accel_limit_decrement_factor, boost::optional<size_t> max_iterations,
    boost::optional<bool> reset_trajectory_after_max_iterations,
    size_t* iterations_taken) const
{
  // 1. Call computeTimeStamps() to time-parameterize the trajectory with given vel/accel limits.
  // 2. Run forward dynamics to check if torque limits are violated at any waypoint.
  // 3. If a torque limit was violated, decrease the acceleration limit for that joint and go back to Step 1.

  const double accel_decrement_factor = accel_limit_decrement_factor.get_value_or(0.1);
  const size_t max_iter = max_iterations.get_value_or(10);
  const bool reset_trajectory_after_max_iter = reset_trajectory_after_max_iterations.get_value_or(false);

  size_t num_iterations = 0;

  if (iterations_taken)
  {
    *iterations_taken = 0;
  }

  if (trajectory.empty())
    return true;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED(LOGNAME, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const size_t dof = group->getActiveJointModels().size();

  if (joint_torque_limits.size() != dof)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Joint torque limit vector size (" << joint_torque_limits.size()
                                                                       << ") does not match the DOF of the RobotModel ("
                                                                       << dof << ")");
    return false;
  }

  if (accel_decrement_factor < 0.01)
  {
    ROS_ERROR_NAMED(LOGNAME, "The accel_limit_decrement_factor is too small, less than 0.01");
    return false;
  }

  // Lambda for validating limits
  auto validate_limit = [](const char* type, double value, const std::string& name) {
    if (value <= std::numeric_limits<double>::epsilon())
    {
      ROS_ERROR_NAMED(LOGNAME, "Invalid %s limit %f for joint '%s'. Must be greater than zero!", type, value,
                      name.c_str());
      return false;
    }
    return true;
  };

  const std::vector<std::string>& joint_names = group->getActiveJointModelNames();
  size_t num_joints = joint_names.size();
  const robot_model::JointBoundsVector joint_bounds = group->getActiveJointModelsBounds();

  // TODO(cj): Factor out limit resolution; common/duplicated in TOTG as well.
  std::unordered_map<std::string, double> velocity_limits;
  std::unordered_map<std::string, double> mutable_acceleration_limits;

  for (size_t j = 0; j < num_joints; ++j)
  {
    const std::string& name = joint_names[j];
    // Each element in `joint_bounds` is a pointer to a Bounds object, which masks vector<VariableBounds>.
    const robot_model::JointModel::Bounds* bounds_ptr = joint_bounds[j];
    if (bounds_ptr->size() != 1)
    {
      ROS_ERROR_NAMED(LOGNAME, "Cannot handle bounds for multi-variable joint '%s'!", name.c_str());
      return false;
    }
    const moveit::core::VariableBounds vb = (*bounds_ptr)[0];

    // Resolve velocity limit
    if (vb.velocity_bounded_)
    {
      double limit = std::min(std::fabs(vb.max_velocity_), std::fabs(vb.min_velocity_));
      if (!validate_limit("velocity", limit, name))
      {
        return false;
      }
      velocity_limits[name] = limit;
    }
    else
    {
      velocity_limits[name] = DEFAULT_VELOCITY_LIMIT;
      ROS_WARN_ONCE_NAMED(LOGNAME, "No velocity limits defined for '%s'! Define them in the URDF.", name.c_str());
    }

    // Resolve acceleration limit
    if (vb.acceleration_bounded_)
    {
      double limit = std::min(std::fabs(vb.max_acceleration_), std::fabs(vb.min_acceleration_));
      if (!validate_limit("acceleration", limit, name))
      {
        return false;
      }
      mutable_acceleration_limits[name] = limit;
    }
    else
    {
      mutable_acceleration_limits[name] = DEFAULT_ACCELERATION_LIMIT;
      ROS_WARN_ONCE_NAMED(LOGNAME, "No acceleration limits defined for '%s'! Define them in the URDF.", name.c_str());
    }
  }

  dynamics_solver::DynamicsSolver dynamics_solver(trajectory.getRobotModel(), group->getName(), gravity_vector);

  // Copy the waypoints so we can modify them while iterating
  moveit_msgs::RobotTrajectory original_traj;
  trajectory.getRobotTrajectoryMsg(original_traj);
  moveit::core::RobotState initial_state = trajectory.getFirstWayPoint();

  bool iteration_needed = true;

  while (iteration_needed && num_iterations < max_iter)
  {
    ++num_iterations;
    iteration_needed = false;

    // TOTG is always run on the same trajectory;
    // `mutable_acceleration_limits` is the only thing changing across iterations.
    trajectory.setRobotTrajectoryMsg(initial_state, original_traj);
    totg_.computeTimeStamps(trajectory, velocity_limits, mutable_acceleration_limits, max_velocity_scaling_factor,
                            max_acceleration_scaling_factor);

    std::vector<double> joint_positions(dof);
    std::vector<double> joint_velocities(dof);
    std::vector<double> joint_accelerations(dof);
    std::vector<double> joint_torques(dof);

    const std::vector<const moveit::core::JointModel*>& joint_models = group->getActiveJointModels();

    // Check if any torque limits are violated
    for (size_t waypoint_idx = 0; waypoint_idx < trajectory.getWayPointCount(); ++waypoint_idx)
    {
      moveit::core::RobotStatePtr& waypoint = trajectory.getWayPointPtr(waypoint_idx);
      waypoint->copyJointGroupPositions(group->getName(), joint_positions);
      waypoint->copyJointGroupVelocities(group->getName(), joint_velocities);
      waypoint->copyJointGroupAccelerations(group->getName(), joint_accelerations);

      if (!dynamics_solver.getTorques(joint_positions, joint_velocities, joint_accelerations, external_link_wrenches,
                                      joint_torques))
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Dynamics computation failed.");
        return false;
      }

      // For each joint, check if torque exceeds the limit
      for (size_t joint_idx = 0; joint_idx < joint_torque_limits.size(); ++joint_idx)
      {
        if (std::fabs(joint_torques.at(joint_idx)) > joint_torque_limits.at(joint_idx))
        {
          // We can't always just decrease acceleration to decrease joint torque.
          // There are some edge cases where decreasing acceleration could actually increase joint torque. For example,
          // if gravity is accelerating the joint. In that case, the joint would be fighting against gravity more.
          // There is also a small chance that changing acceleration has no effect on joint torque, for example:
          // centripetal acceleration caused by velocity of another joint. This should be uncommon on serial manipulators
          // because their torque limits are high enough to withstand issues like that (or it just wouldn't work at all...)

          // Reset
          waypoint->copyJointGroupAccelerations(group->getName(), joint_accelerations);

          // Check if decreasing acceleration of this joint actually decreases joint torque. Else, increase acceleration.
          double previous_torque = joint_torques.at(joint_idx);
          joint_accelerations.at(joint_idx) *= (1 + accel_decrement_factor);
          if (!dynamics_solver.getTorques(joint_positions, joint_velocities, joint_accelerations,
                                          external_link_wrenches, joint_torques))
          {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Dynamics computation failed.");
            return false;
          }
          if (std::fabs(joint_torques.at(joint_idx)) < std::fabs(previous_torque))
          {
            mutable_acceleration_limits.at(joint_models.at(joint_idx)->getName()) *= (1 + accel_decrement_factor);
          }
          else
          {
            mutable_acceleration_limits.at(joint_models.at(joint_idx)->getName()) *= (1 - accel_decrement_factor);
          }

          iteration_needed = true;
        }
      }  // for each joint
      if (iteration_needed)
      {
        // Start over from the first waypoint
        break;
      }
    }  // for each waypoint
  }  // while (iteration_needed && num_iterations < max_iter)

  if (iterations_taken)
  {
    *iterations_taken = num_iterations;
  }

  // ROS_WARN_STREAM_NAMED(LOGNAME, "ITLP took " << num_iterations << " iterations (max: " << max_iter << ")");

  if (num_iterations >= max_iter && iteration_needed)
  {
    if (reset_trajectory_after_max_iter)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "ITLP needs more than max_iterations; resetting trajectory");
      trajectory.setRobotTrajectoryMsg(initial_state, original_traj);
    }
    else
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Trajectory still violates torque limits!");
    }
  }

  return true;
}
}  // namespace trajectory_processing
