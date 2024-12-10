/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Shengjie Li */

#include <my_motion_planner/iRRTCUp_planner.h>
#include <my_motion_planner/my_planner.h>
#include <my_motion_planner/my_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>

namespace my_planner
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("my_planner");

bool my_planner::iRRT_CUp_Planner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const planning_interface::MotionPlanRequest& req, const iRRT_CUp_Parameters& params,
                         planning_interface::MotionPlanDetailedResponse& res) const
{
  int count = 0;
  double avg_time = 0;
  double max_time = 0;
  double min_time = 1000000;
  int loopNum = 100;
  int successNum = 0;
  double successRate = 0.0;
  double avg_length = 0;
  double max_length = 0;
  double min_length = 1000000;

  while(count < loopNum)
  {
    auto start_time = std::chrono::system_clock::now();
    if (!planning_scene)
    {
      RCLCPP_ERROR(LOGGER, "No planning scene initialized.");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return false;
    }

    // get the specified start state
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

    if (!start_state.satisfiesBounds())
    {
      RCLCPP_ERROR(LOGGER, "Start state violates joint limits");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return false;
    }

    my_planner::iRRTCUp_Trajectory trajectory(planning_scene->getRobotModel(), 3.0, .03, req.group_name);
    robotStateToArray(start_state, req.group_name, trajectory.getTrajectoryPoint(0));

    if (req.goal_constraints.size() != 1)
    {
      RCLCPP_ERROR(LOGGER, "Expecting exactly one goal constraint, got: %zd", req.goal_constraints.size());
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    if (req.goal_constraints[0].joint_constraints.empty() || !req.goal_constraints[0].position_constraints.empty() ||
        !req.goal_constraints[0].orientation_constraints.empty())
    {
      RCLCPP_ERROR(LOGGER, "Only joint-space goals are supported");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    const size_t goal_index = trajectory.getNumPoints() - 1;
    moveit::core::RobotState goal_state(start_state);
    for (const moveit_msgs::msg::JointConstraint& joint_constraint : req.goal_constraints[0].joint_constraints)
      goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
    if (!goal_state.satisfiesBounds())
    {
      RCLCPP_ERROR(LOGGER, "Goal state violates joint limits");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return false;
    }
    robotStateToArray(goal_state, req.group_name, trajectory.getTrajectoryPoint(goal_index));

    const moveit::core::JointModelGroup* model_group =
        planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
    // fix the goal to move the shortest angular distance for wrap-around joints:
    for (size_t i = 0; i < model_group->getActiveJointModels().size(); ++i)
    {
      const moveit::core::JointModel* model = model_group->getActiveJointModels()[i];
      const moveit::core::RevoluteJointModel* revolute_joint =
          dynamic_cast<const moveit::core::RevoluteJointModel*>(model);

      if (revolute_joint != nullptr)
      {
        if (revolute_joint->isContinuous())
        {
          double start = (trajectory)(0, i);
          double end = (trajectory)(goal_index, i);
          RCLCPP_INFO(LOGGER, "Start is %f end %f short %f", start, end, shortestAngularDistance(start, end));
          (trajectory)(goal_index, i) = start + shortestAngularDistance(start, end);
        }
      }
    }

    //==========================以上计算时间可忽略不计========================

    trajectory.fillInLinearInterpolation();
    
    auto create_time = std::chrono::system_clock::now();

    int replan_count = 0;
    bool replan_flag = false;
    double org_planning_time_limit = 10;
    int org_max_iterations = 200;

    // storing the initial parameters values
    org_planning_time_limit = params.planning_time_limit_;
    org_max_iterations = params.max_iterations_;

    std::unique_ptr<my_planner::iRRT_CUp> planner;

    // create a non_const_params variable which stores the non constant version of the const params variable
    my_planner::iRRT_CUp_Parameters params_nonconst = params;
    RCLCPP_DEBUG(LOGGER, "Input trajectory has %zd joints", trajectory.getTrajectory().rows());

    auto start_time1 = std::chrono::system_clock::now();
    
    planning_scene::PlanningScenePtr planning_scene2 = planning_scene::PlanningScene::clone(planning_scene);
    
    try
    {
      planner = std::make_unique<my_planner::iRRT_CUp>(&trajectory, planning_scene2, req.group_name, start_state, params_nonconst);
    }
    catch(const std::exception& e)
    {
      // std::cout << "===initialization failed, trying again...===" << std::endl;
      return false;
    }
    
    
    // std::cout << "===initialization over===" << std::endl;
    if (!planner->isInitialized())
    {
      RCLCPP_ERROR(LOGGER, "Could not initialize planner");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }
    auto initialize_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time1).count();
    // std::cout << "initialize_time: " << initialize_time << std::endl;
    
    double length = 0;
    bool planning_result = planner->planning(length);
    
    // std::cout << "Length: " << length << std::endl;
      
    //==========================以下计算时间可忽略不计========================
    // resetting the Parameters to the original values after a successful plan
    params_nonconst.setRecoveryParams(org_planning_time_limit, org_max_iterations);

    RCLCPP_DEBUG(LOGGER, "Planning actually took %ld sec to run",
                (std::chrono::system_clock::now() - create_time).count());
    create_time = std::chrono::system_clock::now();

    RCLCPP_DEBUG(LOGGER, "Output trajectory has %zd joints", trajectory.getNumJoints());

    auto result = std::make_shared<robot_trajectory::RobotTrajectory>(planning_scene->getRobotModel(), req.group_name);

    for (int i = 0; i < trajectory.getTrajectory().rows(); ++i)
    {
      const Eigen::MatrixXd::RowXpr source = trajectory.getTrajectoryPoint(i);
      auto state = std::make_shared<moveit::core::RobotState>(start_state);
      size_t joint_index = 0;

      for (const moveit::core::JointModel* jm : result->getGroup()->getActiveJointModels())
      {
        assert(jm->getVariableCount() == 1);
        state->setVariablePosition(jm->getFirstVariableIndex(), source[joint_index++]);
      }

      result->addSuffixWayPoint(state, 0.0);
    }
    // std::cout << "result->getWayPointCount(): " << result->getWayPointCount() << std::endl;
    

    res.trajectory_.resize(1);
    res.trajectory_[0] = result;
    
    RCLCPP_DEBUG(LOGGER, "Bottom took %ld sec to create", (std::chrono::system_clock::now() - create_time).count());
    RCLCPP_DEBUG(LOGGER, "Serviced planning request in %ld wall-seconds",
                (std::chrono::system_clock::now() - start_time).count());

    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    res.processing_time_.resize(1);
    res.processing_time_[0] = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

    if(planning_result)
    {
      if(res.processing_time_[0] > max_time)
        max_time = res.processing_time_[0];
      if(res.processing_time_[0] < min_time)
        min_time = res.processing_time_[0];
      avg_time += res.processing_time_[0];

      if(length > max_length)
        max_length = length;
      if(length < min_length)
        min_length = length;
      avg_length += length;
    }
    else
    {
      count++;
      continue;
      // return false;
    }

    // std::cout << "Processing_time_" << count << ": " << res.processing_time_[0] << std::endl;

    // report planning failure if path has collisions
    if (not planner->isCollisionFree())
    {
      RCLCPP_ERROR(LOGGER, "Motion plan is invalid.--LSJ");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }

    successNum++;
    count++;
  }

  avg_time /= successNum;
  avg_length /= successNum;
  successRate = successNum * 1.0 / loopNum;
  std::cout << "Processing_avg_time: " << avg_time << std::endl;
  std::cout << "Processing_max_time: " << max_time << std::endl;
  std::cout << "Processing_min_time: " << min_time << std::endl;
  std::cout << "Processing_avg_length: " << avg_length << std::endl;
  std::cout << "Processing_max_length: " << max_length << std::endl;
  std::cout << "Processing_min_length: " << min_length << std::endl;
  std::cout << "SuccessRate: " << successRate << std::endl;

  return true;
}

}  // namespace my_planner
