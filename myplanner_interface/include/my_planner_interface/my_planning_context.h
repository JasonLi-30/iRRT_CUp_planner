/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Willow Garage, Inc.
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

/* Author: Chittaranjan Srinivas Swaminathan */

#pragma once

#include <my_planner_interface/my_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <rclcpp/rclcpp.hpp>

namespace my_interface
{
MOVEIT_CLASS_FORWARD(iRRT_CUp_PlanningContext);  // Defines iRRT_CUp_PlanningContextPtr, ConstPtr, WeakPtr... etc

class iRRT_CUp_PlanningContext : public planning_interface::PlanningContext
{
public:
  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  void clear() override;
  bool terminate() override;

  iRRT_CUp_PlanningContext(const std::string& name, const std::string& group, const moveit::core::RobotModelConstPtr& model,
                       const rclcpp::Node::SharedPtr& node);

  ~iRRT_CUp_PlanningContext() override = default;

  void initialize();

private:
  iRRT_CUp_InterfacePtr iRRT_CUp_interface_;
  moveit::core::RobotModelConstPtr robot_model_;
};

}  // namespace my_interface
