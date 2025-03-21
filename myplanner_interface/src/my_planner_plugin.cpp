/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <my_planner_interface/my_planning_context.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include <pluginlib/class_list_macros.hpp>
#include <vector>

namespace my_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("iRRT_CUp_Planner");

class iRRT_CUp_PlannerManager : public planning_interface::PlannerManager
{
public:
  iRRT_CUp_PlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& /* unused */) override
  {
    planning_interface::PlannerConfigurationMap pconfig;
    for (const std::string& group : model->getJointModelGroupNames())
    {
      planning_contexts_[group] = std::make_shared<iRRT_CUp_PlanningContext>("my_planning_context", group, model, node);
      const planning_interface::PlannerConfigurationSettings planner_config_settings{
        group, group, std::map<std::string, std::string>()
      };
      pconfig[planner_config_settings.name] = planner_config_settings;
    }

    setPlannerConfigurations(pconfig);
    return true;
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    std::cout << "===getPlanningContext===" << std::endl;
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      RCLCPP_ERROR(LOGGER, "No group specified to plan for");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      RCLCPP_ERROR(LOGGER, "No planning scene supplied as input");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // create PlanningScene using hybrid collision detector
    planning_scene::PlanningScenePtr ps = planning_scene->diff();
    // ps->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create());

    // retrieve and configure existing context
    const iRRT_CUp_PlanningContextPtr& context = planning_contexts_.at(req.group_name);
    context->setPlanningScene(ps);
    context->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return context;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    // TODO: this is a dummy implementation
    //      capabilities.dummy = false;
    return true;
  }

  std::string getDescription() const override
  {
    return "iRRT_CUp";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.resize(1);
    algs[0] = "iRRT_CUp";
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override
  {
    config_settings_ = pcs;
  }

protected:
  std::map<std::string, iRRT_CUp_PlanningContextPtr> planning_contexts_;
};

}  // namespace my_interface

PLUGINLIB_EXPORT_CLASS(my_interface::iRRT_CUp_PlannerManager, planning_interface::PlannerManager)
