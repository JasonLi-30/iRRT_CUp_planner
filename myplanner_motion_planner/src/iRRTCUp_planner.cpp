/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
#include <my_motion_planner/my_utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <random>
#include <visualization_msgs/msg/marker_array.hpp>

namespace my_planner
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("iRRT_CUp planner");

iRRT_CUp::iRRT_CUp(iRRTCUp_Trajectory* trajectory, planning_scene::PlanningScenePtr& planning_scene,
                  const std::string& planning_group, moveit::core::RobotState& start_state, iRRT_CUp_Parameters params)
  : group_trajectory_(*trajectory, planning_group, DIFF_RULE_LENGTH)
  , state_(start_state)
{
  full_trajectory_ = trajectory;
  planning_group_ = planning_group;
  planning_scene_ = planning_scene;
  initialized_ = false;
  max_iterations_ = params.max_iterations_;
  initialize();
}

void iRRT_CUp::initialize()
{
  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = planning_group_;
  num_collision_points_ = 1;

  double max_cost_scale = 0.0;

  joint_model_group_ = planning_scene_->getRobotModel()->getJointModelGroup(planning_group_);

  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();

  joint_axes_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));
  joint_positions_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));

  is_collision_free_ = false;
  Tree1_add_ = false;
  Tree2_add_ = false;
  int start = free_vars_start_;
  int end = free_vars_end_;

  initialized_ = true;
  upsampling_ = true;    //是否启用升采样策略
  DPS_ = true;          //是否启用动态路径简化
  incremental_ = true; //是否启用增量规划

  joint_factor_.resize(num_joints_);
  for(int i = 0;i < num_joints_;i++)
    joint_factor_[i] = 1.0;
}

iRRT_CUp::~iRRT_CUp()
{
  destroy();
}


bool iRRT_CUp::planning(double& length)   //路径规划算法主干内容
{
  const auto start_time = std::chrono::system_clock::now();
  bool should_break_out = false;
  double c_cost_last = 0;
  int last_n = 3;
  std::vector<int> last_n_iteration(last_n, 0);
  std::vector<RRT_Node> TEMP_Path;

  bool result = RRTConnectToTrajectory(TEMP_Path, length);

  group_trajectory_.getTrajectory() = PathSplicing(group_trajectory_.getTrajectory(), TEMP_Path, free_vars_start_, free_vars_end_);

  updateFullTrajectory();

  is_collision_free_ = result;

  return is_collision_free_;
  
}


int iRRT_CUp::RRTConnectSteerNode(RRT_Node* goal, std::vector<RRT_Node>& Tree, double maxDis, bool flag_CCD)
{
  RRT_Node start, temp;
  std::vector<double> tempV(num_joints_, 0);
  temp.node.configuration = tempV;
  double min_dist = INFINITY;  
  double dist;
  int j_start = 0;
  int output = 0;
  double norm = 0;
  double RND_Temp = 0;
  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
  bool colli_flag = false;
  std::random_device rd;
  std::mt19937 gen2(rd());                         // 定义随机数生成器对象gen，使用time(nullptr)作为随机数生成器的种子
  std::normal_distribution<double> dis2(0.0, 1.0); // 定义随机数分布器对象dis，期望为0.0，标准差为1.0的正态分布

  {
    for(int j = 0;j < Tree.size();j++)
    {
      dist = SED(goal, &Tree[j]);  
      if(dist < min_dist)
      {
        min_dist = dist;
        j_start = j;
        output = j;
        start = Tree[j];
      }
    }


    for(int j = 0;j < num_joints_;j++)
    {
      norm += pow(goal->node.configuration[j] - start.node.configuration[j],2);
    }
    norm = sqrt(norm);

    RND_Temp = 1;   // (rand() % 1000 / 1000.0); // dis2(gen2);
    for(int j = 0;j < num_joints_;j++)
    {
      const moveit::core::JointModel* joint_model1 = joint_models[j];
      const moveit::core::JointModel::Bounds& bounds = joint_model1->getVariableBounds();

      temp.node.configuration[j] = start.node.configuration[j] + (goal->node.configuration[j]-start.node.configuration[j]) / norm * maxDis * RND_Temp;  //(rand() % 100 / 100.0);
      if(bounds[0].min_position_ > temp.node.configuration[j])
        temp.node.configuration[j] = bounds[0].min_position_;

      if(bounds[0].max_position_ < temp.node.configuration[j])
        temp.node.configuration[j] = bounds[0].max_position_;
    }

    if(flag_CCD)
      colli_flag = getCCD(&Tree[j_start], &temp, 0.05, false);
    else
      colli_flag = getPointColi(temp.node.configuration);

    if(!colli_flag)
    {
      temp.iParent = j_start;
      Tree.push_back(temp);
      output = Tree.size() - 1;
    }
  }
  return output;

}

double iRRT_CUp::SED(RRT_Node* Node1, RRT_Node* Node2)
{
  double output = 0;
  
  for(int i = 0;i < num_joints_;i++)
    output += joint_factor_[i] * pow(Node1->node.configuration[i] - Node2->node.configuration[i], 2);

  return output;
}


double iRRT_CUp::FindNearestNodes(std::vector<RRT_Node>& Tree1, std::vector<RRT_Node>& Tree2, int& iTree1Node, int& iTree2Node, bool flag_CCD, double threshold)
{
  double MinDist = INFINITY;
  double dist;
  bool flag = false;
  if(Tree1.size() < 1 || Tree2.size() < 1)
    return -1;

  if(Tree1_add_ == true)
  {
    for(int i = Tree1.size() - 1;i < Tree1.size();i++)
    {
      for(int j = 0;j < Tree2.size();j++)
      {
        dist = SED(&Tree1[i],&Tree2[j]);
        if(MinDist > dist)
        {
          if(!flag_CCD || (flag_CCD && !getCCD(&Tree1[i],&Tree2[j], 0.05, false)))
          {
            MinDist = dist;
            iTree1Node = i;
            iTree2Node = j;
            if(flag_CCD)
              return MinDist;
          }
          else
          {
            flag = true;
          }
        }
      }
    }
  }
  
  if(Tree2_add_ == true)
  {
    for(int i = 0;i < Tree1.size();i++)
    {
      for(int j = Tree2.size() - 1;j < Tree2.size();j++)
      {
        dist = SED(&Tree1[i],&Tree2[j]);
        if(MinDist > dist)
        {
          if(!flag_CCD || (flag_CCD && !getCCD(&Tree1[i],&Tree2[j], 0.05, false)))
          {
            MinDist = dist;
            iTree1Node = i;
            iTree2Node = j;
            if(flag_CCD)
              return MinDist;
          }
          else
          {
            flag = true;
          }
        }
      }
    }
  }
  
  if(MinDist == INFINITY)    //if(flag && MinDist > threshold)
  {
    // std::cout << "Tree1.size(): " << Tree1.size() << std::endl; 
    // std::cout << "Tree2.size(): " << Tree2.size() << std::endl; 
    // std::cout << "MinDist: " << MinDist << std::endl; 
    return -1;
  }
  return MinDist;
}


bool iRRT_CUp::FindParentNode(RRT_Node* node, std::vector<RRT_Node>& Tree, bool flag_CCD)
{
  double min_dist = INFINITY;  
  double dist;
  bool output = false;
  for(int i = 0;i < Tree.size();i++)
  {
    dist = SED(node, &Tree[i]);  
    if(dist < min_dist && (!flag_CCD || (flag_CCD && !getCCD(node, &Tree[i], 0.05, false)))) 
    {
      min_dist = dist;
      node->iParent = i;
      // node->length = Tree[i].length + sqrt(dist);
      output = true;
    }
  }
  return output;
}

RRT_Node iRRT_CUp::RRTConnectGenNode(std::vector<RRT_Node>& parent, double* RND_Mean_, double* STD_dev, RRT_Node* goal, double maxDis, bool flag_CCD)
{
  int count = 0, MaxIter = 700;
  RRT_Node output;
  output.node.in_Coli = false;
  std::vector<double> temp(num_joints_, 0);
  output.node.configuration = temp;
  output.iParent = 0;
  double norm = 0;
  double normMin = INFINITY;
  int point_i_Min;
  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
  std::random_device rd;
  std::mt19937 gen2(rd());                                      // 定义随机数生成器对象gen，使用time(nullptr)作为随机数生成器的种子
  std::normal_distribution<double> dis2(*RND_Mean_, *STD_dev);  // 定义随机数分布器对象dis，期望为0.0，标准差为1.0的正态分布
  double randTemp = 0;
  bool colli_flag = false;
  while(count < MaxIter)
  {
    for (size_t joint_i = 0; joint_i < joint_models.size(); ++joint_i)
    {
      randTemp = dis2(gen2);
      
      output.node.configuration[joint_i] = randTemp;      //(rand() % 200 / 200.0)
    }
    
    int point_i = floor((rand() % 200 / 200.0)*parent.size());
    if(point_i >= parent.size())
      point_i = parent.size() - 1;
    norm = 0;
    for(int joint_i = 0;joint_i < num_joints_;joint_i++)
    {
      norm += pow(output.node.configuration[joint_i] - parent[point_i].node.configuration[joint_i],2);
    }

    normMin = norm;
    point_i_Min = point_i;
    temp = output.node.configuration;
    
    normMin = sqrt(normMin);

    if(upsampling_ == true)
      randTemp = (rand() % 100 / 100.0);
    else
      randTemp = 1.0;

    for(int j = 0;j < num_joints_;j++)
    {
      const moveit::core::JointModel* joint_model1 = joint_models[j];
      const moveit::core::JointModel::Bounds& bounds = joint_model1->getVariableBounds();

      output.node.configuration[j] = parent[point_i_Min].node.configuration[j] + (temp[j]-parent[point_i_Min].node.configuration[j]) / normMin * maxDis * randTemp;  //dis2(gen2);     //(rand() % 100 / 100.0);
      
      //确保不会超限
      if(bounds[0].min_position_ > output.node.configuration[j])
        output.node.configuration[j] = bounds[0].min_position_;

      if(bounds[0].max_position_ < output.node.configuration[j])
        output.node.configuration[j] = bounds[0].max_position_;
    }

    colli_flag = getPointColi(output.node.configuration);

    if(!colli_flag)
    {
      return output;
    }
    count++;
  }

  output.node.in_Coli = true;
  return output;
}

Eigen::MatrixXd iRRT_CUp::PathSplicing(Eigen::MatrixXd Path1, std::vector<RRT_Node> Path2, int iStart, int iEnd)
{
  int num_points = iStart + Path2.size() + (Path1.rows() - iEnd);
  Eigen::MatrixXd output(num_points, num_joints_);

  for(int i = 0;i < iStart;i++)
  {
    output.row(i) = Path1.row(i);
  }

  for(int i = 0;i < Path2.size();i++)
  {
    for(int j = 0;j < num_joints_;j++)
      output.row(i + iStart)[j] = Path2[i].node.configuration[j];
  }

  for(int i = 0;i < Path1.rows() - iEnd;i++)
  {
    output.row(i + iStart + Path2.size()) = Path1.row(i + iEnd);
  }

  return output;
}


bool iRRT_CUp::getPointColi(std::vector<double>& joint_states)
{
  setRobotState(joint_states);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = planning_group_;
  planning_scene_->checkCollision(req, res, state_, planning_scene_->getAllowedCollisionMatrix());
  return res.collision;
}

bool iRRT_CUp::getCCD(RRT_Node* Node1, RRT_Node* Node2, double Stepsize, bool mode)
{
  double TEMP = SED(Node1, Node2);
  int num = ceil(TEMP / Stepsize);   //* Node1->Co_colli_prob
  std::vector<double> joint_states(num_joints_, 0);
  // std::cout << "num: " << num  << ", TEMP: " << TEMP  << ", Stepsize: " << Stepsize << std::endl;
  for(int ii = 0;ii < num;ii++)
  {
    for(int j = 0;j < num_joints_;j++)
    {
      joint_states[j] = Node1->node.configuration[j] + (Node2->node.configuration[j] - Node1->node.configuration[j]) / (1.0 * num) * ii;
    }
    if(getPointColi(joint_states))
    {
      if(mode)
      {
        Node1->Co_colli_prob = (Node1->Co_colli_prob * Node1->Num_colli_prob + 1.0) / (Node1->Num_colli_prob + 1);
        Node1->Num_colli_prob += 1;
        Node2->Co_colli_prob = (Node2->Co_colli_prob * Node2->Num_colli_prob + 1.0) / (Node2->Num_colli_prob + 1);
        Node2->Num_colli_prob += 1;
      }
      return true;      //有碰撞
    }
  }
  if(mode)
  {
    Node1->Co_colli_prob = (Node1->Co_colli_prob * Node1->Num_colli_prob + 0.0) / (Node1->Num_colli_prob + 1);
    Node1->Num_colli_prob += 1;
    Node2->Co_colli_prob = (Node2->Co_colli_prob * Node2->Num_colli_prob + 0.0) / (Node2->Num_colli_prob + 1);
    Node2->Num_colli_prob += 1;
  }
  return false;         //无碰撞
}

int iRRT_CUp::RRTConnectOnce(std::vector<RRT_Node>& output, RRT_Node* LocalStart, RRT_Node* LocalEnd, double maxDis, bool flag_CCD, bool init)
{
  //报错“free(): double free detected in tcache 2“的一个可能原因是没有return结果。
  //std::vector<double> temp(num_joints_, 0);
  std::vector<RRT_Node> Tree1, Tree2, Path1, Path2;

  int iTree1_out = 0, iTree2_out = 0;
  double threshold = 0.10;

  LocalStart->iParent = -1;
  LocalEnd->iParent = -1;

  Tree1.push_back(*LocalStart);
  Tree2.push_back(*LocalEnd);

  int count = 0;
  double distNearest = 0;
  RRT_Node Samp1;
  double RND_Mean;
  double STD_dev;
  int num_fail = 0;
  
  RND_Mean = 0.0;
  STD_dev = 1.0;
  output.clear();

  while(count < max_iterations_)
  {
    // std::cout << "count: " << count << std::endl;
    // std::cout << "===1===" << std::endl;
    if(count % 2 == 0)         
    {
      if(incremental_ && (flag_CCD || init) && count == 0)
      {
        if(!getCCD(LocalStart, LocalEnd, 0.05))
        {
          output.push_back(*LocalStart);
          output.push_back(*LocalEnd);
          // std::cout << "CCD: collision-free" << std::endl;
          // std::cout << "output.size(): " << output.size() << std::endl;
          return 2;
        }
      }
      
      Samp1 = RRTConnectGenNode(Tree1, &RND_Mean, &STD_dev, LocalEnd, maxDis, flag_CCD);
      if(Samp1.node.in_Coli == true) 
      {
        count++;
        continue;
      }
      
      if(!FindParentNode(&Samp1, Tree1, flag_CCD))
      {
        count++;
        continue;
      }
      Tree1.push_back(Samp1);
      Tree1_add_ = true;
      int iNearest = RRTConnectSteerNode(&Tree1.back(), Tree2, maxDis, flag_CCD);
    }
    else
    {
      Samp1 = RRTConnectGenNode(Tree2, &RND_Mean, &STD_dev, LocalStart, maxDis, flag_CCD);
      if(Samp1.node.in_Coli == true) 
      {
        count++;
        continue;
      }

      if(!FindParentNode(&Samp1, Tree2, flag_CCD))
      {
        count++;
        continue;
      }
      Tree2.push_back(Samp1);
      Tree2_add_ = true;
      int iNearest = RRTConnectSteerNode(&Tree2.back(), Tree1, maxDis, flag_CCD);
    }
    // std::cout << "===2===" << std::endl;
    //尝试将找到的两个近点用于引导随机点生成
    distNearest = FindNearestNodes(Tree1,Tree2,iTree1_out,iTree2_out,flag_CCD,threshold);
    // std::cout << "Distance_of_NearestNodes: " << distNearest << std::endl;
    Tree1_add_ = false;
    Tree2_add_ = false;
    if(distNearest < 0)
    {
      num_fail++;
      // std::cout << "num_fail: " << num_fail << std::endl;
      if(num_fail > 10 && upsampling_ == true)
      {
        // std::cout << "failed" << std::endl;
        return 0;
      }
      count++;
      continue;       //不continue的话会在后面的条件结构中跳出
    }
    else
    {
      num_fail = 0;
      if(flag_CCD)
        break;
    }
    // std::cout << "===3===" << std::endl;
    if(distNearest < maxDis)
    {
      break;
    }
    // count++;
  }
  if(count >= max_iterations_)
  {
    std::cout << "count >= max_iterations_" << std::endl;
    return 0;
  }

  Path1.push_back(Tree1[iTree1_out]);
  Path2.push_back(Tree2[iTree2_out]);

  RRT_Node temp1 = Path1.back();
  RRT_Node temp2 = Path2.back();

  while(temp1.iParent != -1 && Tree1[temp1.iParent].iParent != -1)    //倒序
  {
    Path1.push_back(temp1);
    temp1 = Tree1[temp1.iParent];
  }
  Path1.push_back(Tree1[0]);

  while(temp2.iParent != -1 && Tree2[temp2.iParent].iParent != -1)    //正序
  {
    Path2.push_back(temp2);
    temp2 = Tree2[temp2.iParent];
  }
  Path2.push_back(Tree2[0]);

  // std::cout << "Path1.size(): " << Path1.size() << std::endl;
  // std::cout << "Path2.size(): " << Path2.size() << std::endl;

  for(int i = Path1.size() - 1;i > -1;i--)
  {
    output.push_back(Path1[i]);
    // std::cout << "Path1: " << std::endl;
    // for(int j = 0;j < num_joints_;j++)
    // {
    //   std::cout << Path1[i].node.configuration[j] << ", ";
    // }
    // std::cout << std::endl;
  }
  for(int i = 0;i < Path2.size();i++)
  {
    output.push_back(Path2[i]);
    // std::cout << "Path2: " << std::endl;
    // for(int j = 0;j < num_joints_;j++)
    // {
    //   std::cout << Path2[i].node.configuration[j] << ", ";
    // }
    // std::cout << std::endl;
  }

  return 1;
}

void iRRT_CUp::Simplify(std::vector<RRT_Node>& input, std::vector<RRT_Node>& output, double maxDis, int flag_CCD, double length_total)
{
  // 路径简化
  int i_pre = 0, i_post = 2;
  bool flag_avai = false;
  std::vector<RRT_Node> TEMP_Path;
  TEMP_Path.push_back(input[i_pre]);
  if(input.size() < 3)
  {
    output = input;   //因为两个点的路径的可达性已经得到了验证
    return;    
  }
  else
  {
    auto sq_maxDis = maxDis * maxDis;
    while(i_post < input.size())
    {
      flag_avai = false;
      auto i_post_temp = i_post;
      for(int i = i_post_temp; i < input.size(); i++)
      {
        if(!flag_CCD && SED(&input[i_pre], &input[i]) <= sq_maxDis)  
        { 
          i_post = i + 1;
          flag_avai = true;
        }
        else if(flag_CCD && (SED(&input[i_pre], &input[i]) <= length_total / 10.0) && !getCCD(&input[i_pre], &input[i], 0.05, true))
        {
          i_post = i + 1;
          flag_avai = true;
        }
      }

 
      if(flag_avai)
      {
        TEMP_Path.push_back(input[i_post - 1]);
        i_pre = i_post - 1;
        i_post = i_pre + 2;
      }
      else
      {
        TEMP_Path.push_back(input[i_pre + 1]);
        i_pre = i_pre + 1;
        i_post = i_pre + 2;
      }

      if(i_post == input.size())
      {
        TEMP_Path.push_back(input.back());
        break;
      }
      else if(i_post > input.size())
      {
        break;
      }
    }

    output = TEMP_Path;
    return; 
  }
}

void iRRT_CUp::Simplify_longest(std::vector<RRT_Node>& input, std::vector<RRT_Node>& output, double maxDis)
{
    // 路径简化思路2：查找最远距离的三个相邻点，若SED大于sq_maxDis，尝试线性插补至间隔小于sq_maxDis
    // 若插补点发生碰撞，则不对此处进行简化，否则移除原始点并插入插补点，如此重复直至所有点都无法进一步简化
  int i_pre = 0, i_post = 2;
  bool flag_avai = false;
  std::vector<RRT_Node> TEMP_Path;
  TEMP_Path.push_back(input[i_pre]);
  if(input.size() < 3)
  {
    output = input;   //因为两个点的路径的可达性已经得到了验证
    return;    
  }
  else
  {
    auto sq_maxDis = maxDis * maxDis;
    while(i_post < input.size())
    {
      flag_avai = false;
      auto i_post_temp = i_post;
      for(int i = i_post_temp; i < input.size(); i++)
      {
        if(SED(&input[i_pre], &input[i]) <= sq_maxDis)  
        { 
          i_post = i + 1;
          flag_avai = true;
        }
      }

      if(flag_avai)
      {
        TEMP_Path.push_back(input[i_post - 1]);
        i_pre = i_post - 1;
        i_post = i_pre + 2;
      }
      else
      {
        TEMP_Path.push_back(input[i_pre + 1]);
        i_pre = i_pre + 1;
        i_post = i_pre + 2;
      }

      if(i_post == input.size())
      {
        TEMP_Path.push_back(input.back());
        break;
      }
      else if(i_post > input.size())
      {
        break;
      }
    }

    output = TEMP_Path;
    return; 
  }
}


bool iRRT_CUp::RRTConnectToTrajectory(std::vector<RRT_Node>& output, double& length)
{
  std::vector<double> ColiPath_start(num_joints_, 0);
  std::vector<double> ColiPath_end(num_joints_, 0);
  std::vector<RRT_Node> TEMP_Path, TEMP_Path2;
  RRT_Node LocalStart, LocalEnd, TEMP_Point;
  bool ori = false;
  bool init = true;

  for(int j = 0;j < num_joints_;j++)
    ColiPath_start[j] = group_trajectory_.getTrajectory().row(free_vars_start_)[j];

  for(int j = 0;j < num_joints_;j++)
    ColiPath_end[j] = group_trajectory_.getTrajectory().row(free_vars_end_-1)[j];

  LocalStart.node.configuration = ColiPath_start;
  LocalEnd.node.configuration = ColiPath_end;
  TEMP_Point.node.configuration = ColiPath_start;
  LocalStart.iParent = -1;
  LocalEnd.iParent = -1;
  TEMP_Point.iParent = -1;


  if(SED(&LocalStart,&LocalEnd) < 1e-3)
    return true;

  double maxDis = 0;
  double length_total = SED(&LocalStart, &LocalEnd);
  double maxDis_threshold = 0.2;
  int flag = 0;
  int Path_Size;
  bool flag_CCD = false, coll_last = false;
  double TEMP = SED(&LocalStart, &LocalEnd);
  double Stepsize = 0.05;
  int num = ceil(TEMP / Stepsize);
  TEMP_Path2.push_back(LocalStart);
  TEMP_Path2.push_back(LocalEnd);

  maxDis = 0;
  for(int i = 0;i < TEMP_Path2.size() - 1;i++)
  {
    TEMP = SED(&TEMP_Path2[i], &TEMP_Path2[i + 1]);
    if(maxDis < TEMP)
      maxDis = TEMP;
  }

  if(maxDis <= Stepsize)
  {
    length = 0;
    output = TEMP_Path2;
    for(int i = 0;i < output.size() - 1;i++)
    {
      double max_diff = 0;
      for(int j = 0;j < num_joints_;j++)
      {
        if(fabs(output[i].node.configuration[j]-output[i+1].node.configuration[j]) > max_diff)
        {
          max_diff = fabs(output[i].node.configuration[j]-output[i+1].node.configuration[j]);
        }
      }
      length += max_diff;
    }
    return true;
  }

  maxDis /= 2;

  if(upsampling_ == false)
    maxDis = maxDis_threshold;    //如果要切换为原始RRT-connect则使用这一行，否则注释掉这一行

  while(init || maxDis >= maxDis_threshold)
  {
    Path_Size = TEMP_Path2.size();
    output.clear();
    output.push_back(TEMP_Path2[0]);
    if(maxDis == maxDis_threshold)
    {
      flag_CCD = true;
    }

    for(int i = 0;i < Path_Size - 1;i++)
    {
      LocalStart = TEMP_Path2[i];
      LocalEnd = TEMP_Path2[i + 1];

      flag = RRTConnectOnce(TEMP_Path, &LocalStart, &LocalEnd, maxDis, flag_CCD, init);
      if(flag == 0)
      {
        break;
      }

      if(flag > 0)
      {
        for(int j = 1;j < TEMP_Path.size();j++)
        {
          output.push_back(TEMP_Path[j]);
        }
      }
      TEMP_Path.clear();
      if(init && flag == 2)
      {
        length = length_total;
        return true;
      }
      init = false;
    }

    if(!flag)
    {
      break;
    }

    if(DPS_ == true)
    {
      if(!flag_CCD)        //最后做连续碰撞检测的一轮时不做简化，以避免影响可达性
      {
        int size1, size2, size3;
        while(true)
        {
          // std::cout << "output_1.size(): " << output.size() << std::endl;
          size1 = output.size();
          std::vector<RRT_Node> output_reverse;
          for(int i = output.size() - 1; i > -1; i--)
          {
            output[i].isCCDfree = false;
            output_reverse.push_back(output[i]);
          }
          // std::cout << "output_2.size(): " << output.size() << std::endl;
          size2 = output.size();
          Simplify(output_reverse, output_reverse, maxDis, flag_CCD, length_total);
          output.clear();
          for(int i = output_reverse.size() - 1; i > -1; i--)
          {
            output.push_back(output_reverse[i]);
          }
          Simplify(output, output, maxDis, flag_CCD, length_total);
          // std::cout << "output_3.size(): " << output.size() << std::endl;
          size3 = output.size();

          if(size1 == size2 && size1 == size3)
            break;
        }
      }
      else
      {
        Simplify(output, output, maxDis, flag_CCD, length_total);
      }
    }

    TEMP_Path2 = output;

    if(flag_CCD)
    {
      break;
    }

    maxDis /= 2;
    if(maxDis < maxDis_threshold)
    {
      maxDis = maxDis_threshold;
    }
  }
  output = TEMP_Path2;

  if(!flag)
    return false;
  else
  {
    length = 0;

    for(int i = 0;i < output.size() - 1;i++)
    {
      double max_diff = 0;
      max_diff = SED(&output[i], &output[i+1]);
      length += sqrt(max_diff);
    }

    return true;
  }
  
}


void iRRT_CUp::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void iRRT_CUp::setRobotState(std::vector<double>& joint_states)
{
  state_.setJointGroupPositions(planning_group_, joint_states);
  state_.update();
}


}  // namespace my_planner
