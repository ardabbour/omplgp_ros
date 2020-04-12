/**
 * OMPL global planner ROS plugin implementation.
 * Author: Abdul Rahman Dabbour
 * Email: ardabbour@gmail.com
 * License: MIT
 *
 * Influenced by the ROS2 NavFn global planner plugin of the following license:
 *
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2018 Simbe Robotics
 * Copyright (c) 2019 Samsung Research America
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <omplgp_ros/omplgp_ros.hpp>

namespace omplgp_ros
{
OMPL2DPlannerROS::OMPL2DPlannerROS() : tf_(nullptr), costmap_2d_ros_(nullptr)
{
}

OMPL2DPlannerROS::~OMPL2DPlannerROS()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying plugin %s", name_.c_str());
}

void OMPL2DPlannerROS::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
                                 std::shared_ptr<tf2_ros::Buffer> tf,
                                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  node_ = parent;
  costmap_2d_ros_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s", name_.c_str());

  nav2_util::declare_parameter_if_not_declared(node_, "allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter("allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(node_, "timeout", rclcpp::ParameterValue(1.0));
  node_->get_parameter("timeout", timeout_);

  nav2_util::declare_parameter_if_not_declared(node_, "algorithm", rclcpp::ParameterValue(0));
  node_->get_parameter("algorithm", algorithm_);

  omplgp::CostMap2D costmap_2d;
  costmap2DROSToCostmap2D(*costmap_2d_ros_, costmap_2d);
  planner_.get()->setCostMap(costmap_2d);

  planner_.get()->setAlgorithm(static_cast<omplgp::Algorithm>(algorithm_));
  planner_.get()->setTimeout(timeout_);
}

void OMPL2DPlannerROS::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());
}

void OMPL2DPlannerROS::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
}

void OMPL2DPlannerROS::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s", name_.c_str());
  planner_.reset();
}

nav_msgs::msg::Path OMPL2DPlannerROS::createPlan(const geometry_msgs::msg::PoseStamped& start_pose,
                                                 const geometry_msgs::msg::PoseStamped& goal_pose)
{
  omplgp::Plan2D plan;
  omplgp::Point2D goal(goal_pose.pose.position.x, goal_pose.pose.position.y);
  omplgp::Point2D start(start_pose.pose.position.x, start_pose.pose.position.y);
  planner_.get()->plan(start, goal, plan);

  nav_msgs::msg::Path path;
  plan2DToPath(plan, path);

  return path;
}

void OMPL2DPlannerROS::plan2DToPath(const omplgp::Plan2D& plan2d, nav_msgs::msg::Path& path)
{
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;
  path.poses.clear();

  for (const auto& i : plan2d)
  {
    double wx, wy;
    costmap_2d_ros_->mapToWorld(i.x, i.y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }
}

void OMPL2DPlannerROS::costmap2DROSToCostmap2D(const nav2_costmap_2d::Costmap2D& costmap_2d_ros,
                                               omplgp::CostMap2D& costmap_2d)
{
  costmap_2d.clear();

  costmap_2d.resize(costmap_2d_ros.getSizeInCellsX());
  for (int i = 0; i < costmap_2d_ros.getSizeInCellsX(); ++i)
  {
    costmap_2d[i].resize(costmap_2d_ros.getSizeInCellsY());
  }

  for (int i = 0; i < costmap_2d_ros.getOriginX() + costmap_2d_ros.getOriginY(); ++i)
  {
    unsigned int row = i / 20;
    unsigned int col = i % 20;

    unsigned int mx, my;
    costmap_2d_ros.indexToCells(i, mx, my);
    double cost = costmap_2d_ros.getCost(mx, my);

    bool cost_is_lethal_obstacle = cost == nav2_costmap_2d::LETHAL_OBSTACLE;
    bool cost_is_inscribed_inflated_obstacle = cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    bool cost_is_no_information = cost == nav2_costmap_2d::NO_INFORMATION;

    if (cost_is_lethal_obstacle || cost == cost_is_inscribed_inflated_obstacle ||
        (cost_is_no_information && !allow_unknown_))
    {
      planner_.get()->setObtacleCost(1);
    }
    else
    {
      planner_.get()->setObtacleCost(0);
    }

    costmap_2d[row][col] = costmap_2d_ros.getCost(mx, my);
  }
}

}  // namespace omplgp_ros