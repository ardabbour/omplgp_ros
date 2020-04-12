/**
 * OMPL global planner ROS plugin prototype.
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

#ifndef OMPLGP_ROS_HPP
#define OMPLGP_ROS_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omplgp/omplgp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace omplgp_ros
{
class OMPL2DPlannerROS : public nav2_core::GlobalPlanner
{
public:
  /**
   * Constructor
   */
  OMPL2DPlannerROS();

  /**
   * Destructor
   */
  ~OMPL2DPlannerROS();

  /**
   * Configures the plugin
   * @param parent the node to which this plugin belongs to
   * @param name the name of the plugin
   * @param tf the TF buffer
   * @param costmap_ros the costmap to be used for planning
   */
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * Activates the plugin
   */
  void activate() override;

  /**
   * Deactivates the plugin
   */
  void deactivate() override;

  /**
   * Cleans up the plugin
   */
  void cleanup() override;

  /**
   * Creates a plan
   * @param start_pose the start pose of the robot
   * @param goal_pose the goal pose of the robot
   */
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start_pose,
                                 const geometry_msgs::msg::PoseStamped& goal_pose) override;

protected:
  /**
   * Converts the ROS costmap into the omplgp library format
   * @param costmap_2d_ros ROS costmap
   * @return omplgp costmap
   */
  void costmap2DROSToCostmap2D(const nav2_costmap_2d::Costmap2D& costmap_2d_ros, omplgp::CostMap2D& costmap_2d);

  /**
   * Converts the omplgp plan into a ROS path
   * @param plan2d omplgp plan
   * @return a ROS path
   */
  void plan2DToPath(const omplgp::Plan2D& plan2d, nav_msgs::msg::Path& path);

  /**
   * Plugin name
   */
  std::string name_;

  /**
   * Node pointer
   */
  nav2_util::LifecycleNode::SharedPtr node_;

  /**
   * ROS costmap
   */
  nav2_costmap_2d::Costmap2D* costmap_2d_ros_;

  /**
   * Global frame of the costmap
   */
  std::string global_frame_;

  /**
   * TF buffer
   */
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /**
   * Whether or not the planner should be allowed to plan through unknown space
   */
  bool allow_unknown_;

  /**
   * omplgp planner pointer
   */
  std::unique_ptr<omplgp::OMPL2DPlanner> planner_;

  /**
   * Planning algorithm
   */
  int algorithm_;

  /**
   * Maximum amount of time the planner will run for
   */
  double timeout_;
};
}  // namespace omplgp_ros

PLUGINLIB_EXPORT_CLASS(omplgp_ros::OMPL2DPlannerROS, nav2_core::GlobalPlanner)

#endif