/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein, Boston Cleek
   Desc:   A demo to show MoveIt Task Constructor using a deep learning based
           grasp generator
*/

// ROS
#include <ros/ros.h>

// Object Detection
#include "jaco_grasp_ros_interfaces/BboxCoords.h"

// MTC demo implementation
#include <deep_grasp_task/deep_pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>

#include <geometric_shapes/shape_operations.h>
#include <grasping_msgs/GraspPlanningAction.h>
#include <actionlib/client/simple_action_client.h>

constexpr char LOGNAME[] = "deep_grasp_demo";

/**
 * @brief Stage enum for keeping track of planning process
*/
enum class Stage : uint8_t
{
  planning = 0,
  executed = 1,
  stopped = 2,
  waiting = 3
};

/**
 * @brief Generates grasp poses for a generator stage with MTC
 * @details Interfaces with the GPD lib using ROS messages and interfaces
 *          with MTC using an Action Server
 */
class DeepGraspDemo
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  DeepGraspDemo(const ros::NodeHandle& nh);

private:
  /**
   * @brief Timer callback
   * @details Continuously running callback
   */
  void timerCallback(const ros::TimerEvent&);

  /**
   * @brief Passthrough values callback
   * @param msg - custom BboxCoords msg type containing bounding box information
   * @details To be used in passthrough filter to isolate object in point cloud
  */
  void passthroughCallback(const jaco_grasp_ros_interfaces::BboxCoords::ConstPtr& msg);


private:
  ros::NodeHandle nh_;         // node handle
  ros::Subscriber passthrough_sub_; //subscribes to passthrough values (custom jaco_grasp_ros_interfaces msg)
  ros::Publisher trajectory_finished_pub_;   // publishes segmented cloud

  ros::NodeHandle pnh_;   // private node handle

  moveit::planning_interface::PlanningSceneInterface psi;

  Stage stage;
  uint8_t num_attempts;
  std_msgs::Int8 traj_fin_msg;
  std::vector<double> obj_dimensions_;  // [height, radius]
  geometry_msgs::Pose obj_cam_pose; // camera frame
  bool goal_active;
  bool object_spawned;

  ros::Timer timer;

  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
};
