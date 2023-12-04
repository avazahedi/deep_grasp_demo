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

// // ROS
// #include <ros/ros.h>

// // Object Detection
// #include "jaco_grasp_ros_interfaces/BboxCoords.h"

// // MTC demo implementation
// #include <deep_grasp_task/deep_pick_place_task.h>

// #include <geometry_msgs/Pose.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <rosparam_shortcuts/rosparam_shortcuts.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <std_msgs/Int8.h>

// #include <iostream>

// #include <geometric_shapes/shape_operations.h>
// #include <grasping_msgs/GraspPlanningAction.h>
// #include <actionlib/client/simple_action_client.h>

// constexpr char LOGNAME[] = "deep_grasp_demo";

#include <deep_grasp_task/deep_grasp_demo.h>


void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object)
{
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable()
{
  ros::NodeHandle pnh("~");
  std::string table_name, table_reference_frame;
  std::vector<double> table_dimensions;
  geometry_msgs::Pose pose;
  std::size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

  moveit_msgs::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = table_dimensions;
  pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createObject()
{
  ros::NodeHandle pnh("~");
  std::string object_name, object_reference_frame;
  std::vector<double> object_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  pose.position.z += 0.5 * object_dimensions[0];
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCamera()
{
  ros::NodeHandle pnh("~");
  std::string camera_name, camera_reference_frame, camera_mesh_file;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_name", camera_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_mesh_file", camera_mesh_file);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_reference_frame", camera_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  shapes::Mesh* obj_mesh = shapes::createMeshFromResource(camera_mesh_file);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(obj_mesh, mesh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject object;
  object.id = camera_name;
  object.header.frame_id = camera_reference_frame;
  object.meshes.emplace_back(mesh);
  object.mesh_poses.emplace_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createObjectMesh()
{
  ros::NodeHandle pnh("~");
  std::string object_name, object_reference_frame, object_mesh_file;
  std::vector<double> object_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_mesh_file", object_mesh_file);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  shapes::Mesh* obj_mesh = shapes::createMeshFromResource(object_mesh_file);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(obj_mesh, mesh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.meshes.emplace_back(mesh);
  object.mesh_poses.emplace_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCollisionObject(const std::string& object_name, 
                                                   const std::string& object_reference_frame, 
                                                   const std::vector<double>& object_dimensions, 
                                                   const geometry_msgs::Pose& object_pose)
{
  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  // object_pose.position.z += 0.5 * object_dimensions[0];
  object.primitive_poses.push_back(object_pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

DeepGraspDemo::DeepGraspDemo(const ros::NodeHandle& nh) : nh_(nh), obj_dimensions_({-99, -99}), tfBuffer(), tfListener(tfBuffer)
{
  ROS_INFO_NAMED(LOGNAME, "DEEPGRASPDEMO CONSTRUCTOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

  trajectory_finished_pub_ = nh_.advertise<std_msgs::Int8>("/trajectory_finished", 1, true);
  traj_fin_msg.data = 0;

  // subscriber to /passthrough_filter_vals
  passthrough_sub_ = nh_.subscribe("/passthrough_filter_vals", 1, &DeepGraspDemo::passthroughCallback, this);

  pnh_ = ros::NodeHandle("~");

  stage = Stage::waiting;
  num_attempts = 0;
  // goal_active = false;
  object_spawned = false;

  // this is always the same
  obj_cam_pose.orientation.x = 0.0;
  obj_cam_pose.orientation.y = 0.0;
  obj_cam_pose.orientation.z = 0.0;
  obj_cam_pose.orientation.w = 1.0;

  // initialize transformStamped to default dummy value
  transformStamped.transform.translation.x = -99;

  // // Wait for ApplyPlanningScene service
  // ros::Duration(1.0).sleep();

  // if (pnh_.param("spawn_table", true))
  // {
  //   spawnObject(psi, createTable());
  // }

  // // Add camera to planning scene
  // if (pnh_.param("spawn_camera", true))
  // {
  //   spawnObject(psi, createCamera());
  // }

  // // Add object to planning scene either as mesh or geometric primitive
  // if (pnh_.param("spawn_mesh", true))
  // {
  //   spawnObject(psi, createObjectMesh());
  // }

  // // Spawn collision object in pre-defined location
  // if (pnh_.param("spawn_object", true))
  // {
  //   spawnObject(psi, createObject());
  // }
  

  timer = nh_.createTimer(ros::Duration(0.1), &DeepGraspDemo::timerCallback, this);
}

void DeepGraspDemo::passthroughCallback(const jaco_grasp_ros_interfaces::BboxCoords::ConstPtr& msg)
{
  // ROS_INFO_NAMED(LOGNAME, "IN THE PASSTHROUGH CALLBACK$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n\n\n\n\n");
  auto pass_xmin = msg->left[0];
  auto pass_xmax = msg->right[0];
  auto pass_ymax = msg->top[1];
  auto pass_ymin = msg->bottom[1];
  auto pass_depth = msg->center_depth;
  auto obj_xcenter = msg->center[0];
  auto obj_ycenter = msg->center[1];

  auto obj_radius = (pass_xmax - pass_xmin) / 2.0;

  obj_dimensions_.at(0) = pass_ymax - pass_ymin;
  obj_dimensions_.at(1) = obj_radius;

  obj_cam_pose.position.x = obj_xcenter;
  obj_cam_pose.position.y = obj_ycenter;
  obj_cam_pose.position.z = pass_depth + obj_radius;  // to center the collision object
}

void DeepGraspDemo::timerCallback(const ros::TimerEvent&)
{
  // ROS_INFO_NAMED(LOGNAME, "IN THE TIMER CALLBACK @@@@@@@@@@@@@@@@@@@@@@@@@@\n\n\n\n\n\n");
  try {
    // target frame, source frame, ros::Time, (ros::Duration ?)
    // transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time(0));
    transformStamped = tfBuffer.lookupTransform("world", "camera_color_optical_frame", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // wait for object dimensions to be initialized from point cloud
  if (obj_dimensions_.at(0) != -99 && transformStamped.transform.translation.x != -99 && !object_spawned)
  {
    stage = Stage::planning;

    std::string desired_object, object_name, object_reference_frame;
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "/desired_object", desired_object);  // "bottle"
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_name", object_name);  // "object"
    error += !rosparam_shortcuts::get(LOGNAME, pnh_, "/mtc_tutorial/object_reference_frame", object_reference_frame);
    // error+= !rosparam_shortcuts::get(LOGNAME, pnh_, "/mtc_tutorial/goal_active", goal_active);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    // obj_pose z is center of object
    geometry_msgs::Pose obj_pose; // world frame

    // populate obj_pose by transforming obj_cam_pose with the camera_link to world tf
    tf2::doTransform(obj_cam_pose, obj_pose, transformStamped);

    // restore orientation (should be upright)
    obj_pose.orientation.x = 0.0;
    obj_pose.orientation.y = 0.0;
    obj_pose.orientation.z = 0.0;
    obj_pose.orientation.w = 1.0;

    moveit_msgs::CollisionObject collision_object = createCollisionObject(desired_object, 
                                                                            object_reference_frame, 
                                                                            obj_dimensions_, 
                                                                            obj_pose);

    spawnObject(psi, collision_object);
    ROS_INFO_NAMED(LOGNAME, "SPAWNED OBJECT #########################\n\n");
    object_spawned = true;
  }

  // ROS_INFO_NAMED(LOGNAME, "goal_active param: %d", goal_active);

  // Construct and run task
  while (stage == Stage::planning) {
    ROS_INFO_NAMED(LOGNAME, "000000000000000000000000000000000000000\n\n\n");

    deep_grasp_task::DeepPickPlaceTask deep_pick_place_task("deep_pick_place_task", nh_);
    ROS_INFO_NAMED(LOGNAME, "CONSTRUCTED deep_pick_place_task-------------\n\n\n");
    deep_pick_place_task.loadParameters();
    ROS_INFO_NAMED(LOGNAME, "LOADED PARAMETERS------------\n\n\n");
    deep_pick_place_task.init();

    ROS_INFO_NAMED(LOGNAME, "FINISHED deep_pick_place_task.init() !!!!!!!!!!!!!\n\n\n");

    if (deep_pick_place_task.plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
      if (pnh_.param("execute", false))
      {
        traj_fin_msg.data = 1;
        trajectory_finished_pub_.publish(traj_fin_msg);

        deep_pick_place_task.execute();
        ROS_INFO_NAMED(LOGNAME, "Execution complete");

        traj_fin_msg.data = 2;
        trajectory_finished_pub_.publish(traj_fin_msg);
      }
      else
      {
        ROS_INFO_NAMED(LOGNAME, "Execution disabled");
      }
      stage = Stage::executed;
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
      std::cout << "\n\n\n";
      ROS_INFO_NAMED(LOGNAME, "Trying again...");
      num_attempts++;
      std::cout << "num_attempts: " << num_attempts << "\n";
      if (num_attempts >= 5) {
        ROS_INFO_NAMED(LOGNAME, "5 attempts have failed. Please restart.");
        stage = Stage::stopped;
      }
    }
  }

}

int main(int argc, char** argv)
{
  ROS_INFO_NAMED(LOGNAME, "Init deep_grasp_demo");
  ros::init(argc, argv, "deep_grasp_demo");
  ros::NodeHandle nh;

  // DeepGraspDemo deep_grasp_demo(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Wait for ApplyPlanningScene service
  ros::Duration(1.0).sleep();
  // Add table and object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  ros::NodeHandle pnh("~");
  if (pnh.param("spawn_table", true))
  {
    spawnObject(psi, createTable());
  }

  // Add camera to planning scene
  if (pnh.param("spawn_camera", true))
  {
    spawnObject(psi, createCamera());
  }

  // Add object to planning scene either as mesh or geometric primitive
  if (pnh.param("spawn_mesh", true))
  {
    spawnObject(psi, createObjectMesh());
  }

  // Spawn collision object in pre-defined location
  if (pnh.param("spawn_object", true))
  {
    spawnObject(psi, createObject());
  }

  DeepGraspDemo deep_grasp_demo(nh);

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
