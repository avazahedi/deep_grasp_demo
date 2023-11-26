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

/* Author: Boston Cleek
   Desc:   Grasp pose detection (GPD) using point clouds
*/

// ROS
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/impl/point_types.hpp>
#include <moveit_msgs/Grasp.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

// Eigen
#include <Eigen/Dense>

// Object Detection
#include "jaco_grasp_ros_interfaces/BboxCoords.h"

#include <moveit_task_constructor_gpd/grasp_detection.h>
#include <moveit_task_constructor_gpd/cloud_utils.h>

namespace moveit_task_constructor_gpd
{
GraspDetection::GraspDetection(const ros::NodeHandle& nh) : nh_(nh), goal_active_(false), tfBuffer(), tfListener(tfBuffer)
{
  loadParameters();
  init();
}

void GraspDetection::loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading grasp action server parameters");
  ros::NodeHandle pnh("~");

  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "load_cloud", load_cloud_);
  if (load_cloud_)
  {
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_pcd_file", path_to_pcd_file_);
  }
  else
  {
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "point_cloud_topic", point_cloud_topic_);
  }

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_gpd_config", path_to_gpd_config_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "trans_cam_opt", transform_cam_opt_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "trans_base_cam", trans_base_cam_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "view_point", view_point_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "frame_id", frame_id_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void GraspDetection::init()
{
  // action server
  server_.reset(new actionlib::SimpleActionServer<grasping_msgs::GraspPlanningAction>(nh_, action_name_, false));
  server_->registerGoalCallback(std::bind(&GraspDetection::goalCallback, this));
  server_->registerPreemptCallback(std::bind(&GraspDetection::preemptCallback, this));
  server_->start();

  // GPD point cloud camera, load cylinder from file
  // set camera view origin
  // assume cloud was taken using one camera
  if (load_cloud_)
  {
    Eigen::Matrix3Xd camera_view_point(3, 1);
    camera_view_point << view_point_.at(0), view_point_.at(1), view_point_.at(2);
    cloud_camera_.reset(new gpd::util::Cloud(path_to_pcd_file_, camera_view_point));
  }
  else
  {
    cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &GraspDetection::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1, true);

    // subscriber to /passthrough_filter_vals
    passthrough_sub_ = nh_.subscribe("/passthrough_filter_vals", 1, &GraspDetection::passthroughCallback, this);

    // initialize passthrough and center values to -99 (dummy default value)
    pass_xmin = -99;
    pass_xmax = -99;
    pass_ymax = -99;
    pass_ymin = -99;
    pass_depth = -99;
    obj_xcenter = -99;
    obj_ycenter = -99;

    // initialize transformStamped to default dummy value
    transformStamped.transform.translation.x = -99;

    timer = nh_.createTimer(ros::Duration(0.1), &GraspDetection::timerCallback, this);
  }

  // Grasp detector
  grasp_detector_.reset(new gpd::GraspDetector(path_to_gpd_config_));
}

void GraspDetection::timerCallback(const ros::TimerEvent&)
{
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);

  try {
    // target frame, source frame, ros::Time, (ros::Duration ?)
    // transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time(0));
    transformStamped = tfBuffer.lookupTransform("world", "camera_color_optical_frame", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
}

void GraspDetection::passthroughCallback(const jaco_grasp_ros_interfaces::BboxCoords::ConstPtr& msg)
{
  pass_xmin = msg->left[0];
  pass_xmax = msg->right[0];
  pass_ymax = msg->top[1];
  pass_ymin = msg->bottom[1];
  pass_depth = msg->center_depth;
  obj_xcenter = msg->center[0];
  obj_ycenter = msg->center[1];
}

void GraspDetection::goalCallback()
{
  goal_name_ = server_->acceptNewGoal()->object.name;
  ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_name_.c_str());
  goal_active_ = true;

  // sample grasps now else need to wait to callback
  // use GPD to find the grasp candidates
  if (load_cloud_)
  {
    sampleGrasps();
  }
}

void GraspDetection::preemptCallback()
{
  ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_name_.c_str());
  server_->setPreempted();
}

void GraspDetection::sampleGrasps()
{
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;  // detect grasp poses
  grasp_detector_->preprocessPointCloud(*cloud_camera_);      // preprocess the point cloud
  grasps = grasp_detector_->detectGrasps(*cloud_camera_);     // detect grasps in the point cloud

  // Use grasps with score > 0
  std::vector<unsigned int> grasp_id;
  for (unsigned int i = 0; i < grasps.size(); i++)
  {
    if (grasps.at(i)->getScore() > 0.0)
    {
      grasp_id.push_back(i);
    }
  }

  if (grasp_id.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No grasp candidates found with a positive cost");
    server_->setAborted(result_);
    return;
  }

  for (auto id : grasp_id)
  {
    // transform grasp from camera optical link into frame_id (panda_link0, root)
    const Eigen::Isometry3d transform_opt_grasp =
        Eigen::Translation3d(grasps.at(id)->getPosition()) * Eigen::Quaterniond(grasps.at(id)->getOrientation());

    const Eigen::Isometry3d transform_base_grasp = trans_base_cam_ * transform_cam_opt_ * transform_opt_grasp;
    const Eigen::Vector3d trans = transform_base_grasp.translation();
    const Eigen::Quaterniond rot(transform_base_grasp.rotation());

    // convert back to PoseStamped
    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = frame_id_;
    grasp_pose.pose.position.x = trans.x();
    grasp_pose.pose.position.y = trans.y();
    grasp_pose.pose.position.z = trans.z();

    grasp_pose.pose.orientation.w = rot.w();
    grasp_pose.pose.orientation.x = rot.x();
    grasp_pose.pose.orientation.y = rot.y();
    grasp_pose.pose.orientation.z = rot.z();

    moveit_msgs::Grasp current_grasp;
    current_grasp.grasp_pose = grasp_pose;
    current_grasp.grasp_quality = static_cast<double>(1.0 / grasps.at(id)->getScore());
    feedback_.grasps.emplace_back(current_grasp);
  }

  server_->publishFeedback(feedback_);
  server_->setSucceeded(result_);
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

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object)
{
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

void GraspDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Make sure RealSense has enough time to get a full point cloud before starting processing
  if (msg->data.size() < 2300000 && point_cloud_topic_ != "/cloud_pcd") {
    return;
  }

  // Make sure passthrough values have been initialized by YOLO object detection bounding box
  if ((pass_xmin == -99 || pass_xmax == -99 || pass_ymax == -99 || pass_ymin == -99 || 
      pass_depth == -99 || obj_xcenter == -99 || obj_ycenter == -99) 
      && point_cloud_topic_ != "/cloud_pcd") {
    return;
  }

  // check that camera_link to world tf has been initialized
  if (transformStamped.transform.translation.x == -99) {
    return;
  }

  if (goal_active_)
  {
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::fromROSMsg(*msg.get(), *cloud.get());

    // Filtering (units: m)
    // Assuming object is less than 0.1m thick (depth for passthrough)
    std::vector<double> xyz_lower{pass_xmin, pass_ymin, 0.01};
    std::vector<double> xyz_upper{pass_xmax, pass_ymax, pass_depth + 0.1};
    passThroughFilter(xyz_lower, xyz_upper, cloud);

    // double radius = 0.01;
    // int min_neighbors = 5;
    double radius = 0.005;
    int min_neighbors = 15;
    radiusOutlierRemoval(radius, min_neighbors, cloud);

    // VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
    voxgrid.setInputCloud(cloud);
    voxgrid.setLeafSize(0.01f, 0.01f, 0.01f);
    // voxgrid.setLeafSize(0.005f, 0.005f, 0.005f);
    voxgrid.filter(*cloud.get());

    // publish the cloud for visualization and debugging purposes
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);
    cloud_pub_.publish(cloud_msg);

    // Create collision object for segmented cloud
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped transformStamped;
    // try {
    //   // target frame, source frame, ros::Time, (ros::Duration ?)
    //   transformStamped = tfBuffer.lookupTransform("world", "camera_link", ros::Time(0));
    // }
    // catch (tf2::TransformException &ex) {
    //   ROS_WARN("%s", ex.what());
    //   // ros::Duration(1.0).sleep();
    // }

    ros::NodeHandle pnh("~");
    moveit::planning_interface::PlanningSceneInterface psi;
    std::string desired_object, object_reference_frame;
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "/desired_object", desired_object);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "/mtc_tutorial/object_reference_frame", object_reference_frame);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    auto obj_radius = (pass_xmax - pass_xmin) / 2.0;

    geometry_msgs::Pose obj_cam_pose; // camera frame
    obj_cam_pose.position.x = obj_xcenter;
    obj_cam_pose.position.y = obj_ycenter;
    obj_cam_pose.position.z = pass_depth + obj_radius;  // to center the collision object
    obj_cam_pose.orientation.x = 0.0;
    obj_cam_pose.orientation.y = 0.0;
    obj_cam_pose.orientation.z = 0.0;
    obj_cam_pose.orientation.w = 1.0;

    ROS_INFO_NAMED(LOGNAME, "obj_cam_pose %f %f %f", obj_cam_pose.position.x, obj_cam_pose.position.y, obj_cam_pose.position.z);

    // obj_pose z is center of object
    geometry_msgs::Pose obj_pose; // world frame
    std::vector<double> obj_dimensions(2);  // [height, radius]
    obj_dimensions.at(0) = pass_ymax - pass_ymin;
    obj_dimensions.at(1) = obj_radius;

    // populate obj_pose by transforming obj_cam_pose with the camera_link to world tf
    tf2::doTransform(obj_cam_pose, obj_pose, transformStamped);

    ROS_INFO_NAMED(LOGNAME, "transformStamped translation %f %f %f", 
                                                        transformStamped.transform.translation.x, 
                                                        transformStamped.transform.translation.y, 
                                                        transformStamped.transform.translation.z);
    ROS_INFO_NAMED(LOGNAME, "transformStamped rotation %f %f %f %f", 
                                                        transformStamped.transform.rotation.x, 
                                                        transformStamped.transform.rotation.y, 
                                                        transformStamped.transform.rotation.z,
                                                        transformStamped.transform.rotation.w);

    obj_pose.orientation.x = 0.0;
    obj_pose.orientation.y = 0.0;
    obj_pose.orientation.z = 0.0;
    obj_pose.orientation.w = 1.0;

    ROS_INFO_NAMED(LOGNAME, "obj_pose %f %f %f", obj_pose.position.x, obj_pose.position.y, obj_pose.position.z);

    moveit_msgs::CollisionObject collision_object = createCollisionObject(desired_object, 
                                                                          object_reference_frame, 
                                                                          obj_dimensions, 
                                                                          obj_pose);
    
    spawnObject(psi, collision_object);

    // TODO: set alpha channel to 1
    // GPD required XYZRGBA
    PointCloudRGBA::Ptr grasp_cloud(new PointCloudRGBA);
    pcl::copyPointCloud(*cloud.get(), *grasp_cloud.get());

    // Construct the cloud camera
    Eigen::Matrix3Xd camera_view_point(3, 1);
    camera_view_point << view_point_.at(0), view_point_.at(1), view_point_.at(2);
    cloud_camera_.reset(new gpd::util::Cloud(grasp_cloud, 0, camera_view_point));

    // use GPD to find the grasp candidates
    sampleGrasps();
  }

  goal_active_ = false;
}
}  // namespace moveit_task_constructor_gpd
