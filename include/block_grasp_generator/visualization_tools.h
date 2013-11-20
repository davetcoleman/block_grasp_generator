/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

// Author: Dave Coleman
// Desc:   Simple tools for showing parts of a robot in Rviz, such as the gripper or arm

#ifndef BLOCK_GRASP_GENERATOR__VISUALIZATION_TOOLS_
#define BLOCK_GRASP_GENERATOR__VISUALIZATION_TOOLS_

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <shape_tools/solid_primitive_dims.h>

// MoveIt Messages
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

// ROS
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>

// Boost
#include <boost/shared_ptr.hpp>

// Messages
#include <std_msgs/ColorRGBA.h>

namespace block_grasp_generator
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string COLLISION_TOPIC = "/collision_object";
static const std::string ATTACHED_COLLISION_TOPIC = "/attached_collision_object";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";

enum rviz_colors { RED, GREEN, BLUE, GREY, WHITE, ORANGE };

class VisualizationTools
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher pub_rviz_marker_; // for rviz visualization markers
  ros::Publisher pub_collision_obj_; // for MoveIt collision objects
  ros::Publisher pub_attach_collision_obj_; // for MoveIt attached objects
  ros::Publisher pub_display_path_; // for MoveIt trajectories

  // Pointer to a Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Strings
  std::string marker_topic_; // topic to publish to rviz
  std::string ee_group_name_; // end effector group name
  std::string planning_group_name_; // planning group we are working with
  std::string base_link_; // name of base link of robot
  std::string ee_parent_link_; // parent link of end effector, loaded from MoveIt!

  double floor_to_base_height_; // allows an offset between base link and floor where objects are built

  // Duration to have Rviz markers persist, 0 for infinity
  ros::Duration marker_lifetime_;

  // End Effector Markers
  visualization_msgs::MarkerArray ee_marker_array_;
  tf::Pose tf_root_to_link_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
  std::vector<geometry_msgs::Pose> marker_poses_;

  // Whether to actually publish to rviz or not
  bool muted_;

  // Cached Rviz markers
  visualization_msgs::Marker arrow_marker_;
  visualization_msgs::Marker sphere_marker_;
  visualization_msgs::Marker block_marker_;
  visualization_msgs::Marker text_marker_;

public:

  /**
   * \brief Constructor with planning scene
   */
  VisualizationTools(std::string base_link, 
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
    std::string marker_topic = RVIZ_MARKER_TOPIC);

  /**
   * \brief Constructor w/o planning scene passed in
   */
  VisualizationTools(std::string base_link = "base", std::string marker_topic = RVIZ_MARKER_TOPIC);

  /**
   * \brief Deconstructor
   */
  ~VisualizationTools();

  /**
   * \brief Allows an offset between base link and floor where objects are built. Default is zero
   * \param floor_to_base_height - the offset
   */
  void setFloorToBaseHeight(double floor_to_base_height);

  /**
   * \brief Convert generic grasp pose to this end effector's frame of reference
   * \param pose - the transform
   */
  void setGraspPoseToEEFPose(geometry_msgs::Pose grasp_pose_to_eef_pose);

  /**
   * \brief Pre-load rviz markers for better efficiency
   */
  void loadRvizMarkers();

  /**
   * \brief Load a planning scene monitor if one was not passed into the constructor
   * \return true if successful in loading
   */
  bool loadPlanningSceneMonitor();

  bool loadRobotMarkers();

  /**
   * \brief Call this once at begining to load the robot marker
   * \return true if it is successful
   */
  bool loadEEMarker();

  /**
   * \brief Publish an end effector to rviz
   * \return true if it is successful
   */
  bool publishEEMarkers(const geometry_msgs::Pose &pose, const rviz_colors &color = WHITE,
    const std::string &ns="end_effector");

  /**
   * \brief Publish an marker of a sphere to rviz
   * \return true if it is successful
   */
  bool publishSphere(const geometry_msgs::Pose &pose);

  /**
   * \brief Publish an marker of an arrow to rviz
   * \return true if it is successful
   */
  bool publishArrow(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE);

  /**
   * \brief Publish an marker of a block to Rviz
   * \return true if it is successful
   */
  bool publishBlock(const geometry_msgs::Pose &pose, const double &block_size, const bool isRed);

  /**
   * \brief Publish an marker of a text to Rviz
   * \return true if it is successful
   */
  bool publishText(const geometry_msgs::Pose &pose, const std::string &text,
    const rviz_colors &color = WHITE);

  /**
   * \brief Set the name of the end effector
   */
  void setEEGroupName(const std::string& ee_group_name)
  {
    ee_group_name_ = ee_group_name;
  }

  /**
   * \brief Provide the name of the planning group moveit will use
   */
  void setPlanningGroupName(const std::string& planning_group_name)    
  {
    planning_group_name_ = planning_group_name;
  }

  /**
   * \brief Set this class to not actually publish anything to Rviz.
   * \param muted true if verbose
   */
  void setMuted(bool muted)
  {
    muted_ = muted;
  }

  /**
   * \brief Return if we are in verbose mode
   */
  bool isMuted()
  {
    return muted_;
  }

  /**
   * \brief Set the lifetime of markers published to rviz
   * \param lifetime seconds of how long to show markers. 0 for inifinity
   */
  void setLifetime(int lifetime)
  {
    marker_lifetime_ = ros::Duration(lifetime);

    // Update cached markers
    arrow_marker_.lifetime = marker_lifetime_;
    block_marker_.lifetime = marker_lifetime_;
    text_marker_.lifetime = marker_lifetime_;    
  }

  /**
   * @brief Get the planning scene monitor that this class is using
   * @param planning_scene_monitor
   * @return true if successful
   */
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    if( !planning_scene_monitor_ )
    {
      loadPlanningSceneMonitor();
    }

    return planning_scene_monitor_;
  }

  /**
   * \brief Remove a collision object from the planning scene
   * \param Name of object
   */
  void cleanupCO(std::string name);

  void cleanupACO(const std::string& name);

  void attachCO(const std::string& name);

  void publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name,
    double block_size);

  void publishCollisionWall(double x, double y, double angle, double width, const std::string name);

  void publishCollisionTable(double x, double y, double angle, double width, double height,
    double depth, const std::string name);

  /**
   * \brief Animate trajectory in rviz
   * \param trajectory_msg the actual plan
   * \param waitTrajectory whether we need to wait for the animation to complete
   * \return true if no errors
   */
  bool publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg, bool waitTrajectory);

  /**
   * \brief Get the RGB value of standard colors
   * \param color - a enum pre-defined name of a color
   * \return the RGB message equivalent
   */
  std_msgs::ColorRGBA getColor(const rviz_colors &color);

  /**
   * \brief Get the end effector parent link as loaded from the SRDF
   * \return string of name of end effector parent link
   */
  const std::string& getEEParentLink()
  {
    // Make sure we already loaded the EE markers
    loadEEMarker();

    return ee_parent_link_;
  }


  bool isMarkerPubLoaded()
  {
    if( !pub_rviz_marker_ )
    {
    ROS_ERROR_STREAM_NAMED("temp","Pub Rviz Marker is not loaded. this is a HACK! - todo");
    pub_rviz_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
    ROS_DEBUG_STREAM_NAMED("viz_tools","Visualizing rviz markers on topic " << marker_topic_);      
    }
    
    return pub_rviz_marker_;
  }

}; // class

typedef boost::shared_ptr<VisualizationTools> VisualizationToolsPtr;
typedef boost::shared_ptr<const VisualizationTools> VisualizationToolsConstPtr;

} // namespace

#endif
