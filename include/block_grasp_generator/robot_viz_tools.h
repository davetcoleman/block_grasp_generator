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

#ifndef BLOCK_GRASP_GENERATOR__ROBOT_VIZ_TOOLS_
#define BLOCK_GRASP_GENERATOR__ROBOT_VIZ_TOOLS_

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_tools/solid_primitive_dims.h>

// ROS
#include <tf_conversions/tf_eigen.h>

// Boost
#include <boost/shared_ptr.hpp>

// Messages
#include <std_msgs/ColorRGBA.h>

namespace block_grasp_generator
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string COLLISION_TOPIC = "/collision_object";
static const std::string ATTACHED_COLLISION_TOPIC = "/attached_collision_object";

enum rviz_colors { RED, GREEN, BLUE, GREY, WHITE };

class RobotVizTools
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher rviz_marker_pub_;
  ros::Publisher pub_collision_obj_; // for MoveIt
  ros::Publisher pub_attach_collision_obj_; // for MoveIt

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
  //  geometry_msgs::Pose grasp_pose_to_eef_pose_;
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
  RobotVizTools(std::string marker_topic, std::string ee_group_name, std::string planning_group_name,
    std::string base_link, double floor_to_base_height,
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor) :
    planning_scene_monitor_(planning_scene_monitor)
  {
    // Pass to next contructor
    RobotVizTools(marker_topic, ee_group_name, planning_group_name, base_link, floor_to_base_height);
  }

  /**
   * \brief Constructor w/o planning scene passed in
   */
  RobotVizTools(std::string marker_topic, std::string ee_group_name, std::string planning_group_name,
    std::string base_link, double floor_to_base_height) :
    marker_topic_(marker_topic),
    ee_group_name_(ee_group_name),
    planning_group_name_(planning_group_name),
    base_link_(base_link),
    floor_to_base_height_(floor_to_base_height),
    marker_lifetime_(ros::Duration(30.0)),
    nh_("~"),
    muted_(false)
  {

    // Load EE Markers
    if( !loadEEMarker() )
      ROS_ERROR_STREAM_NAMED("robot_viz","Unable to publish EE marker");

    // Rviz Visualizations
    rviz_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1);
    ROS_DEBUG_STREAM_NAMED("robot_viz","Visualizing rviz markers on topic " << marker_topic_);

    // Collision object creator
    pub_collision_obj_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 10);
    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing collision objects on topic " << COLLISION_TOPIC);

    // Collision object attacher
    pub_attach_collision_obj_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>
      (ATTACHED_COLLISION_TOPIC, 10);
    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing attached collision objects on topic "
      << ATTACHED_COLLISION_TOPIC);

    loadRvizMarkers();

    // Wait
    ros::spinOnce();
    ros::Duration(0.1).sleep(); // TODO: better way of doing this?
  }

  /**
   * \brief Deconstructor
   */
  ~RobotVizTools()
  {
  }

  /**
   * \brief Pre-load rviz markers for better efficiency
   */
  void loadRvizMarkers()
  {
    // Load arrow ----------------------------------------------------

    arrow_marker_.header.frame_id = base_link_;
    // Set the namespace and id for this marker.  This serves to create a unique ID
    arrow_marker_.ns = "Arrow";
    // Set the marker type.
    arrow_marker_.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD and DELETE
    arrow_marker_.action = visualization_msgs::Marker::ADD;
    // Size
    arrow_marker_.scale.x = 0.05; //0.025; // arrow width - but i would call this the length
    arrow_marker_.scale.y = 0.005; // arrow height
    arrow_marker_.scale.z = 0.005; // arrow length
    // Lifetime
    arrow_marker_.lifetime = marker_lifetime_;

    // Load Block ----------------------------------------------------
    block_marker_.header.frame_id = base_link_;
    // Set the namespace and id for this marker.  This serves to create a unique ID
    block_marker_.ns = "Block";
    // Set the marker action.  Options are ADD and DELETE
    block_marker_.action = visualization_msgs::Marker::ADD;
    // Set the marker type.
    block_marker_.type = visualization_msgs::Marker::CUBE;
    // Lifetime
    block_marker_.lifetime = marker_lifetime_;

    // Load Sphere -------------------------------------------------
    sphere_marker_.header.frame_id = base_link_;
    // Set the namespace and id for this marker.  This serves to create a unique ID
    sphere_marker_.ns = "Sphere";
    // Set the marker type.
    sphere_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    // Set the marker action.  Options are ADD and DELETE
    sphere_marker_.action = visualization_msgs::Marker::ADD;
    // Marker group position and orientation
    sphere_marker_.pose.position.x = 0;
    sphere_marker_.pose.position.y = 0;
    sphere_marker_.pose.position.z = 0;
    sphere_marker_.pose.orientation.x = 0.0;
    sphere_marker_.pose.orientation.y = 0.0;
    sphere_marker_.pose.orientation.z = 0.0;
    sphere_marker_.pose.orientation.w = 1.0;
    // Sphere size
    sphere_marker_.scale.x = 0.01;
    sphere_marker_.scale.y = 0.01;
    sphere_marker_.scale.z = 0.01;
    // Color
    sphere_marker_.color = getColor( BLUE );
    // Create a sphere point
    geometry_msgs::Point point_a;
    // Add the point pair to the line message
    sphere_marker_.points.push_back( point_a );
    sphere_marker_.colors.push_back( getColor( BLUE ) );

    // Load Text ----------------------------------------------------
    text_marker_.header.frame_id = base_link_;
    // Set the namespace and id for this marker.  This serves to create a unique ID
    text_marker_.ns = "Text";
    // Set the marker action.  Options are ADD and DELETE
    text_marker_.action = visualization_msgs::Marker::ADD;
    // Set the marker type.
    text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // Lifetime
    text_marker_.lifetime = marker_lifetime_;
  }

  /**
   * \brief Load a planning scene monitor if one was not passed into the constructor
   * \return true if successful in loading
   */
  bool loadPlanningSceneMonitor()
  {
    // ---------------------------------------------------------------------------------------------
    // Create planning scene monitor
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    if (planning_scene_monitor_->getPlanningScene())
    {
      //planning_scene_monitor_->startWorldGeometryMonitor();
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        "dave_planning_scene");
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("rviz_tools","Planning scene not configured");
      return false;
    }

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    return true;
  }


  /**  NEEDS TO BE PORTED TO THE NEW MOVEIT FUNCTIONALITY
   * \brief Move the robot arm to the ik solution in rviz
   * \param joint_values - the in-order list of values to set the robot's joints
   * \return true if it is successful
   *
   bool publishPlanningScene(std::vector<double> joint_values)
   {
   if(muted_)
   return true; // this function will only work if we have loaded the publishers

   // Load planning scene monitor if one was not already passed in
   if(!planning_scene_monitor_)
   if(!loadPlanningSceneMonitor())
   return false;

   ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing planning scene");

   // Output debug
   //ROS_INFO_STREAM_NAMED("robot_viz","Joint values being sent to planning scene:");
   //std::copy(joint_values.begin(),joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));

   // Update planning scene
   robot_state::JointStateGroup* joint_state_group = planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst()
   .getJointStateGroup(planning_group_name_);
   joint_state_group->setVariableValues(joint_values);

   //    planning_scene_monitor_->updateFrameTransforms();
   planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

   return true;
   }
  */

  /**
   * \brief Call this once at begining to load the robot marker
   * \return true if it is successful
   */
  bool loadEEMarker()
  {
    // Load planning scene monitor if one was not already passed in
    if(!planning_scene_monitor_)
      if(!loadPlanningSceneMonitor())
        return false;

    // -----------------------------------------------------------------------------------------------
    // Get end effector group

    // Create color to use for EE markers
    std_msgs::ColorRGBA marker_color = getColor( GREY );

    // Get robot state
    robot_model::RobotModelConstPtr robot_model = planning_scene_monitor_->getRobotModel();

    // Get joint state group
    //robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup(ee_group_name_);
    const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(ee_group_name_);

    if( joint_model_group == NULL ) // make sure EE_GROUP exists
    {
      ROS_ERROR_STREAM_NAMED("robot_viz","Unable to find joint model group " << ee_group_name_ );
      return false;
    }

    // Get link names that are in end effector
    const std::vector<std::string> &ee_link_names = joint_model_group->getLinkModelNames();

    ROS_DEBUG_STREAM_NAMED("robot_viz","Number of links in group " << ee_group_name_ << ": " << ee_link_names.size());

    // Robot Interaction - finds the end effector associated with a planning group
    robot_interaction::RobotInteraction robot_interaction( planning_scene_monitor_->getRobotModel() );

    // Decide active end effectors
    robot_interaction.decideActiveEndEffectors(planning_group_name_);

    // Get active EE
    std::vector<robot_interaction::RobotInteraction::EndEffector> active_eef =
      robot_interaction.getActiveEndEffectors();

    ROS_DEBUG_STREAM_NAMED("robot_viz","Number of active end effectors: " << active_eef.size());
    if( !active_eef.size() )
    {
      ROS_ERROR_STREAM_NAMED("robot_viz","No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
      return false;
    }

    // Just choose the first end effector \todo better logic?
    robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

    // -----------------------------------------------------------------------------------------------
    // Get EE link markers for Rviz
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    robot_state.getRobotMarkers(ee_marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
    ROS_DEBUG_STREAM_NAMED("robot_viz","Number of rviz markers in end effector: " << ee_marker_array_.markers.size());

    // Change pose from Eigen to TF
    try
    {
      ee_parent_link_ = eef.parent_link; // save the name of the link for later use
      // FUTURE: tf::poseEigenToTF(robot_state.getGlobalLinkTransform(eef.parent_link), tf_root_to_link_);
      tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link_);
    }
    catch(...)
    {
      ROS_ERROR_STREAM_NAMED("robot_viz","Didn't find link state for " << ee_parent_link_);
    }
    ROS_ERROR_STREAM_NAMED("temp","eef parent link = "<< ee_parent_link_);

    /*
    // Offset from gasp_pose to end effector
    static const double X_OFFSET = 0; //-0.15;
    // Allow a transform from our pose to the end effector position
    // TODO: make this more generic for arbitrary grippers
    // Orientation
    double angle = 0; //M_PI / 2;  // turn on Z axis
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    grasp_pose_to_eef_pose_.position.x = X_OFFSET;
    grasp_pose_to_eef_pose_.position.y = 0;
    grasp_pose_to_eef_pose_.position.z = 0;
    grasp_pose_to_eef_pose_.orientation.x = quat.x();
    grasp_pose_to_eef_pose_.orientation.y = quat.y();
    grasp_pose_to_eef_pose_.orientation.z = quat.z();
    grasp_pose_to_eef_pose_.orientation.w = quat.w();
    */

    // Copy original marker poses to a vector
    for (std::size_t i = 0 ; i < ee_marker_array_.markers.size() ; ++i)
    {
      marker_poses_.push_back( ee_marker_array_.markers[i].pose );
    }

    return true;
  }

  /**
   * \brief Publish an end effector to rviz
   * \return true if it is successful
   */
  bool publishEEMarkers(const geometry_msgs::Pose &grasp_pose)
  {
    if(muted_)
      return true;

    //ROS_INFO_STREAM("Mesh (" << grasp_pose.position.x << ","<< grasp_pose.position.y << ","<< grasp_pose.position.z << ")");

    // -----------------------------------------------------------------------------------------------
    // Process each link of the end effector
    for (std::size_t i = 0 ; i < ee_marker_array_.markers.size() ; ++i)
    {
      // Make sure ROS is still spinning
      if( !ros::ok() )
        break;

      // Header
      ee_marker_array_.markers[i].header.frame_id = base_link_;
      ee_marker_array_.markers[i].header.stamp = ros::Time::now();

      // Options
      ee_marker_array_.markers[i].lifetime = marker_lifetime_;

      // Options for meshes
      if( ee_marker_array_.markers[i].type == visualization_msgs::Marker::MESH_RESOURCE )
      {
        ee_marker_array_.markers[i].mesh_use_embedded_materials = true;
      }

      // -----------------------------------------------------------------------------------------------
      // Do some math for the offset
      // grasp_pose             - our generated grasp
      // markers[i].pose        - an ee link's pose relative to the whole end effector
      // REMOVED grasp_pose_to_eef_pose_ - the offset from the grasp pose to eef_pose - probably nothing
      tf::Pose tf_root_to_marker;
      tf::Pose tf_root_to_mesh;
      tf::Pose tf_pose_to_eef;

      // Simple conversion from geometry_msgs::Pose to tf::Pose
      tf::poseMsgToTF(grasp_pose, tf_root_to_marker);
      tf::poseMsgToTF(marker_poses_[i], tf_root_to_mesh);
      // tf::poseMsgToTF(grasp_pose_to_eef_pose_, tf_pose_to_eef); // \todo REMOVE

      // Conversions
      tf::Pose tf_eef_to_mesh = tf_root_to_link_.inverse() * tf_root_to_mesh;
      // REMOVED tf::Pose tf_marker_to_mesh = tf_pose_to_eef * tf_eef_to_mesh;
      //tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_marker_to_mesh;
      tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_eef_to_mesh;
      tf::poseTFToMsg(tf_root_to_mesh_new, ee_marker_array_.markers[i].pose);
      // -----------------------------------------------------------------------------------------------

      //ROS_INFO_STREAM("Marker " << i << ":\n" << ee_marker_array_.markers[i]);

      rviz_marker_pub_.publish( ee_marker_array_.markers[i] );
      ros::Duration(0.01).sleep();  // Sleep to prevent markers from being 'skipped' in rviz
    }

    return true;
  }

  /**
   * \brief Publish an marker of a mesh to rviz
   * \return true if it is successful
   */
  bool publishMesh(double x, double y, double z, double qx, double qy, double qz, double qw )
  {
    if(muted_)
      return true; // this function will only work if we have loaded the publishers

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing mesh");

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Mesh";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://clam_description/stl/gripper_base_link.STL";
    ROS_ERROR_STREAM_NAMED("temp","TODO - add mesh resource");

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color = getColor( RED );

    // Make line color
    std_msgs::ColorRGBA color = getColor( RED );

    // Point
    geometry_msgs::Point point_a;
    point_a.x = x;
    point_a.y = y;
    point_a.z = z;
    //ROS_INFO_STREAM("Publishing marker \n" << point_a );

    // Add the point pair to the line message
    marker.points.push_back( point_a );
    marker.colors.push_back( color );

    marker.lifetime = marker_lifetime_;

    rviz_marker_pub_.publish( marker );

    return true;
  }

  /**
   * \brief Publish an marker of a sphere to rviz
   * \return true if it is successful
   */
  bool publishSphere(const geometry_msgs::Pose &pose)
  {
    if(muted_)
      return true; // this function will only work if we have loaded the publishers

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    sphere_marker_.header.stamp = ros::Time::now();

    static int id = 0;
    sphere_marker_.id = ++id;

    sphere_marker_.lifetime = marker_lifetime_;

    // Update the single point with new pose
    sphere_marker_.points[0] = pose.position;
    
    // Publish
    rviz_marker_pub_.publish( sphere_marker_ );

    return true;
  }

  /**
   * \brief Publish an marker of an arrow to rviz
   * \return true if it is successful
   */
  bool publishArrow(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE)
  {
    if(muted_)
      return true;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    arrow_marker_.header.stamp = ros::Time::now();

    static int id = 0;
    arrow_marker_.id = ++id;

    arrow_marker_.pose = pose;

    arrow_marker_.color = getColor(color);

    rviz_marker_pub_.publish( arrow_marker_ );
    //    ros::Duration(0.01).sleep(); // Sleep to prevent markers from being 'skipped' in rviz

    return true;
  }

  /**
   * \brief Publish an marker of a block to Rviz
   * \return true if it is successful
   */
  bool publishBlock(const geometry_msgs::Pose &pose, const double &block_size, const bool isRed)
  {
    if(muted_)
      return true;

    //ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing block");

    // Set the timestamp
    block_marker_.header.stamp = ros::Time::now();

    static int id = 0;
    block_marker_.id = ++id;

    // Set the pose
    block_marker_.pose = pose;

    // Set marker size
    block_marker_.scale.x = block_size;
    block_marker_.scale.y = block_size;
    block_marker_.scale.z = block_size;

    // Set marker color
    if(isRed)
    {
      block_marker_.color = getColor( RED );
    }
    else
    {
      block_marker_.color = getColor( GREEN );
    }

    rviz_marker_pub_.publish( block_marker_ );
    //ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz

    return true;
  }

  /**
   * \brief Publish an marker of a text to Rviz
   * \return true if it is successful
   */
  bool publishText(const geometry_msgs::Pose &pose, const std::string &text, const rviz_colors &color = WHITE)
  {
    if(muted_)
      return true;

    ROS_DEBUG_STREAM_NAMED("robot_viz","Publishing text");

    text_marker_.id = 0;

    text_marker_.header.stamp = ros::Time::now();
    text_marker_.text = text;
    text_marker_.pose = pose;
    text_marker_.color = getColor( color );
    text_marker_.scale.z = 0.01;    // only z is required (size of an "A")

    rviz_marker_pub_.publish( text_marker_ );

    return true;
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
  }

  /**
   * @brief Get the planning scene monitor that this class is using
   * @param planning_scene_monitor
   * @return true if successful
   */
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    // Load planning scene monitor if one was not already passed in
    if(!planning_scene_monitor_)
      if(!loadPlanningSceneMonitor())
        ROS_ERROR_STREAM_NAMED("","Unable to get planning scene");

    return planning_scene_monitor_;
  }

  /**
   * \brief Remove a collision object from the planning scene
   * \param Name of object
   */
  void cleanupCO(std::string name)
  {
    // Clean up old collision objects
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = base_link_;
    co.id = name;
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    ros::WallDuration(0.1).sleep();
    pub_collision_obj_.publish(co);
    ros::WallDuration(0.1).sleep();
    pub_collision_obj_.publish(co);
  }

  void cleanupACO(const std::string& name)
  {
    // Clean up old attached collision object
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.header.stamp = ros::Time::now();
    aco.object.header.frame_id = base_link_;

    //aco.object.id = name;
    aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

    aco.link_name = ee_parent_link_;

    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);
    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);

  }
  void attachCO(const std::string& name)
  {
    // Clean up old attached collision object
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.header.stamp = ros::Time::now();
    aco.object.header.frame_id = base_link_;

    aco.object.id = name;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;

    // Link to attach the object to
    aco.link_name = ee_parent_link_;

    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);
    ros::WallDuration(0.1).sleep();
    pub_attach_collision_obj_.publish(aco);

  }

  void publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name, double block_size)
  {
    moveit_msgs::CollisionObject collision_obj;
    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = base_link_;
    collision_obj.id = block_name;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = block_size;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = block_size;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = block_size;
    collision_obj.primitive_poses.resize(1);
    collision_obj.primitive_poses[0] = block_pose;

    //ROS_INFO_STREAM_NAMED("pick_place","CollisionObject: \n " << collision_obj);

    pub_collision_obj_.publish(collision_obj);

    ROS_DEBUG_STREAM_NAMED("simple_pick_place","Published collision object " << block_name);
  }

  void publishCollisionWall(double x, double y, double angle, double width, const std::string name)
  {
    moveit_msgs::CollisionObject collision_obj;
    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = base_link_;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

    geometry_msgs::Pose rec_pose;

    // ----------------------------------------------------------------------------------
    // Name
    collision_obj.id = name;

    double depth = 0.1;
    double height = 2.5;

    // Position
    rec_pose.position.x = x;
    rec_pose.position.y = y;
    rec_pose.position.z = height / 2 + floor_to_base_height_;

    // Size
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;
    // ----------------------------------------------------------------------------------

    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    rec_pose.orientation.x = quat.x();
    rec_pose.orientation.y = quat.y();
    rec_pose.orientation.z = quat.z();
    rec_pose.orientation.w = quat.w();

    collision_obj.primitive_poses.resize(1);
    collision_obj.primitive_poses[0] = rec_pose;

    pub_collision_obj_.publish(collision_obj);
  }

  void publishCollisionTable(double x, double y, double angle, double width, double height,
    double depth, const std::string name)
  {
    geometry_msgs::Pose table_pose;

    // Position
    table_pose.position.x = x;
    table_pose.position.y = y;
    table_pose.position.z = height / 2 + floor_to_base_height_;

    // Orientation
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    table_pose.orientation.x = quat.x();
    table_pose.orientation.y = quat.y();
    table_pose.orientation.z = quat.z();
    table_pose.orientation.w = quat.w();

    moveit_msgs::CollisionObject collision_obj;
    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = base_link_;
    collision_obj.id = name;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

    // Size
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;

    collision_obj.primitive_poses.resize(1);
    collision_obj.primitive_poses[0] = table_pose;

    pub_collision_obj_.publish(collision_obj);
  }

  /**
   * \brief Get the RGB value of standard colors
   * \param color - a enum pre-defined name of a color
   * \return the RGB message equivalent
   */
  std_msgs::ColorRGBA getColor(const rviz_colors &color)
  {
    std_msgs::ColorRGBA result;
    result.a = 0.8;
    switch(color)
    {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;      
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;      
      break;
    case GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      break;
    case WHITE:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      break;
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
    }

    return result;
  }


}; // class

typedef boost::shared_ptr<RobotVizTools> RobotVizToolsPtr;
typedef boost::shared_ptr<const RobotVizTools> RobotVizToolsConstPtr;

} // namespace

#endif
