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

#include <block_grasp_generator/block_grasp_generator.h>

namespace block_grasp_generator
{

// Constructor
BlockGraspGenerator::BlockGraspGenerator(VisualizationToolsPtr rviz_tools) :
  rviz_tools_(rviz_tools),
  animate_(false)
{
}

// Deconstructor
BlockGraspGenerator::~BlockGraspGenerator()
{
}

// Create all possible grasp positions for a block
bool BlockGraspGenerator::generateGrasps(const geometry_msgs::Pose& block_pose, const RobotGraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Create a transform from the block's frame (center of block) to /base_link
  tf::poseMsgToEigen(block_pose, block_global_transform_);

  // ---------------------------------------------------------------------------------------------
  // Calculate grasps in two axis in both directions
//  generateAxisGrasps( possible_grasps, X_AXIS, DOWN, grasp_data); // got no grasps with this alone
//  generateAxisGrasps( possible_grasps, X_AXIS, UP,   grasp_data); // gives some grasps... looks ugly
  generateAxisGrasps( possible_grasps, Y_AXIS, DOWN, grasp_data); // GOOD ONES!
//  generateAxisGrasps( possible_grasps, Y_AXIS, UP,   grasp_data); // gave a grasp from top... bad
  ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  // Visualize results
  visualizeGrasps(possible_grasps, block_pose, grasp_data);

  return true;
}

// Create grasp positions in one axis
bool BlockGraspGenerator::generateAxisGrasps(std::vector<moveit_msgs::Grasp>& possible_grasps, grasp_axis_t axis,
  grasp_direction_t direction, const RobotGraspData& grasp_data)
{

  // ---------------------------------------------------------------------------------------------
  // Grasp parameters

  // Create re-usable approach motion
  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
  pre_grasp_approach.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable retreat motion
  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
  post_grasp_retreat.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // ---------------------------------------------------------------------------------------------
  // Angle calculations
  double radius = grasp_data.grasp_depth_; //0.12
  double xb;
  double yb = 0.0; // stay in the y plane of the block
  double zb;
  double theta1 = 0.0; // Where the point is located around the block
  double theta2 = 0.0; // UP 'direction'

  // Gripper direction (UP/DOWN) rotation. UP set by default
  if( direction == DOWN )
  {
    theta2 = M_PI;
  }

  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------
  // Begin Grasp Generator Loop
  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------

  /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the block, then later convert it to the base link
   */
  for(int i = 0; i <= grasp_data.angle_resolution_; ++i)
  {
    // Create a Grasp message
    moveit_msgs::Grasp new_grasp;

    // Calculate grasp pose
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    Eigen::Affine3d grasp_pose;

    switch(axis)
    {
    case X_AXIS:
      grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

      grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

      break;
    case Y_AXIS:
      grasp_pose =
        Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

      grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

      break;
    case Z_AXIS:
      ROS_ERROR_STREAM_NAMED("grasp","Z Axis not implemented!");
      return false;

      break;
    }

    /* The estimated probability of success for this grasp, or some other measure of how "good" it is.
     * Here we base bias the score based on how far the wrist is from the surface, preferring a greater
     * distance to prevent wrist/end effector collision with the table
     */
    double score = sin(theta1);
    new_grasp.grasp_quality = std::max(score,0.1); // don't allow score to drop below 0.1 b/c all grasps are ok

    // Calculate the theta1 for next time
    theta1 += M_PI / grasp_data.angle_resolution_;

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;

    // PreGrasp and Grasp Postures --------------------------------------------------------------------------

    // The internal posture of the hand for the pre-grasp only positions are used
    new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;

    // The internal posture of the hand for the grasp positions and efforts are used
    new_grasp.grasp_posture = grasp_data.grasp_posture_;

    // Grasp ------------------------------------------------------------------------------------------------


    // DEBUG - show original grasp pose before tranform to gripper frame
    if( true )
    {
      tf::poseEigenToMsg(block_global_transform_ * grasp_pose, grasp_pose_msg.pose);
      rviz_tools_->publishArrow(grasp_pose_msg.pose, GREEN);
    }

    // ------------------------------------------------------------------------
    // Change grasp to frame of reference of this custom end effector

    // Convert to Eigen
    Eigen::Affine3d eef_conversion_pose;
    tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose);

    // Transform the grasp pose
    grasp_pose = grasp_pose * eef_conversion_pose;

    // ------------------------------------------------------------------------
    // Convert pose to global frame (base_link)
    tf::poseEigenToMsg(block_global_transform_ * grasp_pose, grasp_pose_msg.pose);

    // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
    new_grasp.grasp_pose = grasp_pose_msg;

    // Other ------------------------------------------------------------------------------------------------

    // the maximum contact force to use while grasping (<=0 to disable)
    new_grasp.max_contact_force = 0;

    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------
    // Approach and retreat
    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------

    // Straight down ---------------------------------------------------------------------------------------
    // With respect to the base link/world frame

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.base_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

    // Angled with pose -------------------------------------------------------------------------------------
    // Approach with respect to end effector orientation

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = 1;
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = -1;
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

  }

  return true;
}

// Show all grasps in Rviz
void BlockGraspGenerator::visualizeGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
  const geometry_msgs::Pose& block_pose, const RobotGraspData& grasp_data)
{
  if(rviz_tools_->isMuted())
  {
    ROS_DEBUG_STREAM_NAMED("grasp","Not visualizing grasps - muted.");
    return;
  }

  if( !animate_ )
  {
    ROS_DEBUG_STREAM_NAMED("grasp","Not visualizing grasps - animation set to false.");
    return;
  }

  ROS_DEBUG_STREAM_NAMED("grasp","Visualizing " << possible_grasps.size() << " grasps");

  int i = 0;
  for(std::vector<moveit_msgs::Grasp>::const_iterator grasp_it = possible_grasps.begin();
      grasp_it < possible_grasps.end(); ++grasp_it)
  {
    if( !ros::ok() )  // Check that ROS is still ok and that user isn't trying to quit
      break;

    // Make sure block is still visible
    rviz_tools_->publishBlock(block_pose, grasp_data.block_size_, false);

    ++i;

    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose " << i);

      animateGrasp(*grasp_it, grasp_data);


    // Show robot joint positions if available
    /*
      if( grasp_it->grasp_posture.position.size() > 1 )
      {
      ROS_WARN_STREAM_NAMED("temp","HAS IK SOLUTION - Positions:");
      std::copy(grasp_it->grasp_posture.position.begin(), grasp_it->grasp_posture.position.end(), std::ostream_iterator<double>(std::cout, "\n"));
      rviz_tools_->publishPlanningScene(grasp_it->grasp_posture.position);
      ros::Duration(5.0).sleep();
      }
    */

    ros::Duration(0.001).sleep();
  }
}

void BlockGraspGenerator::animateGrasp(const moveit_msgs::Grasp &grasp, const RobotGraspData& grasp_data)
{
  // Grasp Pose Variables
  geometry_msgs::Pose grasp_pose = grasp.grasp_pose.pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose, grasp_pose_eigen);

  // Pre-grasp pose variables
  geometry_msgs::Pose pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen;

  // Approach direction variables
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // Display Grasp Score
  std::string text = "Grasp Quality: " + boost::lexical_cast<std::string>(int(grasp.grasp_quality*100)) + "%";
  rviz_tools_->publishText(grasp_pose, text);

  // Convert the grasp pose into the frame of reference of the approach/retreat frame_id

  // Animate the movement - for ee approach direction
  double animation_resulution = 0.1; // the lower the better the resolution
  for(double percent = 0; percent < 1; percent += animation_resulution)
  {
    if( !ros::ok() ) // Check that ROS is still ok and that user isn't trying to quit
      break;

    // Copy original grasp pose to pre-grasp pose
    pre_grasp_pose_eigen = grasp_pose_eigen;

    // The direction of the pre-grasp
    // Calculate the current animation position based on the percent
    Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
      -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance * (1-percent),
      -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance * (1-percent),
      -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance * (1-percent)
    );

    // Decide if we need to change the approach_direction to the local frame of the end effector orientation
    if( grasp.pre_grasp_approach.direction.header.frame_id == grasp_data.ee_parent_link_ )
    {
      // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
      pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
    }
    else
    {
      pre_grasp_approach_direction_local = pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
    }

    // Update the grasp matrix usign the new locally-framed approach_direction
    pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

    // Convert eigen pre-grasp position back to regular message
    tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose);

    //rviz_tools_->publishArrow(pre_grasp_pose, BLUE);
    rviz_tools_->publishEEMarkers(pre_grasp_pose);

    ros::Duration(0.01).sleep();
  }

}


} // namespace
