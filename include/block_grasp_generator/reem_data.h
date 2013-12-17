/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */

/* Author: Bence Magyar
   Desc:   Parameters specific to Baxter for performing pick-place
*/

#include <block_grasp_generator/block_grasp_generator.h> // has datastructure

namespace reem_pick_place
{

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string BASE_LINK = "base_link";

// Copied from URDF \todo read straight from URDF?
static const double THUMB_JOINT_DOWN = 1.5; //close
static const double FINGER_JOINT_IN = 0.0; //close
static const double FINGER_JOINT_OUT = 3.9; //open

// robot dimensions
static const double FLOOR_TO_BASE_HEIGHT = -0.9;

block_grasp_generator::RobotGraspData loadRobotGraspData(const std::string& side)
{
  block_grasp_generator::RobotGraspData grasp_data;

  // -------------------------------
  // Convert generic grasp pose to this end effector's frame of reference
  // I think this is kind of the same as the "approach direction" // Bence: yes :D

  // Orientation
  double angle = M_PI / 2;  // turn on Z axis
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
  grasp_data.grasp_pose_to_eef_pose_.orientation.x = quat.x();
  grasp_data.grasp_pose_to_eef_pose_.orientation.y = quat.y();
  grasp_data.grasp_pose_to_eef_pose_.orientation.z = quat.z();
  grasp_data.grasp_pose_to_eef_pose_.orientation.w = quat.w();

  // Position // approach vector?
  grasp_data.grasp_pose_to_eef_pose_.position.x = -0.15;
  grasp_data.grasp_pose_to_eef_pose_.position.y = 0;
  grasp_data.grasp_pose_to_eef_pose_.position.z = 0;

  // -------------------------------
  // Create pre-grasp posture (fingers open, thumb down)
  grasp_data.pre_grasp_posture_.header.frame_id = BASE_LINK;
  grasp_data.pre_grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_data.pre_grasp_posture_.joint_names.push_back("hand_" + side + "_thumb_joint");
  grasp_data.pre_grasp_posture_.joint_names.push_back("hand_" + side + "_index_finger_joint");
  grasp_data.pre_grasp_posture_.joint_names.push_back("hand_" + side + "_middle_finger_joint");
  // Position of joints
  grasp_data.pre_grasp_posture_.points.resize(1);
  grasp_data.pre_grasp_posture_.points[0].positions.push_back(THUMB_JOINT_DOWN);
  grasp_data.pre_grasp_posture_.points[0].positions.push_back(FINGER_JOINT_OUT);
  grasp_data.pre_grasp_posture_.points[0].positions.push_back(FINGER_JOINT_OUT);

  // -------------------------------
  // Create grasp posture (fingers closed, thumb down)
  grasp_data.grasp_posture_.header.frame_id = BASE_LINK;
  grasp_data.grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_data.grasp_posture_.joint_names.push_back("hand_" + side + "_thumb_joint");
  grasp_data.grasp_posture_.joint_names.push_back("hand_" + side + "_index_finger_joint");
  grasp_data.grasp_posture_.joint_names.push_back("hand_" + side + "_middle_finger_joint");
  // Position of joints
  grasp_data.grasp_posture_.points.resize(1);
  grasp_data.grasp_posture_.points[0].positions.push_back(THUMB_JOINT_DOWN);
  grasp_data.grasp_posture_.points[0].positions.push_back(FINGER_JOINT_IN);
  grasp_data.grasp_posture_.points[0].positions.push_back(FINGER_JOINT_IN);

  // -------------------------------
  // SRDF Info
  grasp_data.base_link_ = BASE_LINK;
  grasp_data.ee_parent_link_ = "arm_" + side + "_7_link";
  grasp_data.ee_group_ = side + "_hand";
  //grasp_data.ee_joint_ = side + "_gripper_l_finger_joint"; // NOTE: only 1 joint?

  // -------------------------------
  // Nums
  grasp_data.approach_retreat_desired_dist_ = 0.3;
  grasp_data.approach_retreat_min_dist_ = 0.06;
  // distance from center point of object to end effector
  grasp_data.grasp_depth_ = 0.12;

  // generate grasps at PI/angle_resolution increments
  grasp_data.angle_resolution_ = 16;

  // Debug
  //block_grasp_generator::BlockGraspGenerator::printBlockGraspData(grasp_data);

  return grasp_data;
}



} // namespace
