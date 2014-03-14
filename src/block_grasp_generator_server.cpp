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
   Desc:   Grasp generator action server
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/server/simple_action_server.h>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>
#include <block_grasp_generator/GenerateBlockGraspsAction.h>


// Baxter specific properties
#include <block_grasp_generator/reem_data.h>
#include <block_grasp_generator/custom_environment2.h>

namespace block_grasp_generator
{

  class GraspGeneratorServer
  {
  private:
    // A shared node handle
    ros::NodeHandle nh_;

    // Action server
    actionlib::SimpleActionServer<block_grasp_generator::GenerateBlockGraspsAction> as_;
    block_grasp_generator::GenerateBlockGraspsResult result_;

    // Grasp generator
    block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

    // class for publishing stuff to rviz
    block_grasp_generator::VisualizationToolsPtr visual_tools_;

    // robot-specific data for generating grasps
    block_grasp_generator::RobotGraspData grasp_data_;

    // which arm are we using
    std::string side_;
    std::string planning_group_name_;

  public:

    // Constructor
    GraspGeneratorServer(const std::string &name, const std::string &side)
      : nh_("~")
      , as_(nh_, name, boost::bind(&block_grasp_generator::GraspGeneratorServer::executeCB, this, _1), false)
      , side_(side)
      , planning_group_name_(side_+"_arm")
    {
      // ---------------------------------------------------------------------------------------------
      // Load grasp data specific to our robot
      grasp_data_ = reem_pick_place::loadRobotGraspData(side_); // Load robot specific data

      // ---------------------------------------------------------------------------------------------
      // Load the Robot Viz Tools for publishing to Rviz
      visual_tools_.reset(new block_grasp_generator::VisualizationTools(reem_pick_place::BASE_LINK));
      visual_tools_->setLifetime(120.0);
      visual_tools_->setMuted(false);
      visual_tools_->setEEGroupName(grasp_data_.ee_group_);
      visual_tools_->setPlanningGroupName(planning_group_name_);

      // ---------------------------------------------------------------------------------------------
      // Load grasp generator
      block_grasp_generator_.reset( new block_grasp_generator::BlockGraspGenerator(visual_tools_) );
      as_.start();
    }

    void executeCB(const block_grasp_generator::GenerateBlockGraspsGoalConstPtr &goal)
    {
      // ---------------------------------------------------------------------------------------------
      // Remove previous results
      result_.grasps.clear();

      // ---------------------------------------------------------------------------------------------
      // Set object width and generate grasps
      grasp_data_.block_size_ = goal->width;
      block_grasp_generator_->generateGrasps(goal->pose, grasp_data_, result_.grasps);

      // ---------------------------------------------------------------------------------------------
      // Publish results
      as_.setSucceeded(result_);
    }

  };
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_server");
  block_grasp_generator::GraspGeneratorServer grasp_generator_server("generate", "right");
  ros::spin();
  return 0;
}
