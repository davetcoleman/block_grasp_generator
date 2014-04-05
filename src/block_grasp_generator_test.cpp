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

/* Author: Dave Coleman
   Desc:   Tests the grasp generator
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Grasp generation
#include <block_grasp_generator/block_grasp_generator.h>

// Baxter specific properties
#include <block_grasp_generator/baxter_data.h>
#include <block_grasp_generator/custom_environment2.h>

namespace baxter_pick_place
{

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  block_grasp_generator::BlockGraspGeneratorPtr block_grasp_generator_;

  // class for publishing stuff to rviz
  moveit_visual_tools::VisualizationToolsPtr visual_tools_;

  // robot-specific data for generating grasps
  block_grasp_generator::RobotGraspData grasp_data_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests)
    : nh_("~"),
      arm_("right"),
      planning_group_name_(arm_+"_arm")
  {
    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_ = baxter_pick_place::loadRobotGraspData(arm_, BLOCK_SIZE); // Load robot specific data

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::VisualizationTools(baxter_pick_place::BASE_LINK));
    visual_tools_->setLifetime(120.0);
    visual_tools_->setMuted(false);
    visual_tools_->setEEGroupName(grasp_data_.ee_group_);
    visual_tools_->setPlanningGroupName(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    block_grasp_generator_.reset( new block_grasp_generator::BlockGraspGenerator(visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random blocks
    geometry_msgs::Pose block_pose;
    std::vector<moveit_msgs::Grasp> possible_grasps;

    // Allow ROS to catchup
    ros::Duration(2.0).sleep();

    // Loop
    int i = 0;
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","Adding random block " << i+1 << " of " << num_tests);

      //generateRandomBlock(block_pose);
      generateTestBlock(block_pose);

      possible_grasps.clear();
      block_grasp_generator_->generateGrasps( block_pose, grasp_data_, possible_grasps);

      // Test if done
      ++i;
      if( i >= num_tests )
        break;
    }
  }

  void generateTestBlock(geometry_msgs::Pose& block_pose)
  {
    // Position
    geometry_msgs::Pose start_block_pose;
    geometry_msgs::Pose end_block_pose;

    start_block_pose.position.x = 0.4;
    start_block_pose.position.y = -0.2;
    start_block_pose.position.z = 0.0;

    end_block_pose.position.x = 0.25;
    end_block_pose.position.y = 0.15;
    end_block_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_block_pose.orientation.x = quat.x();
    start_block_pose.orientation.y = quat.y();
    start_block_pose.orientation.z = quat.z();
    start_block_pose.orientation.w = quat.w();

    angle = M_PI / 1.1;
    quat = Eigen::Quaterniond(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    end_block_pose.orientation.x = quat.x();
    end_block_pose.orientation.y = quat.y();
    end_block_pose.orientation.z = quat.z();
    end_block_pose.orientation.w = quat.w();

    // Choose which block to test
    block_pose = start_block_pose;

    visual_tools_->publishBlock( block_pose, BLOCK_SIZE, true );
  }

  void generateRandomBlock(geometry_msgs::Pose& block_pose)
  {
    // Position
    block_pose.position.x = fRand(0.1,0.9); //0.55);
    block_pose.position.y = fRand(-0.28,0.28);
    block_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI * fRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 1;

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  baxter_pick_place::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
