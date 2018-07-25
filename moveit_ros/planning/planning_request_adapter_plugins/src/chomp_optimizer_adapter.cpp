//
// Created by deepthought on 24/07/18.
//

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>

#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>

#include <tf/transform_listener.h>

#include <moveit/robot_state/conversions.h>

using namespace chomp;

namespace default_planner_request_adapters
{
class CHOMPOptimizerAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  static const std::string DT_PARAM_NAME;

  chomp::ChompParameters params_;
  ChompPlanner chomp_interface_;
  moveit::core::RobotModelConstPtr robot_model_;

  boost::shared_ptr<tf::TransformListener> tf_;
  CHOMPOptimizerAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
  }

  virtual std::string getDescription() const
  {
    return "CHOMP Optimizer yo!!@$@#$@%$@#!!";
  }

  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const
  {
    collision_detection::CollisionDetectorAllocatorPtr hybrid_cd(
        collision_detection::CollisionDetectorAllocatorHybrid::create());

    // planning_interface::PlanningContext planningCOntext;
    // robot_model_ = planning_scene->getRobotModel();
    if (!planning_scene)
    {
      ROS_INFO_STREAM("Configuring New Planning Scene.");
      planning_scene::PlanningScenePtr planning_scene_ptr(
          new planning_scene::PlanningScene(planning_scene->getRobotModel()));
      planning_scene_ptr->setActiveCollisionDetector(hybrid_cd, true);
      // planningCOntext.setPlanningScene(planning_scene_ptr);
    }

    std::cout << "I AM INSIDE CHOMP PLANNING ADAPTER..!!@$e$@#@$^@^@^#$&#^&# " << std::endl;
    chomp::ChompParameters params_;
    params_.planning_time_limit_ = 10.0;
    params_.max_iterations_ = 200;
    params_.max_iterations_after_collision_free_ = 5;
    params_.smoothness_cost_weight_ = 0.1;
    params_.obstacle_cost_weight_ = 1.0;
    params_.learning_rate_ = 0.01;
    params_.animate_path_ = true;
    params_.add_randomness_ = false;
    params_.smoothness_cost_velocity_ = 0.0;
    params_.smoothness_cost_acceleration_ = 1.0;
    params_.smoothness_cost_jerk_ = 0.0;
    params_.hmc_discretization_ = 0.01;
    params_.hmc_stochasticity_ = 0.01;
    params_.hmc_annealing_factor_ = 0.99;
    params_.use_hamiltonian_monte_carlo_ = false;
    params_.ridge_factor_ = 0.0;
    params_.use_pseudo_inverse_ = false;
    params_.pseudo_inverse_ridge_factor_ = 1e-4;
    params_.animate_endeffector_ = false;
    params_.animate_endeffector_segment_ = std::string("r_gripper_tool_frame");
    params_.joint_update_limit_ = 0.1;
    params_.min_clearence_ = 0.2;
    params_.collision_threshold_ = 0.07;
    params_.random_jump_amount_ = 1.0;
    params_.use_stochastic_descent_ = true;

    chomp::ChompPlanner chompPlanner;

    planning_interface::MotionPlanDetailedResponse res_detailed;
    moveit_msgs::MotionPlanDetailedResponse res2;

    bool planning_success;

    if (chompPlanner.solve(planning_scene, req, params_, res2))
    {
      res_detailed.trajectory_.resize(1);
      res_detailed.trajectory_[0] = robot_trajectory::RobotTrajectoryPtr(
          new robot_trajectory::RobotTrajectory(planning_scene->getRobotModel(), "panda_arm"));

      moveit::core::RobotState start_state(planning_scene->getRobotModel());
      robot_state::robotStateMsgToRobotState(res2.trajectory_start, start_state);
      res_detailed.trajectory_[0]->setRobotTrajectoryMsg(start_state, res2.trajectory[0]);
      res_detailed.description_.push_back("plan");
      res_detailed.processing_time_ = res2.processing_time;
      res_detailed.error_code_ = res2.error_code;
      planning_success = true;
    }
    else
    {
      res_detailed.error_code_ = res2.error_code;
      planning_success = false;
    }

    std::cout << res_detailed.trajectory_[0] << " YOYO TRAJECTORY" << std::endl;

    res.error_code_ = res_detailed.error_code_;

    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ = res_detailed.processing_time_[0];
    }
    bool solved = planner(planning_scene, req, res);
    return solved;

    // CALL THE CHOMPPlanner's solve method here and populate the  request, planningSceneCOnstPtr appropriately, the
    // solver will then output a response in the Motion Plan Response
    // then finally call the planner , once this is done it should be good DO IT FOR CHOMP first and then move later to
    // STOMP, CHOMP looks less complex than STOMP!!!!!!!
  }

private:
  ros::NodeHandle nh_;
};
}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::CHOMPOptimizerAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
