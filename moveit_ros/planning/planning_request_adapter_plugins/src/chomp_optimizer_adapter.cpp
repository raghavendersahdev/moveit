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

using namespace chomp;

namespace default_planner_request_adapters
{
class CHOMPOptimizerAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  static const std::string DT_PARAM_NAME;
  static const std::string JIGGLE_PARAM_NAME;
  static const std::string ATTEMPTS_PARAM_NAME;
  chomp::ChompParameters params_;

  CHOMPOptimizerAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
    /*params_.planning_time_limit_ = 10.0;
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
    params_.collision_threshold_= 0.07;
    params_.random_jump_amount_ = 1.0;
    params_.use_stochastic_descent_ = true;*/
  }

  virtual std::string getDescription() const
  {
    return "Fix Start State In Collision";
  }

  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const
  {
    chomp::ChompPlanner chompPlanner();

    // CALL THE CHOMPPlanner's solve method here and populate the  request, planningSceneCOnstPtr appropriately, the
    // solver will then output a response in the Motion Plan Response
    // then finally call the planner , once this is done it should be good DO IT FOR CHOMP first and then move later to
    // STOMP, CHOMP looks less complex than STOMP!!!!!!!
  }

private:
  ros::NodeHandle nh_;
  double max_dt_offset_;
  double jiggle_fraction_;
  int sampling_attempts_;
};

const std::string CHOMPOptimizerAdapter::DT_PARAM_NAME = "start_state_max_dt";
const std::string CHOMPOptimizerAdapter::JIGGLE_PARAM_NAME = "jiggle_fraction";
const std::string CHOMPOptimizerAdapter::ATTEMPTS_PARAM_NAME = "max_sampling_attempts";
}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::CHOMPOptimizerAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
