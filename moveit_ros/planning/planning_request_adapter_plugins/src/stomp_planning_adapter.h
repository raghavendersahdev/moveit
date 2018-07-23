//
// Created by deepthought on 22/07/18.
//

#ifndef PROJECT_STOMP_PLANNING_ADAPTER_H
#define PROJECT_STOMP_PLANNING_ADAPTER_H

/**
 * @file stomp_planner_manager.h
 * @brief This defines the stomp planning manager for MoveIt
 *
 * @author Jorge Nicho
 * @date April 5, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef STOMP_MOVEIT_STOMP_PLANNER_ADAPTER_H_
#define STOMP_MOVEIT_STOMP_PLANNER_ADAPTER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <ros/node_handle.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace stomp_moveit
{
/**
 * @class stomp_moveit::StompPlannerAdapter
 * @brief The PlannerManager implementation that loads STOMP into moveit
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class StompPlanningAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  /**
   * @brief Constructor
   */
  StompPlanningAdapter();

  virtual ~StompPlanningAdapter();

  /**
   * @brief Loads the ros parameters for each planning group and initializes the all the planners
   * @param model The robot model
   * @param ns    The parameter namespace
   * @return      True if succeeded, False otherwise
   */
  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns);

  /**
   * @brief Checks if the request can be planned for.
   * @param req
   * @return True if succeeded, False otherwise
   */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const;

  /**
   * @brief Description string getter
   * @return Description string
   */
  std::string getDescription() const
  {
    return "Stomp Planner";
  }

  /**
   * @brief Getter for a list of the available planners, usually one STOMP planner per planning group
   * @param algs List of available planners.
   */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const;

  /**
   * @brief Not applicable
   * @param pcs this argument is ignored.
   */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs);

  /**
   * @brief Provides a planning context that matches the desired plan requests specifications
   * @param planning_scene  A pointer to the planning scene
   * @param req             The motion plan request
   * @param error_code      Error code, will be set to moveit_msgs::MoveItErrorCodes::SUCCESS if it succeeded
   * @return  A pointer to the planning context
   */
  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const;

protected:
  ros::NodeHandle nh_;

  std::map<std::string, planning_interface::PlanningContextPtr> planners_; /**< The planners for each planning group */

  // the robot model
  moveit::core::RobotModelConstPtr robot_model_;

  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const
  {
    std::cout << "Hello, I am a STOMP pre-processor" << std::endl;

    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      ROS_DEBUG("Running '%s'", getDescription().c_str());
      if (!time_param_.computeTimeStamps(*res.trajectory_, req.max_velocity_scaling_factor,
                                         req.max_acceleration_scaling_factor))
        ROS_WARN("Time parametrization for the solution path failed.");
    }

    return result;
  }

private:
  trajectory_processing::IterativeParabolicTimeParameterization time_param_;
};

} /* namespace stomp_moveit */

#endif /* STOMP_MOVEIT_STOMP_PLANNER_ADAPTER_H_ */

#endif  // PROJECT_STOMP_PLANNING_ADAPTER_H
