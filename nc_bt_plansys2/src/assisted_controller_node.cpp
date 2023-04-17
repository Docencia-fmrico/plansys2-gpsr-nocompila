// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Assisted : public rclcpp::Node
{
public:
  Assisted()
  : rclcpp::Node("assisted_controller")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"abitobot", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"grandma", "grandma"});

    problem_expert_->addInstance(plansys2::Instance{"bedroom", "location"});
    problem_expert_->addInstance(plansys2::Instance{"gym", "location"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "location"});
    problem_expert_->addInstance(plansys2::Instance{"livingroom", "location"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom", "location"});

    problem_expert_->addInstance(plansys2::Instance{"door_lg", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door_lb", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door_lbath", "door"});
    problem_expert_->addInstance(plansys2::Instance{"door_lk", "door"});
    problem_expert_->addInstance(plansys2::Instance{"front_door", "door"});

    problem_expert_->addInstance(plansys2::Instance{"dumbell", "item"});
    problem_expert_->addInstance(plansys2::Instance{"ball", "item"});
    problem_expert_->addInstance(plansys2::Instance{"knife", "item"});

    problem_expert_->addPredicate(plansys2::Predicate("(item_at dumbell gym)"));

    problem_expert_->addPredicate(plansys2::Predicate("(item_at ball bedroom)"));

    problem_expert_->addPredicate(plansys2::Predicate("(item_at knife bedroom)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at abitobot gym)"));
    problem_expert_->addPredicate(plansys2::Predicate("(grandma_at grandma bedroom)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(robot_available abitobot)"));

    problem_expert_->addPredicate(
      plansys2::Predicate("(connected livingroom gym door_lg)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected gym livingroom door_lg)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected livingroom bedroom door_lb)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected bedroom livingroom door_lb)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected livingroom bathroom door_lbath)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected bathroom livingroom door_lbath)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected livingroom kitchen door_lk)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(connected kitchen livingroom door_lk)"));

    problem_expert_->addPredicate(plansys2::Predicate("(open door_lg)"));
    problem_expert_->addPredicate(plansys2::Predicate("(open door_lb)"));
    problem_expert_->addPredicate(plansys2::Predicate("(open door_lbath)"));
    problem_expert_->addPredicate(plansys2::Predicate("(open door_lk)"));
    problem_expert_->addPredicate(plansys2::Predicate("(close front_door)"));
    problem_expert_->addPredicate(plansys2::Predicate("(front_door_at front_door livingroom)"));

    problem_expert_->addPredicate(plansys2::Predicate("(item_not_used dumbell)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_not_used ball)"));
    problem_expert_->addPredicate(plansys2::Predicate("(item_not_used knife)"));

    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(grandma_assisted abitobot knife grandma))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Assisted>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
