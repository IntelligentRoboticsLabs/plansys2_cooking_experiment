// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>
#include <random>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Cooking : public rclcpp::Node
{
public:
  Cooking()
  : rclcpp::Node("coocking_controller"),
    robot_location_(),
    generator_(std::random_device()()),
    distribution_dish_(1, 3),
    distribution_num_dishes_(2, 2)  // Fixed to 2 dishes
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();
    generate_new_mission();

    robot_location_["r2d2"] = "cooking_zone";
    robot_location_["c3po"] = "cooking_zone";
    robot_location_["bb8"] = "cooking_zone";
    problem_expert_->addPredicate(
      plansys2::Predicate("(robot_at r2d2 " + robot_location_["r2d2"] +")"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full r2d2)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(robot_at c3po " + robot_location_["c3po"] +")"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full c3po)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(robot_at bb8 " + robot_location_["bb8"] +")"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full bb8)"));

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"c3po", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"bb8", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"fridge_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"pantry_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"watertap_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"cooking_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"recharge_zone", "zone"});

    problem_expert_->addPredicate(plansys2::Predicate("(battery_full r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full c3po)"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full bb8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_cooking_zone cooking_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_fridge_zone fridge_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_fridge_zone fridge_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_watertap_zone watertap_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_recharge_zone recharge_zone)"));
   }

  void generate_new_mission()
  {
    int num_dishes = distribution_num_dishes_(generator_);

    std::vector<std::string> goals(num_dishes, "");
    std::list<int> used_dishes;
    for (int i = 0; i < num_dishes; i++) {
      int new_dish = distribution_dish_(generator_);
      while (std::find(used_dishes.begin(), used_dishes.end(), new_dish) != used_dishes.end()) {
        new_dish = distribution_dish_(generator_);
      }

      used_dishes.push_back(new_dish);

      switch (new_dish)
      {
      case 1:  // omelette
        RCLCPP_INFO_STREAM(get_logger(), "\tI will cook an omelette");
        problem_expert_->addInstance(plansys2::Instance{"eggs", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"oil", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"salt", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"omelette", "dish"});
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at eggs fridge_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at oil pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at salt pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_egg eggs)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_oil oil)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_salt salt)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_omelette omelette)"));

        goals[i] = "(dish_prepared omelette)";
        break;
      case 2:  // cake
        RCLCPP_INFO_STREAM(get_logger(), "\tI will cook a cake");
        problem_expert_->addInstance(plansys2::Instance{"eggs", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"flour", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"sugar", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"cake", "dish"});
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at eggs fridge_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at flour pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at sugar pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_egg eggs)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_flour flour)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_sugar sugar)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_cake cake)"));

        goals[i] = "(dish_prepared cake)";
        break;

      case 3:  // cake
        RCLCPP_INFO_STREAM(get_logger(), "\tI will cook spaghetti");
        problem_expert_->addInstance(plansys2::Instance{"pasta", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"water", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"oil", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"salt", "ingredient"});
        problem_expert_->addInstance(plansys2::Instance{"spaghetti", "dish"});
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at water watertap_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at pasta pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at oil pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(ingredient_at salt pantry_zone)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_water water)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_oil oil)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_salt salt)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_pasta pasta)"));
        problem_expert_->addPredicate(plansys2::Predicate("(is_spaghetti spaghetti)"));

        goals[i] = "(dish_prepared spaghetti)";
        /* code */
        break;

      default:
         RCLCPP_ERROR_STREAM(get_logger(), "\tNo dish for that id");
        break;
      }
    }

    std::string goal_instr = "(and";
    for (const auto & goal : goals) {
      goal_instr = goal_instr + goal;
    }
    goal_instr = goal_instr + ")";
    problem_expert_->setGoal(plansys2::Goal(goal_instr));
  }

  std::string status_to_string(int8_t status) {
    switch (status)
    {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      return "NOT_EXECUTED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
      return "EXECUTING";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      return "FAILED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      return "SUCCEEDED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      return "CANCELLED";
      break;
    default:
      return "UNKNOWN";
      break;
    }
  }

  void step()
  {
    battery_level_["r2d2"] = battery_level_["r2d2"] - 0.1;
    battery_level_["c3po"] = battery_level_["c3po"] - 0.1;
    battery_level_["bb8"] = battery_level_["bb8"] - 0.1;
    RCLCPP_INFO(
      get_logger(), "Battery level = [%lf, %lf, %lf]",
      battery_level_["r2d2"], battery_level_["c3po"], battery_level_["bb8"]);

    if (!executor_client_->execute_and_check_plan()) {
      RCLCPP_INFO(get_logger(), "========================= PLAN FINISHED ==================");

      auto result = executor_client_->getResult();
      if (result.has_value()) {
        // RCLCPP_INFO_STREAM(get_logger(), "Plan succesful: " << result.value().success);
        for (const auto & action_info : result.value().action_execution_status) {
                    std::string args;
          rclcpp::Time start_stamp = action_info.start_stamp;
          rclcpp::Time status_stamp = action_info.status_stamp;
          for (const auto & arg : action_info.arguments) {
            args = args + " " + arg;
          }
          RCLCPP_INFO_STREAM(
            get_logger(), "Action: " << action_info.action << args << " " <<
            status_to_string(action_info.status) << " " <<
            (status_stamp - start_stamp).seconds() << " secs");
        }
      } else {
        RCLCPP_WARN(get_logger(), "No result for this plan");
      }

      problem_expert_->clearKnowledge();
      init_knowledge();
      generate_new_mission();


      bool found_r2d2 = false;
      bool found_c3po = false;
      bool found_bb8 = false;
      std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
      for (const auto & predicate : predicates) {
        if (predicate.name == "robot_at" && predicate.parameters[1].name == "r2d2") {
         found_r2d2 = true;
        }
        if (predicate.name == "robot_at" && predicate.parameters[1].name == "c3po") {
         found_c3po = true;
        }
        if (predicate.name == "robot_at" && predicate.parameters[1].name == "bb8") {
         found_bb8 = true;
        }
      }

      if (!found_r2d2) {
        RCLCPP_INFO_STREAM(
          get_logger(), "Restoring robot location to [" << robot_location_["r2d2"] + "]");
        problem_expert_->addPredicate(
          plansys2::Predicate("(robot_at r2d2 " + robot_location_["r2d2"] +")"));
      }
      if (!found_c3po) {
        RCLCPP_INFO_STREAM(
          get_logger(), "Restoring robot location to [" << robot_location_["c3po"] + "]");
        problem_expert_->addPredicate(
          plansys2::Predicate("(robot_at c3po " + robot_location_["c3po"] +")"));
      }
      if (!found_bb8) {
        RCLCPP_INFO_STREAM(
          get_logger(), "Restoring robot location to [" << robot_location_["bb8"] + "]");
        problem_expert_->addPredicate(
          plansys2::Predicate("(robot_at bb8 " + robot_location_["bb8"] +")"));
      }

      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (!plan.has_value()) {
        std::cout << "Could not find plan to reach goal " <<
          parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      }

      if (!executor_client_->start_plan_execution(plan.value())) {
        RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
      }
    } else {
      std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
      for (const auto & predicate : predicates) {
        if (predicate.name == "robot_at"  && predicate.parameters[1].name == "r2d2") {
          robot_location_["r2d2"] = predicate.parameters[1].name;
        }
        if (predicate.name == "robot_at"  && predicate.parameters[1].name == "c3po") {
          robot_location_["c3po"] = predicate.parameters[1].name;
          // std::cerr << "Robot at " << robot_location_ << std::endl;
        }
        if (predicate.name == "robot_at"  && predicate.parameters[1].name == "bb8") {
          robot_location_["bb8"] = predicate.parameters[1].name;
          // std::cerr << "Robot at " << robot_location_ << std::endl;
        }
      }

      auto feedback = executor_client_->getFeedBack();

      if (!feedback.action_execution_status.empty() && battery_level_["r2d2"] < 10.0) {
        problem_expert_->removePredicate(plansys2::Predicate("(battery_full r2d2)"));
        RCLCPP_INFO(get_logger(), "**********************************************************");
        RCLCPP_INFO(get_logger(), "********************** BATTERY LOW r2d2 ******************");
        RCLCPP_INFO(get_logger(), "**********************************************************");

        battery_level_["r2d2"] = 100.0;
      }

      if (!feedback.action_execution_status.empty() && battery_level_["c3po"] < 10.0) {
        problem_expert_->removePredicate(plansys2::Predicate("(battery_full c3po)"));
        RCLCPP_INFO(get_logger(), "**********************************************************");
        RCLCPP_INFO(get_logger(), "********************** BATTERY LOW c3po ******************");
        RCLCPP_INFO(get_logger(), "**********************************************************");

        battery_level_["c3po"] = 100.0;
      }

      if (!feedback.action_execution_status.empty() && battery_level_["bb8"] < 10.0) {
        problem_expert_->removePredicate(plansys2::Predicate("(battery_full bb8)"));
        RCLCPP_INFO(get_logger(), "**********************************************************");
        RCLCPP_INFO(get_logger(), "********************** BATTERY LOW bb8 *******************");
        RCLCPP_INFO(get_logger(), "**********************************************************");

        battery_level_["bb8"] = 100.0;
      }

      RCLCPP_INFO_STREAM(get_logger(), "------------------------------------------------------");
      for (const auto & action_info : feedback.action_execution_status) {
        std::string args;
        rclcpp::Time start_stamp = action_info.start_stamp;
        rclcpp::Time status_stamp = action_info.status_stamp;
        for (const auto & arg : action_info.arguments) {
          args = args + " " + arg;
        }

        RCLCPP_INFO_STREAM(
          get_logger(), "Action: " << action_info.action << args << " " <<
          status_to_string(action_info.status) << " " <<
          (status_stamp - start_stamp).seconds() << " secs");
      }
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::default_random_engine generator_;
  std::uniform_int_distribution<int> distribution_dish_;
  std::uniform_int_distribution<int> distribution_num_dishes_;

  std::map<std::string, std::string> robot_location_;
  std::map<std::string, double> battery_level_{{"r2d2", 100.0}, {"c3po", 50.0}, {"bb8", 75.0}};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Cooking>();

  node->init();

  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
  }

  rclcpp::shutdown();

  return 0;
}
