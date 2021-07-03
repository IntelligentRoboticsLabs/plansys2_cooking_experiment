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

#include <string>
#include <iostream>
#include <memory>

#include "plansys2_cooking_example/behavior_tree_nodes/Recharge.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2_cooking_example
{

Recharge::Recharge(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
}

void
Recharge::halt()
{
  std::cout << "STARTING Recharge " << std::endl;
}

BT::NodeStatus
Recharge::tick()
{
  std::cout << "Recharge " << counter_ << "/ 100" << std::endl;

  if (counter_++ < 100) {
    return BT::NodeStatus::RUNNING;
  } else {
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full r2d2)"));
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_cooking_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_cooking_example::Recharge>("Recharge");
}
