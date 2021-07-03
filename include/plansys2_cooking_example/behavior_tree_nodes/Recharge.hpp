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

#ifndef PLANSYS2_COOKING_EXAMPLE__BEHAVIOR_TREE_NODES__RECHARGE_HPP_
#define PLANSYS2_COOKING_EXAMPLE__BEHAVIOR_TREE_NODES__RECHARGE_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2_cooking_example
{

class Recharge : public BT::ActionNodeBase
{
public:
  explicit Recharge(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
};

}  // namespace plansys2_cooking_example

#endif  // PLANSYS2_COOKING_EXAMPLE__BEHAVIOR_TREE_NODES__RECHARGE_HPP_
