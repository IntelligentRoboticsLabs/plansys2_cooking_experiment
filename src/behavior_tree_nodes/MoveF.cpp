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

#include "plansys2_cooking_example/behavior_tree_nodes/MoveF.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace plansys2_bt_example
{

std::string MoveF::goal_pos_ = "";  // NOLINT (runtime/string)

MoveF::MoveF(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
MoveF::halt()
{
}

BT::NodeStatus
MoveF::tick()
{
  std::string goal;
  getInput<std::string>("goal", goal);

  if (goal_pos_ != "" && goal_pos_ != goal) {
    std::cerr << "ERROR commanded [" << goal <<
      " while going to [" << goal_pos_ << "]" << std::endl;
  }

  goal_pos_ = goal;

  if (counter_++ < 18) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    goal_pos_ = "";
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_bt_example::MoveF>("MoveF");
}
