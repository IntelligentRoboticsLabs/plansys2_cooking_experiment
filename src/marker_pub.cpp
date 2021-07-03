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

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class MarkerPub : public rclcpp::Node
{
public:
  MarkerPub(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name),
    counter_(0)
  {
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("nav_markers", 10);
    timer_ = create_wall_timer(
      rate, std::bind(&MarkerPub::timer_callback, this));

    if (!has_parameter("waypoints")) {
      declare_parameter("waypoints");
    }

    if (has_parameter("waypoints")) {
      std::vector<std::string> wp_names;

      get_parameter_or("waypoints", wp_names, {});

      std::cerr << "waypoints: " << wp_names.size() << std::endl;

      for (auto & wp : wp_names) {
        if (!has_parameter("waypoint_coords." + wp)) {
          declare_parameter("waypoint_coords." + wp);
        }

        std::cerr << "waypoint: " << wp << std::endl;

        std::vector<double> coords;
        if (get_parameter_or("waypoint_coords." + wp, coords, {})) {
          geometry_msgs::msg::Pose pose;
          pose.position.x = coords[0];
          pose.position.y = coords[1];
          pose.position.z = 1.0;
          waypoints_[wp] = pose;
        } else {
          std::cerr << "No coordinate configured for waypoint [" << wp << "]" << std::endl;
        }
      }
    }
}



void timer_callback()
{


  int counter = 0;
  visualization_msgs::msg::MarkerArray msg;
  for (const auto & wp : waypoints_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.id = counter++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.text = wp.first;
    marker.pose = wp.second;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    msg.markers.push_back(marker);
  }
  pub_->publish(msg);
}

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  std::map<std::string, geometry_msgs::msg::Pose> waypoints_;
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MarkerPub>("marker_pub", 1s);

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
 
  executor.spin();
  
  rclcpp::shutdown();

  return 0;
}