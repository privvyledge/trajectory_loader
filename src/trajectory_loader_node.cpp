// Copyright 2023 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_loader/trajectory_loader_node.hpp"

namespace trajectory_loader
{
using namespace std::placeholders;

TrajectoryLoaderNode::TrajectoryLoaderNode(const rclcpp::NodeOptions & options)
:  Node("trajectory_loader", options)
{
  const double update_rate = this->declare_parameter("update_rate", 10.0);
  const std::string csv_path = this->declare_parameter("csv_path", "");
  const std::string delimiter = this->declare_parameter("delimiter", ",");
  const bool is_header = this->declare_parameter("is_header", false);
  const size_t col_x = this->declare_parameter("col_x", 0);
  const size_t col_y = this->declare_parameter("col_y", 1);
  const size_t col_yaw = this->declare_parameter("col_yaw", 2);
  const size_t col_vel = this->declare_parameter("col_vel", 3);

  trajectory_loader_ = std::make_unique<trajectory_loader::TrajectoryLoader>(
    csv_path, delimiter, is_header, col_x, col_y, col_yaw, col_vel);

  std::vector<std::vector<std::string>> table;

  if (!trajectory_loader_->readCSV(table)) {
    RCLCPP_ERROR(get_logger(), "Cannot read trajectory. csv path = %s.", csv_path.c_str());
    throw std::runtime_error("Shutting down trajectory loader.");
  }

  auto points = trajectory_loader_->getPoints(table);
  trajectory_ = createTrajectory(points);

  rclcpp::QoS qos{1};
  qos.transient_local();
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);

  std::cout << update_rate;
  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ =
    rclcpp::create_timer(
    this, get_clock(), period_ns,
    std::bind(&TrajectoryLoaderNode::onTimer, this));
}

Trajectory TrajectoryLoaderNode::createTrajectory(const Points & points)
{
  Trajectory trajectory;

  trajectory.header.frame_id = "map";

  for (auto & point : points) {
    TrajectoryPoint trajectory_point;
    Pose pose;
    auto q =  autoware_universe_utils::createQuaternionFromYaw(point[2]);
    pose.position.x = point[0];
    pose.position.y = point[1];
    pose.position.z = 0.0;
    pose.orientation = q;

    trajectory_point.pose = pose;

    trajectory_point.longitudinal_velocity_mps = point[3];

    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

void TrajectoryLoaderNode::onTimer()
{
  trajectory_.header.stamp = this->now();
  trajectory_pub_->publish(trajectory_);
}

}  // namespace trajectory_loader

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_loader::TrajectoryLoaderNode)
