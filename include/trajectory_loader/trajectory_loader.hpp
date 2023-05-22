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

#ifndef TRAJECTORY_LOADER__TRAJECTORY_LOADER_HPP_
#define TRAJECTORY_LOADER__TRAJECTORY_LOADER_HPP_

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "trajectory_loader/visibility_control.hpp"


namespace trajectory_loader
{
using Table = std::vector<std::vector<std::string>>;
using Points = std::vector<std::vector<double>>;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
class TRAJECTORY_LOADER_PUBLIC TrajectoryLoader
{
public:
  TrajectoryLoader(
    std::string csv_path, std::string delimiter, bool is_header, size_t col_x, size_t col_y,
    size_t col_yaw, size_t col_vel
  );
  bool readCSV(Table & result);
  bool validateData(const Table & table, const std::string & csv_path);
  Points getPoints(const Table & table);

private:
  std::string csv_path_;
  char delimiter_ {';'};
  size_t idx_begin_ {0};
  size_t col_x_ {0};
  size_t col_y_ {1};
  size_t col_yaw_ {2};
  size_t col_vel_ {3};
};

}  // namespace trajectory_loader

#endif  // TRAJECTORY_LOADER__TRAJECTORY_LOADER_HPP_
