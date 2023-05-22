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

#include "trajectory_loader/trajectory_loader.hpp"

#include <iostream>

namespace trajectory_loader
{

TrajectoryLoader::TrajectoryLoader(
  std::string csv_path, std::string delimiter, bool is_header,
  size_t col_x, size_t col_y, size_t col_yaw, size_t col_vel)
{
  csv_path_ = csv_path;
  delimiter_ = delimiter.c_str()[0];
  idx_begin_ = is_header ? 1 : 0;
  col_x_ = col_x;
  col_y_ = col_y;
  col_yaw_ = col_yaw;
  col_vel_ = col_vel;
}


bool TrajectoryLoader::readCSV(Table & result)
{
  std::ifstream ifs(csv_path_);
  if (!ifs.is_open()) {
    std::cerr << "Cannot open " << csv_path_.c_str() << std::endl;
    return false;
  }

  std::string buf;
  while (std::getline(ifs, buf)) {
    std::vector<std::string> tokens;

    std::istringstream iss(buf);
    std::string token;
    while (std::getline(iss, token, delimiter_)) {
      tokens.push_back(token);
    }

    if (tokens.size() != 0) {
      result.push_back(tokens);
    }
  }
  if (!validateData(result, csv_path_)) {
    return false;
  }
  return true;
}

bool TrajectoryLoader::validateData(const Table & table, const std::string & csv_path)
{
  if (table[0].size() < 3) {
    std::cerr << "Cannot read " << csv_path.c_str() << " CSV file should have at least 3 column"
              << std::endl;
    return false;
  }
  // validate trajectory size
  for (size_t i = idx_begin_; i < table.size(); i++) {
    // validate row size
    if (table[0].size() != table[i].size()) {
      std::cerr << "Cannot read " << csv_path.c_str()
                << ". Each row should have a same number of columns" << std::endl;
      return false;
    }
  }
  return true;
}

Points TrajectoryLoader::getPoints(const Table & table)
{
  Points points = {};
  for (size_t i = idx_begin_; i < table.size(); i++) {
    std::vector<double> point;
    point.push_back(std::stod(table[i][col_x_]));
    point.push_back(std::stod(table[i][col_y_]));
    point.push_back(std::stod(table[i][col_yaw_]));
    point.push_back(std::stod(table[i][col_vel_]));
    points.push_back(point);
  }
  return points;
}

}  // namespace trajectory_loader
