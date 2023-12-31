// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_DIAGNOSTIC_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_DIAGNOSTIC_HPP_

// std
#include <string>

// romea
#include "path_matching_diagnostic_base.hpp"

namespace romea
{
namespace ros2
{

class DiagnosticPathStatus : public diagnostic_updater::DiagnosticTask
{
public:
  explicit DiagnosticPathStatus(const std::string & name);

  void setPathFilename(const std::string & filename);

  std::string getPathFilename();

  void setStatus(const bool & status);

  bool getStatus() const;

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

private:
  std::string filename_;
  std::string basename_;
  bool status_;
};


class PathMatchingDiagnostic : public PathMatchingDiagnosticBase
{
public:
  PathMatchingDiagnostic();

  void update_path_status(const std::string & filename, const bool & isOpened);

private:
  DiagnosticPathStatus path_status_diagnostic_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_DIAGNOSTIC_HPP_
