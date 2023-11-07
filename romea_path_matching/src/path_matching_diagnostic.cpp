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

// std
#include <string>

// romea
#include "romea_path_matching/path_matching_diagnostic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
DiagnosticPathStatus::DiagnosticPathStatus(const std::string & name)
: DiagnosticTask(name), filename_(), status_(false)
{
}

//-----------------------------------------------------------------------------
void DiagnosticPathStatus::setPathFilename(const std::string & filename)
{
  filename_ = filename;
  basename_ = filename_.substr(filename_.find_last_of('/') + 1);
}

//-----------------------------------------------------------------------------
std::string DiagnosticPathStatus::getPathFilename()
{
  return filename_;
}

//-----------------------------------------------------------------------------
void DiagnosticPathStatus::setStatus(const bool & status)
{
  status_ = status;
}

//-----------------------------------------------------------------------------
bool DiagnosticPathStatus::getStatus() const
{
  return status_;
}

//-----------------------------------------------------------------------------
void DiagnosticPathStatus::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (status_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Path is loaded");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unable to load path");
  }
  stat.add("file_path", filename_);
  stat.add("filename", basename_);
  stat.add("is_opened", status_);
}

//-----------------------------------------------------------------------------
DiagnosticLookupTransformStatus::DiagnosticLookupTransformStatus(const std::string & name)
: DiagnosticTask(name), status_(false)
{
}

//-----------------------------------------------------------------------------
void DiagnosticLookupTransformStatus::setStatus(const bool & status)
{
  status_ = status;
}

//-----------------------------------------------------------------------------
bool DiagnosticLookupTransformStatus::getStatus() const
{
  return status_;
}

//-----------------------------------------------------------------------------
void DiagnosticLookupTransformStatus::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!status_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Transformation world to path published");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "Transformation world to path not published ");
  }
  stat.add(" tf status ", status_);
}

//-----------------------------------------------------------------------------
PathMatchingDiagnostic::PathMatchingDiagnostic()
: path_status_diagnostic_("path_status"), lookup_transform_status_diagnostic_("lookup_tf_status")
{
  composite_diagnostic_.addTask(&path_status_diagnostic_);
  composite_diagnostic_.addTask(&lookup_transform_status_diagnostic_);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::update_lookup_transform_status(const bool & status)
{
  lookup_transform_status_diagnostic_.setStatus(status);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnostic::update_path_status(const std::string & filename, const bool & isOpened)
{
  path_status_diagnostic_.setPathFilename(filename);
  path_status_diagnostic_.setStatus(isOpened);
}

}  // namespace romea
