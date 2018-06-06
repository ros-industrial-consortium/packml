/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016 Shaun Edwards
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <functional>
#include <QtConcurrent/QtConcurrent>

#include "packml_sm/state.h"
#include "packml_sm/events.h"
#include "packml_sm/dlog.h"

namespace packml_sm
{
void PackmlState::onEntry(QEvent* e)
{
  DLog::LogDebug("Entering state: %s (%d)", name_.toStdString().c_str(), state_);
  emit stateEntered(static_cast<int>(state_), name_);
  enter_time_ = std::chrono::system_clock::now();
}

void PackmlState::onExit(QEvent* e)
{
  DLog::LogDebug("Exiting state: %s (%d)", name_.toStdString().c_str(), state_);
  exit_time_ = std::chrono::system_clock::now();
  cummulative_time_ = cummulative_time_ + (exit_time_ - enter_time_);

  DLog::LogDebug("Updating cummulative time, for state: %s (%d) to: %f", name_.toStdString().c_str(), state_,
                 cummulative_time_.count());
}

void ActingState::onEntry(QEvent* e)
{
  PackmlState::onEntry(e);
  DLog::LogDebug("Starting thread for state operation");
  function_state_ = QtConcurrent::run(std::bind(&ActingState::operation, this));
}

void ActingState::onExit(QEvent* e)
{
  if (function_state_.isRunning())
  {
    DLog::LogDebug("State exit triggered early, waitiing for state operation to complete");
  }

  function_state_.waitForFinished();
  PackmlState::onExit(e);
}

void ActingState::operation()
{
  QEvent* sc;
  if (function_)
  {
    DLog::LogDebug("Executing operational function in acting state");
    int error_code = function_();
    if (0 == error_code)
    {
      sc = new StateCompleteEvent();
    }
    else
    {
      DLog::LogError("Operational function returned error code: %d", error_code);
      sc = new ErrorEvent(error_code);
    }
  }
  else
  {
    DLog::LogDebug("Default operation, delaying %d ms", delay_ms);
    ros::WallDuration(delay_ms / 1000.0).sleep();
    DLog::LogDebug("Operation delay complete");
    sc = new StateCompleteEvent();
  }
  machine()->postEvent(sc);
}

}  // packml_sm
