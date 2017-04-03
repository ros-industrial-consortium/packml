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


namespace packml_sm
{

void PackmlState::onEntry(QEvent *e)
{
  ROS_DEBUG_STREAM("Entering state: " << name_.toStdString() << "(" << state_ <<")");
  emit stateEntered(static_cast<int>(state_), name_);
  enter_time_ = ros::Time::now();
}


void PackmlState::onExit(QEvent *e)
{

  ROS_DEBUG_STREAM("Exiting state: " << name_.toStdString() << "(" << state_ << ")");
  exit_time_ = ros::Time::now();
  cummulative_time_ = cummulative_time_ + (exit_time_ - enter_time_);
  ROS_DEBUG_STREAM("Updating cummulative time, for state: " << name_.toStdString() << "("
                  << state_ << ") to: " << cummulative_time_.toSec());
}

void ActingState::onEntry(QEvent *e)
{
  PackmlState::onEntry(e);
  ROS_DEBUG_STREAM("Starting thread for state operation");
  function_state_ = QtConcurrent::run(std::bind(&ActingState::operation, this));
}

void ActingState::onExit(QEvent *e)
{
  if (function_state_.isRunning())
  {
    ROS_DEBUG_STREAM("State exit triggered early, waitiing for state operation to complete");
  }
  function_state_.waitForFinished();
  PackmlState::onExit(e);
}


void ActingState::operation()
{
  QEvent* sc;
  if( function_ )
  {
    ROS_DEBUG_STREAM("Executing operational function in acting state");
    int error_code = function_();
    if( 0 == error_code )
    {
      sc = new StateCompleteEvent();
    }
    else
    {
      ROS_ERROR_STREAM("Operational function returned error code: " << error_code);
      sc = new ErrorEvent(error_code);
    }
  }
  else
  {
    ROS_DEBUG_STREAM("Default operation, delaying " << delay_ms << " ms");
    ros::WallDuration(delay_ms/1000.0).sleep();
    ROS_DEBUG_STREAM("Operation delay complete");
    sc = new StateCompleteEvent();
  }
  machine()->postEvent(sc);
}

}//packml_sm
