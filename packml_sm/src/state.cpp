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


#include "packml_sm/state.h"
#include "packml_sm/events.h"

namespace packml_sm
{

void PackmlState::onEntry(QEvent *e)
{
  ROS_INFO_STREAM("Entering state: " << name_.toStdString() << "(" << state_ <<")");
  emit stateEntered(static_cast<int>(state_), name_);
  enter_time_ = ros::Time::now();
  operation();
}


void PackmlState::onExit(QEvent *e)
{
  ROS_INFO_STREAM("Exiting state: " << name_.toStdString() << "(" << state_ << ")");
  exit_time_ = ros::Time::now();
  cummulative_time_ = cummulative_time_ + (exit_time_ - enter_time_);
  ROS_INFO_STREAM("Updating cummulative time, for state: " << name_.toStdString() << "("
                  << state_ << ") to: " << cummulative_time_.toSec());
}


void ActingState::operation()
{
  StateCompleteEvent* sc = new StateCompleteEvent();
  ROS_INFO_STREAM("Default operation, delaying " << delay_ms << " ms");
  ros::WallDuration(delay_ms/1000.0).sleep();
  ROS_INFO_STREAM("Operation delay complete");
  machine()->postEvent(sc);
}



}
