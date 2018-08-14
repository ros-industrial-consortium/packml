/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Plus One Robotics
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

#include "packml_sm/abstract_state_machine.h"
#include "packml_sm/common.h"
#include "packml_sm/dlog.h"

#include <thread>

namespace packml_sm
{
AbstractStateMachine::AbstractStateMachine() : start_time_(std::chrono::steady_clock::now())
{
}

bool AbstractStateMachine::start()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::IDLE:
      _start();
      return true;
    default:
      DLog::LogWarning("Ignoring START command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::clear()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTED:
      _clear();
      return true;
    default:
      DLog::LogWarning("Ignoring CLEAR command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::reset()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::COMPLETE:
    case StatesEnum::STOPPED:
      _reset();
      return true;
    default:
      DLog::LogWarning("Ignoring RESET command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::hold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _hold();
      return true;
    default:
      DLog::LogWarning("Ignoring HOLD command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::unhold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::HELD:
      _unhold();
      return true;
    default:
      DLog::LogWarning("Ignoring HELD command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::suspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _suspend();
      return true;
    default:
      DLog::LogWarning("Ignoring SUSPEND command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::unsuspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::SUSPENDED:
      _unsuspend();
      return true;
    default:
      DLog::LogWarning("Ignoring UNSUSPEND command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::stop()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::STOPPABLE:
    case StatesEnum::STARTING:
    case StatesEnum::IDLE:
    case StatesEnum::SUSPENDED:
    case StatesEnum::EXECUTE:
    case StatesEnum::HOLDING:
    case StatesEnum::HELD:
    case StatesEnum::SUSPENDING:
    case StatesEnum::UNSUSPENDING:
    case StatesEnum::UNHOLDING:
    case StatesEnum::COMPLETING:
    case StatesEnum::COMPLETE:
      _stop();
      return true;
    default:
      DLog::LogWarning("Ignoring STOP command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::abort()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTABLE:
    case StatesEnum::STOPPED:
    case StatesEnum::STARTING:
    case StatesEnum::IDLE:
    case StatesEnum::SUSPENDED:
    case StatesEnum::EXECUTE:
    case StatesEnum::HOLDING:
    case StatesEnum::HELD:
    case StatesEnum::SUSPENDING:
    case StatesEnum::UNSUSPENDING:
    case StatesEnum::UNHOLDING:
    case StatesEnum::COMPLETING:
    case StatesEnum::COMPLETE:
    case StatesEnum::CLEARING:
    case StatesEnum::STOPPING:
      _abort();
      return true;
    default:
      DLog::LogWarning("Ignoring ABORT command in current state: %d", getCurrentState());
      return false;
  }
}

double AbstractStateMachine::getIdleTime()
{
  return getStateDuration(StatesEnum::IDLE);
}

double AbstractStateMachine::getStartingTime()
{
  return getStateDuration(StatesEnum::STARTING);
}

double AbstractStateMachine::getResettingTime()
{
  return getStateDuration(StatesEnum::RESETTING);
}

double AbstractStateMachine::getExecuteTime()
{
  return getStateDuration(StatesEnum::EXECUTE);
}

double AbstractStateMachine::getHeldTime()
{
  return getStateDuration(StatesEnum::HELD);
}

double AbstractStateMachine::getHoldingTime()
{
  return getStateDuration(StatesEnum::HOLDING);
}

double AbstractStateMachine::getUnholdingTime()
{
  return getStateDuration(StatesEnum::UNHOLDING);
}

double AbstractStateMachine::getSuspendedTime()
{
  return getStateDuration(StatesEnum::SUSPENDED);
}

double AbstractStateMachine::getSuspendingTime()
{
  return getStateDuration(StatesEnum::SUSPENDING);
}

double AbstractStateMachine::getUnsuspendingTime()
{
  return getStateDuration(StatesEnum::UNSUSPENDING);
}

double AbstractStateMachine::getCompleteTime()
{
  return getStateDuration(StatesEnum::COMPLETE);
}

double AbstractStateMachine::getStoppedTime()
{
  return getStateDuration(StatesEnum::STOPPED);
}

double AbstractStateMachine::getClearingTime()
{
  return getStateDuration(StatesEnum::CLEARING);
}

double AbstractStateMachine::getStoppingTime()
{
  return getStateDuration(StatesEnum::STOPPING);
}

double AbstractStateMachine::getAbortedTime()
{
  return getStateDuration(StatesEnum::ABORTED);
}

double AbstractStateMachine::getAbortingTime()
{
  return getStateDuration(StatesEnum::ABORTING);
}

double AbstractStateMachine::getTotalTime()
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  std::chrono::duration<double> duration = std::chrono::steady_clock::now() - start_time_;
  auto elapsed_time = duration.count();
  for (auto iter = duration_map_.begin(); iter != duration_map_.end(); iter++)
  {
    elapsed_time += iter->second;
  }

  return elapsed_time;
}

double AbstractStateMachine::calculateAvailability()
{
  auto scheduled_time = getTotalTime();
  auto operating_time = scheduled_time - getAbortedTime() - getHeldTime() - getSuspendedTime();
  if (scheduled_time > std::numeric_limits<double>::epsilon())
  {
    return operating_time / scheduled_time;
  }

  return 0;
}

double AbstractStateMachine::calculateQuality()
{
  auto failure_count = getFailureCount();
  auto success_count = getSuccessCount();
  auto total_count = failure_count + success_count;
  if (total_count > 0)
  {
    return (double)success_count / (double)total_count;
  }

  return 0;
}

void AbstractStateMachine::resetStats()
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  start_time_ = std::chrono::steady_clock::now();
  duration_map_.clear();
}

int AbstractStateMachine::getFailureCount() const
{
  return failure_count_;
}

int AbstractStateMachine::getSuccessCount() const
{
  return success_count_;
}

void AbstractStateMachine::invokeStateChangedEvent(const std::string& name, StatesEnum value)
{
  updateClock(value);
  stateChangedEvent.invoke(*this, { name, value });
}

void AbstractStateMachine::updateClock(StatesEnum new_state)
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  std::chrono::duration<double> duration = std::chrono::steady_clock::now() - start_time_;
  auto elapsed_time = duration.count();
  if (duration_map_.find(current_state_) != duration_map_.end())
  {
    elapsed_time += duration_map_[current_state_];
  }

  duration_map_[current_state_] = elapsed_time;

  current_state_ = new_state;
  start_time_ = std::chrono::steady_clock::now();
}

double AbstractStateMachine::getStateDuration(StatesEnum state)
{
  std::lock_guard<std::mutex> lock(stat_mutex_);
  double elapsed_time = 0;
  if (state == current_state_)
  {
    std::chrono::duration<double> duration = std::chrono::steady_clock::now() - start_time_;
    elapsed_time += duration.count();
  }

  if (duration_map_.find(state) != duration_map_.end())
  {
    elapsed_time += duration_map_[state];
  }

  return elapsed_time;
}
}
