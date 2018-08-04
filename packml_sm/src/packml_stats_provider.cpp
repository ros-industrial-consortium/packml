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

#include "packml_sm/packml_stats_provider.h"

#include <limits>

namespace packml_sm
{
PackmlStatsProvider::PackmlStatsProvider(std::shared_ptr<AbstractStateMachine> state_machine)
  : state_machine_(state_machine)
{
}

void PackmlStatsProvider::start()
{
  start_time_ = std::chrono::steady_clock::now();
}

void PackmlStatsProvider::reset()
{
  fault_duration_ = 0;
  success_count_ = 0;
  failure_count_ = 0;

  if (is_faulted_)
  {
    fault_start_time_ = std::chrono::steady_clock::now();
  }

  state_machine_->resetStats();

  if (state_machine_->isActive())
  {
    start();
  }
}

void PackmlStatsProvider::incrementCycleCount(bool success)
{
  if (success)
  {
    success_count_++;
  }
  else
  {
    failure_count_++;
  }
}

void PackmlStatsProvider::setFaulted(bool is_faulted)
{
  if (is_faulted)
  {
    if (!is_faulted_)
    {
      fault_start_time_ = std::chrono::steady_clock::now();
    }
  }
  else
  {
    if (is_faulted_)
    {
      std::chrono::duration<double> delta = std::chrono::steady_clock::now() - fault_start_time_;
      fault_duration_ += delta.count();
    }
  }

  is_faulted_ = is_faulted;
}

void PackmlStatsProvider::setTargetRate(float target_rate)
{
  target_rate_ = target_rate;
}

double PackmlStatsProvider::totalDuration() const
{
  auto end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> delta = end_time - start_time_;

  return delta.count();
}

double PackmlStatsProvider::idleDuration() const
{
  return state_machine_->getIdleTime();
}

double PackmlStatsProvider::executeDuration() const
{
  return state_machine_->getExecuteTime();
}

double PackmlStatsProvider::heldDuration() const
{
  return state_machine_->getHeldTime();
}

double PackmlStatsProvider::suspendedDuration() const
{
  return state_machine_->getSuspendedTime();
}

double PackmlStatsProvider::completedDuration() const
{
  return state_machine_->getCompleteTime();
}

double PackmlStatsProvider::stoppedDuration() const
{
  return state_machine_->getStoppedTime();
}

double PackmlStatsProvider::abortedDuration() const
{
  return state_machine_->getAbortedTime();
}

int PackmlStatsProvider::cycleCount() const
{
  return success_count_ + failure_count_;
}

int PackmlStatsProvider::successCount() const
{
  return success_count_;
}

int PackmlStatsProvider::failureCount() const
{
  return failure_count_;
}

float PackmlStatsProvider::throughput() const
{
  auto duration = totalDuration();
  if (duration > std::numeric_limits<double>::epsilon())
  {
    return static_cast<float>(cycleCount()) / static_cast<float>(duration);
  }

  return 0.0f;
}

float PackmlStatsProvider::availabilty() const
{
  auto fault_duration = fault_duration_;

  if (is_faulted_)
  {
    std::chrono::duration<double> delta = std::chrono::steady_clock::now() - fault_start_time_;
    fault_duration += delta.count();
  }

  if (fault_duration > std::numeric_limits<double>::epsilon())
  {
    auto executing = totalDuration() - fault_duration;
    return static_cast<float>(executing) / static_cast<float>(fault_duration);
  }

  return 0.0f;
}

float PackmlStatsProvider::performance() const
{
  if (cycleCount() != 0)
  {
    return target_rate_ / static_cast<float>(cycleCount());
  }

  return 0.0f;
}

float PackmlStatsProvider::quality() const
{
  if (failure_count_ != 0)
  {
    return static_cast<float>(success_count_) / static_cast<float>(failure_count_);
  }

  return 0.0f;
}

float PackmlStatsProvider::overallEquipmentEffectiveness() const
{
  if (quality() > std::numeric_limits<float>::epsilon())
  {
    return performance() / quality();
  }

  return 0.0f;
}
}
