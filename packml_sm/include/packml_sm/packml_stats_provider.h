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

#ifndef PACKML_STATS_PROVIDER_H
#define PACKML_STATS_PROVIDER_H

#include "packml_sm/abstract_state_machine.h"

#include <memory>
#include <chrono>

namespace packml_sm
{
class PackmlStatsProvider
{
public:
  explicit PackmlStatsProvider(std::shared_ptr<AbstractStateMachine> state_machine);

  void start();
  void reset();

  void setFaulted(bool is_faulted);
  void incrementCycleCount(bool success);
  void setTargetRate(float target_rate);

  double totalDuration() const;
  double idleDuration() const;
  double executeDuration() const;
  double heldDuration() const;
  double suspendedDuration() const;
  double completedDuration() const;
  double stoppedDuration() const;
  double abortedDuration() const;

  int cycleCount() const;
  int successCount() const;
  int failureCount() const;

  float throughput() const;

  float availabilty() const;
  float performance() const;
  float quality() const;
  float overallEquipmentEffectiveness() const;

private:
  std::shared_ptr<AbstractStateMachine> state_machine_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point fault_start_time_;
  bool is_faulted_ = false;
  int success_count_ = 0;
  int failure_count_ = 0;
  float target_rate_;
  double fault_duration_ = 0.0;
};
}

#endif  // PACKML_STATS_PROVIDER_H
