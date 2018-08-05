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

#pragma once

#include "packml_sm/packml_stats_provider.h"

#include <ros/ros.h>
#include <packml_msgs/Stats.h>

namespace packml_sm
{
void statsToROSStatsMessage(const PackmlStatsProvider& stats_provider, packml_msgs::Stats& stats)
{
  stats.idle_duration.data.fromSec(stats_provider.idleDuration());
  stats.exe_duration.data.fromSec(stats_provider.executeDuration());
  stats.held_duration.data.fromSec(stats_provider.heldDuration());
  stats.susp_duration.data.fromSec(stats_provider.suspendedDuration());
  stats.cmplt_duration.data.fromSec(stats_provider.completedDuration());
  stats.stop_duration.data.fromSec(stats_provider.stoppedDuration());
  stats.abort_duration.data.fromSec(stats_provider.abortedDuration());

  stats.availability = stats_provider.availabilty();
  stats.cycle_count = stats_provider.cycleCount();
  stats.duration.data.fromSec(stats_provider.totalDuration());
  stats.fail_count = stats_provider.failureCount();
  stats.overall_equipment_effectiveness = stats_provider.overallEquipmentEffectiveness();
  stats.performance = stats_provider.performance();
  stats.quality = stats_provider.quality();
  stats.success_count = stats_provider.successCount();
  stats.throughput = stats_provider.throughput();

  stats.header.stamp = ros::Time::now();
}
}
