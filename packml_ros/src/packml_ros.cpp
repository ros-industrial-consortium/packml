/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
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

#include "packml_ros/packml_ros.h"
#include "packml_sm/packml_stats_snapshot.h"

#include <packml_msgs/utils.h>
#include "packml_msgs/ItemizedStats.h"

namespace packml_ros
{
PackmlRos::PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn, std::shared_ptr<packml_sm::AbstractStateMachine> sm)
  : nh_(nh), pn_(pn), sm_(sm)
{
  ros::NodeHandle packml_node("~/packml");

  status_pub_ = packml_node.advertise<packml_msgs::Status>("status", 10, true);

  trans_server_ = packml_node.advertiseService("transition", &PackmlRos::transRequest, this);
  reset_stats_server_ = packml_node.advertiseService("reset_stats", &PackmlRos::resetStats, this);
  get_stats_server_ = packml_node.advertiseService("get_stats", &PackmlRos::getStats, this);

  status_msg_ = packml_msgs::initStatus(pn.getNamespace());

  sm_->stateChangedEvent.bind_member_func(this, &PackmlRos::handleStateChanged);
  sm_->activate();
}

PackmlRos::~PackmlRos()
{
  if (sm_ != nullptr)
  {
    sm_->stateChangedEvent.unbind_member_func(this, &PackmlRos::handleStateChanged);
  }
}

void PackmlRos::spin()
{
  while (ros::ok())
  {
    spinOnce();
    ros::Duration(0.001).sleep();
  }
  return;
}

void PackmlRos::spinOnce()
{
  ros::spinOnce();
}

bool PackmlRos::transRequest(packml_msgs::Transition::Request& req, packml_msgs::Transition::Response& res)
{
  bool command_rtn = false;
  bool command_valid = true;
  int command_int = static_cast<int>(req.command);
  std::stringstream ss;
  ROS_DEBUG_STREAM("Evaluating transition request command: " << command_int);

  switch (command_int)
  {
    case req.ABORT:
    case req.ESTOP:
      command_rtn = sm_->abort();
      break;
    case req.CLEAR:
      command_rtn = sm_->clear();
      break;
    case req.HOLD:
      command_rtn = sm_->hold();
      break;
    case req.RESET:
      command_rtn = sm_->reset();
      break;
    case req.START:
      command_rtn = sm_->start();
      break;
    case req.STOP:
      command_rtn = sm_->stop();
      break;
    case req.SUSPEND:
      command_rtn = sm_->suspend();
      break;
    case req.UNHOLD:
      command_rtn = sm_->unhold();
      break;
    case req.UNSUSPEND:
      command_rtn = sm_->unsuspend();
      break;

    default:
      command_valid = false;
      break;
  }
  if (command_valid)
  {
    if (command_rtn)
    {
      ss << "Successful transition request command: " << command_int;
      ROS_INFO_STREAM(ss.str());
      res.success = true;
      res.error_code = res.SUCCESS;
      res.message = ss.str();
    }
    else
    {
      ss << "Invalid transition request command: " << command_int;
      ROS_ERROR_STREAM(ss.str());
      res.success = false;
      res.error_code = res.INVALID_TRANSITION_REQUEST;
      res.message = ss.str();
    }
  }
  else
  {
    ss << "Unrecognized transition request command: " << command_int;
    ROS_ERROR_STREAM(ss.str());
    res.success = false;
    res.error_code = res.UNRECOGNIZED_REQUEST;
    res.message = ss.str();
  }
}

void PackmlRos::handleStateChanged(packml_sm::AbstractStateMachine& state_machine,
                                   const packml_sm::StateChangedEventArgs& args)
{
  ROS_DEBUG_STREAM("Publishing state change: " << args.name << "(" << args.value << ")");

  status_msg_.header.stamp = ros::Time().now();
  int cur_state = static_cast<int>(args.value);
  if (packml_msgs::isStandardState(cur_state))
  {
    status_msg_.state.val = cur_state;
    status_msg_.sub_state = packml_msgs::State::UNDEFINED;
  }
  else
  {
    status_msg_.sub_state = cur_state;
  }

  status_pub_.publish(status_msg_);
}

void PackmlRos::getCurrentStats(packml_msgs::Stats& out_stats)
{
  packml_sm::PackmlStatsSnapshot stats_snapshot;
  sm_->getCurrentStatSnapshot(stats_snapshot);

  out_stats.idle_duration.data.fromSec(stats_snapshot.idle_duration);
  out_stats.exe_duration.data.fromSec(stats_snapshot.exe_duration);
  out_stats.held_duration.data.fromSec(stats_snapshot.held_duration);
  out_stats.susp_duration.data.fromSec(stats_snapshot.susp_duration);
  out_stats.cmplt_duration.data.fromSec(stats_snapshot.cmplt_duration);
  out_stats.stop_duration.data.fromSec(stats_snapshot.stop_duration);
  out_stats.abort_duration.data.fromSec(stats_snapshot.abort_duration);
  out_stats.duration.data.fromSec(stats_snapshot.duration);
  out_stats.fail_count = stats_snapshot.fail_count;
  out_stats.success_count = stats_snapshot.success_count;
  out_stats.availability = stats_snapshot.availability;
  out_stats.performance = stats_snapshot.performance;
  out_stats.quality = stats_snapshot.quality;
  out_stats.overall_equipment_effectiveness = stats_snapshot.overall_equipment_effectiveness;

  out_stats.error_items.clear();
  for (const auto& itemized_it : stats_snapshot.itemized_error_map)
  {
    packml_msgs::ItemizedStats stat;
    stat.id = itemized_it.second.id;
    stat.count = itemized_it.second.count;
    stat.duration.data.fromSec(itemized_it.second.duration);
    out_stats.error_items.push_back(stat);
  }

  out_stats.quality_items.clear();
  for (const auto& itemized_it : stats_snapshot.itemized_quality_map)
  {
    packml_msgs::ItemizedStats stat;
    stat.id = itemized_it.second.id;
    stat.count = itemized_it.second.count;
    stat.duration.data.fromSec(itemized_it.second.duration);
    out_stats.quality_items.push_back(stat);
  }

  out_stats.header.stamp = ros::Time::now();
}

bool PackmlRos::getStats(packml_msgs::GetStats::Request& req, packml_msgs::GetStats::Response& response)
{
  packml_msgs::Stats stats;
  getCurrentStats(stats);
  response.stats = stats;

  return true;
}

bool PackmlRos::resetStats(packml_msgs::ResetStats::Request& req, packml_msgs::ResetStats::Response& response)
{
  packml_msgs::Stats stats;
  getCurrentStats(stats);
  response.last_stat = stats;

  sm_->resetStats();

  return true;
}
}  // namespace kitsune_robot
