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
#ifndef PACKML_ROS_H
#define PACKML_ROS_H

#include <ros/ros.h>

#include <packml_msgs/GetStats.h>
#include <packml_msgs/ResetStats.h>
#include <packml_msgs/Transition.h>
#include <packml_msgs/Status.h>
#include <packml_sm/abstract_state_machine.h>

namespace packml_ros
{
class PackmlRos
{
public:
  PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn, std::shared_ptr<packml_sm::AbstractStateMachine> sm);
  ~PackmlRos();
  void spin();
  void spinOnce();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pn_;
  std::shared_ptr<packml_sm::AbstractStateMachine> sm_;
  ros::Publisher status_pub_;
  ros::ServiceServer trans_server_;
  ros::ServiceServer reset_stats_server_;
  ros::ServiceServer get_stats_server_;
  packml_msgs::Status status_msg_;

  bool transRequest(packml_msgs::Transition::Request& req, packml_msgs::Transition::Response& res);

private:
  void handleStateChanged(packml_sm::AbstractStateMachine& state_machine, const packml_sm::StateChangedEventArgs& args);
  void getCurrentStats(packml_msgs::Stats& out_stats);
  bool getStats(packml_msgs::GetStats::Request& req, packml_msgs::GetStats::Response& response);
  bool resetStats(packml_msgs::ResetStats::Request& req, packml_msgs::ResetStats::Response& response);
};
}  // namespace packml_ros

#endif  // PACKML_ROS_H
